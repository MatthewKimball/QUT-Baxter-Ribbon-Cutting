#!/usr/bin/python
#----------------------------------------------------------------------------------
#----------------------------- QUT Ribbon Cutting ---------------------------------
# Filename: pick_place_new.py
# Author:	Matthew Kimball
# Email:	mp.kimball@gmail.com
# Date:		11/01/15
#
# Summary: This code was designed for the intention of cutting a ribbon at the unveiling
#			of the ACRV (Research Centre for Robotic Vision) using the Baxter Robot. 
#----------------------------------------------------------------------------------

import collections

from copy import deepcopy

import rospy
import sys
import tf
import cv2
import thread

from cv_bridge import CvBridge, CvBridgeError
import rospkg
import tf

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header

from sensor_msgs.msg import (
    Image,
    JointState,
)

import baxter_interface

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


class PickPlace(object):
    def __init__(self):
        self._rp = rospkg.RosPack()
        self.right_limb = 'right'
        self.left_limb = 'left'
        self._side = self.right_limb
        self._limb = baxter_interface.Limb(self.right_limb)
        self._left_limb = baxter_interface.Limb(self.left_limb)
        self._path = self._rp.get_path('ribbon_cut') + '/config/'
        self._images = (self._rp.get_path('ribbon_cut') +
                          '/share/images')
        dash_io = baxter_interface.DigitalIO(self.right_limb + '_upper_button')
        circle_io = baxter_interface.DigitalIO(self.right_limb + '_lower_button')

        self._done = False
        self._limbs = ('left', 'right')
        self._arms = {
            'left': baxter_interface.Limb('left'),
            'right': baxter_interface.Limb('right'),
            }
        self._tuck_rate = rospy.Rate(20.0)  # Hz
        self._tuck_threshold = 0.2  # radians
        self._peak_angle = -1.6  # radians
        self._arm_state = {
                           'tuck': {'left': 'none', 'right': 'none'},
                           'collide': {'left': False, 'right': False},
                           'flipped': {'left': False, 'right': False}
                          }
        self._joint_moves = {
            'tuck': {
                     'left':  [-1.0, -2.07,  3.0, 2.55,  0.0, 0.01,  0.0],
                     'right':  [1.0, -2.07, -3.0, 2.55, -0.0, 0.01,  0.0]
                     },
            'untuck': {
                       'left':  [-0.08, -1.0, -1.19, 1.94,  0.67, 1.03, -0.50],
                       'right':  [0.08, -1.0,  1.19, 1.94, -0.67, 1.03,  0.50]
                       }
            }

        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self.ribbon_location = dict()
        self.cut_location = dict()
        self.left_home_jp = dict()
        self.right_home_jp = dict()
        self.think_jp = dict()
        self.shake_head_jp = dict()
        self.shake_away_jp = dict()
        self.safe_jp = dict()
        self.ribbon_approach = dict()
        self.cut_location = dict()
        self.cut_approach = dict()
        self.bridge = CvBridge()
        self._gripper = baxter_interface.Gripper(self.right_limb, baxter_interface.CHECK_VERSION)

		# connect callback fns to signals
        if self._gripper.type() != 'custom':
            if not (self._gripper.calibrated() or
                    self._gripper.calibrate() == True):
                rospy.logwarn("%s (%s) calibration failed.",
                              self._gripper.name.capitalize(),
                              self._gripper.type())
        else:
            msg = (("%s (%s) not capable of gripper commands."
                   " Running cuff-light connection only.") %
                   (self._gripper.name.capitalize(), self._gripper.type()))
            rospy.logwarn(msg)

        ik_srv = "ExternalTools/" + self.right_limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ik_srv, SolvePositionIK)
        self._ikreq = SolvePositionIKRequest()

        circle_io.state_changed.connect(self._path_positions)

#----------------------------------------------------------------------------------
#----------------------------- Initialisation Functions----------------------------
#----------------------------------------------------------------------------------

	def _check_calibration(self, value):
		if self._gripper.calibrated():
		    return True
		elif value == 'electric':
		    rospy.loginfo("calibrating %s...",
		                  self._gripper.name.capitalize())
		    return (self._gripper.calibrate() == True)
		else:
		    return False

    def _check_arm_state(self):
        """
        Check for goals and behind collision field.

        If s1 joint is over the peak, collision will need to be disabled
        to get the arm around the head-arm collision force-field.
        """
        diff_check = lambda a, b: abs(a - b) <= self._tuck_threshold
        for limb in self._limbs:
            angles = [self._arms[limb].joint_angle(joint)
                      for joint in self._arms[limb].joint_names()]

            # Check if in a goal position
            untuck_goal = map(diff_check, angles,
                              self._joint_moves['untuck'][limb])
            tuck_goal = map(diff_check, angles[0:2],
                            self._joint_moves['tuck'][limb][0:2])
            if all(untuck_goal):
                self._arm_state['tuck'][limb] = 'untuck'
            elif all(tuck_goal):
                self._arm_state['tuck'][limb] = 'tuck'
            else:
                self._arm_state['tuck'][limb] = 'none'

            # Check if shoulder is flipped over peak
            self._arm_state['flipped'][limb] = (
                self._arms[limb].joint_angle(limb + '_s1') <= self._peak_angle) 
 
#----------------------------------------------------------------------------------
#----------------------------- Limb Control Functions------------------------------
#---------------------------------------------------------------------------------- 
    
    def _move_to(self, tuck, disabled):

        while (any(self._arm_state['tuck'][limb] != goal
                   for limb, goal in tuck.viewitems())
               and not rospy.is_shutdown()):

            for limb in self._limbs:
                if disabled[limb]:
                    self._disable_pub[limb].publish(Empty())
                if limb in tuck:
                    self._arms[limb].set_joint_positions(dict(zip(
                                      self._arms[limb].joint_names(),
                                      self._joint_moves[tuck[limb]][limb])))
            self._check_arm_state()
            self._tuck_rate.sleep()

        if any(self._arm_state['collide'].values()):
            self._rs.disable()
        return
        
    def _find_jp(self, pose):
        ikreq = SolvePositionIKRequest()

        goal_pose = Pose()
        goal_pose.position = pose['position']
        goal_pose.orientation = pose['orientation']

        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose_req = PoseStamped(header=hdr, pose=goal_pose)
        ikreq.pose_stamp.append(pose_req)
        resp = self._iksvc(ikreq)
        return dict(zip(resp.joints[0].name, resp.joints[0].position))

    def _find_approach(self, pose, offset):
        ikreq = SolvePositionIKRequest()
        # Add 5 cm offset in Z direction
        try:
            pose['position'] = Point(x=pose['position'][0],
                                     y=pose['position'][1],
                                     z=pose['position'][2] + offset
                                     )
        except Exception:
            pose['position'] = Point(x=pose['position'].x,
                                     y=pose['position'].y,
                                     z=pose['position'].z + offset
                                     )
        approach_pose = Pose()
        approach_pose.position = pose['position']
        approach_pose.orientation = pose['orientation']

        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose_req = PoseStamped(header=hdr, pose=approach_pose)
        ikreq.pose_stamp.append(pose_req)
        resp = self._iksvc(ikreq)
        return dict(zip(resp.joints[0].name, resp.joints[0].position))

    def _path_positions(self, value):
        if value:
            if len(self.safe_jp) == 0:
                # Record Safe Location
                print 'Recording safe location'
                self.safe_jp = self._limb.joint_angles()
            elif len(self.ribbon_approach) == 0:
                # Record Pick Location
                print 'Recording ribbon location'
                self.ribbon_location = self._limb.joint_angles()
                self.ribbon_approach = self._find_approach(
                                         self._limb.endpoint_pose(),
                                         0.05)
            elif len(self.cut_approach) == 0:
                # Record Cut Location
                print 'Cut ribbon location'
                self.cut_location = self._limb.joint_angles()
                self.cut_approach = self._find_approach(
                                         self._limb.endpoint_pose(),
                                         0.05)

#----------------------------------------------------------------------------------
#----------------------------- File IO Functions ----------------------------------
#----------------------------------------------------------------------------------
    def _read_file(self, file):
        with open(file, 'r') as f:
            for line in f:
                split = line.split('=')
                location = split[0]
                prefix = location.split("_")[0]
                position = split[1]
                if location == 'ribbon_location':
                    self.ribbon_location = eval(position)
                elif location == 'cut_location':
                    self.cut_location = eval(position)
                elif location == 'safe':
                    self.safe_jp = eval(position)
                elif location == 'right_home':
                    self.right_home_jp = eval(position)
                elif location == 'left_home':
                    self.left_home_jp = eval(position)
                elif location == 'think':
                    self.think_jp = eval(position)
                elif location == 'shake_head':
                    self.shake_head_jp = eval(position)
                elif location == 'shake_away':
                    self.shake_away_jp = eval(position)


    def _save_file(self, file):
        print "Saving your positions to file!\n\n"
        f = open(file, 'w')
        f.write('ribbon_location=' + str(self.ribbon_location) + '\n')
        f.write('cut_location=' + str(self.cut_location) + '\n')
        f.write('safe=' + str(self.safe_jp) + '\n')
        f.close()

    def get_locations(self):
        self._gripper.open()
        good_input = False
        while not good_input:
            filename2 = self._path + 'arm.config'
            self._read_file(filename2)
            self.read_file = raw_input("Would you like to use the previously "
                                       "found cut ribbon path locations (y/n)?")
            if self.read_file != 'y' and self.read_file != 'n':
                print "You must answer 'y' or 'n'"
            elif self.read_file == 'y':
                filename = self._path + self._side + '_poses.config'
                self._read_file(filename)
                good_input = True
            else:

                print ("Move %s arm into a safe location- press Circle button to confirm"% (self._side,))
                while(len(self.safe_jp) == 0 and not rospy.is_shutdown()):
                    rospy.sleep(0.1)
                print ("Cool - Got it!")

                print ("Move Gripper into location directly above ribbon - press Circle button to confirm")
                while(len(self.ribbon_location) == 0 and
                      not rospy.is_shutdown()):
                    rospy.sleep(0.1)
                print ("Cool - Got it!")

                print ("Move Gripper into cut position - press Circle button to confirm "
                       "button to record")
                while(len(self.cut_location) == 0 and not rospy.is_shutdown()):
                    rospy.sleep(0.1)
                print ("Cool - Got it!\n")
                filename = self._path + self._side + '_poses.config'
                self._save_file(filename)
                good_input = True

#----------------------------------------------------------------------------------
#----------------------------- Task Functions -------------------------------------
#----------------------------------------------------------------------------------
    def move_safe(self):
        self._limb.set_joint_position_speed(0.7)
        self._limb.move_to_joint_positions(self.safe_jp, threshold=0.1)

    def move_home(self):
        self._left_limb.set_joint_position_speed(0.4)
        self._limb.set_joint_position_speed(0.4)
        self._check_arm_state()
        suppress = deepcopy(self._arm_state['flipped'])
        actions = {'left': 'untuck', 'right': 'untuck'}
        self._move_to(actions, suppress)

    def send_image(self, image_name):
        img = cv2.imread(self._images + image_name)
        self.image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

    def cut(self):
        #Eyes closed
        self.send_image('/closed.png')
        rospy.sleep(0.3)

        #Eyes open
        self.send_image('/open.png')
        rospy.sleep(0.4)

        #Eyes look right

        self.send_image('/look_right.png')
        self._gripper.open()
        rospy.sleep(0.5)

        #Eyes open
        self.send_image('/open.png')
        rospy.sleep(0.4)

        self.focus_head(-0.4)
        rospy.sleep(0.2)
        self.move_safe()

        #Eyes look down
        self._limb.set_joint_position_speed(0.6)
        self._limb.move_to_joint_positions(self.ribbon_location,
                                           threshold=0.01745)
        #Move left arm to worried position
        self.send_image('/worried.png')
        self._left_limb.set_joint_position_speed(0.8)
        self._left_limb.move_to_joint_positions(self.think_jp,
                                           threshold=0.01745)
                                                  
        rospy.sleep(0.3)
        self._limb.set_joint_position_speed(0.6)
        self._limb.move_to_joint_positions(self.cut_location,
                                           threshold=0.01745)
        self._gripper.command_position(0.0)
        rospy.sleep(0.4)
        self.send_image('/suprised.png')
        self.head_neutral()
        rospy.sleep(0.4)
        self.send_image('/open.png')

        #Move left arm to whoop position one
        self._left_limb.set_joint_position_speed(0.8)
        self._left_limb.move_to_joint_positions(self.shake_head_jp,
                                           threshold=0.01745)
        #Eyes appear looking straight forward

        rospy.sleep(0.3)

        #Eyes wink
        self.send_image('/right_wink.png')
        rospy.sleep(0.3)

        #Eyes appear looking straight forward
        self.send_image('/open.png')
        rospy.sleep(0.4)
   
        #Eyes close
        self.send_image('/closed.png')
        rospy.sleep(0.2)

        self.nod()
        rospy.sleep(0.3)

        #Eyes appear looking straight forward
        self.send_image('/open.png')

        self.move_safe()
        #self.blink()

        self.move_home()
        self._gripper.command_position(100.0, block=True)
        self.send_image('/closed.png')
        rospy.sleep(1.5)

    def print_acr_image(self):
        self.send_image('/ACRV.png')

    def display_default_image(self):
        self.send_image('/default.png')

    def blink(self):
        self.send_image('/closed.png')
        rospy.sleep(0.2)
        self.send_image('/open.png')

    def head_neutral(self):
        head = baxter_interface.Head()        
        head.set_pan(0.0)

    def focus_head(self, angle):
        head = baxter_interface.Head()
        head.set_pan(angle, speed=10, timeout=500)
        head.pan()

    def nod(self):
        head = baxter_interface.Head()
        for _ in xrange(3):
            head.command_nod()

    def test_function(self):
        '''
        try:
            thread.start_new_thread(self.celebrate, ())
            thread.start_new_thread(self.boo, ())
        except Exception, e:
            print str(e)
        '''
        self.focus_head()
        self.set_head_neutral()

#----------------------------------------------------------------------------------
#----------------------------- Main Function --------------------------------------
#----------------------------------------------------------------------------------
def main():
    rospy.init_node("ribbon_cutting")

    rs = baxter_interface.RobotEnable()
    print("Enabling robot... ")
    rs.enable()

if __name__ == "__main__":
    main()
