#!/usr/bin/python

# Copyright (c) 2013, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import collections

from copy import deepcopy

import rospy
import sys
import tf
import cv2

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
    def __init__(self, limb):
        self._rp = rospkg.RosPack()
        self._side = limb
        self._limb = baxter_interface.Limb(limb)
        self._path = self._rp.get_path('ribbon_cut') + '/config/'
        self._images = (self._rp.get_path('ribbon_cut') +
                          '/share/images')
        dash_io = baxter_interface.DigitalIO(limb + '_upper_button')
        circle_io = baxter_interface.DigitalIO(limb + '_lower_button')

        self.ribbon_location = dict()
        self.cut_location = dict()
        self.home_jp = dict()
        self.safe_jp = dict()
        self.ribbon_approach = dict()
        self.cut_location = dict()
        self.cut_approach = dict()
        self.bridge = CvBridge()

        self._gripper = baxter_interface.Gripper(limb, baxter_interface.CHECK_VERSION)

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
        #self._gripper.on_type_changed.connect(self._check_calibration)

        #self._gripper.calibrate()
        #self._gripper.set_holding_force(100.0)

        ik_srv = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ik_srv, SolvePositionIK)
        self._ikreq = SolvePositionIKRequest()

        circle_io.state_changed.connect(self._path_positions)
        

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
            if len(self.home_jp) == 0:
                # Record home Location
                print 'Recording camera position'
                self.home_jp = self._limb.joint_angles()
            elif len(self.safe_jp) == 0:
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

                

    def _read_file(self, file):
        with open(file, 'r') as f:
            for line in f:
                split = line.split('=')
                location = split[0]
                prefix = location.split("_")[0]
                position = split[1]
                if location == 'home':
                    self.home_jp = eval(position)
                elif location == 'ribbon_location':
                    self.ribbon_location = eval(position)
                elif location == 'cut_location':
                    self.cut_location = eval(position)
                elif location == 'safe':
                    self.safe_jp = eval(position)

    def _save_file(self, file):
        print "Saving your positions to file!\n\n"
        f = open(file, 'w')
        f.write('home=' + str(self.home_jp) + '\n')
        f.write('ribbon_location=' + str(self.ribbon_location) + '\n')
        f.write('cut_location=' + str(self.cut_location) + '\n')
        f.write('safe=' + str(self.safe_jp) + '\n')
        f.close()

    def get_locations(self):
        self._gripper.open()
        good_input = False
        while not good_input:
            self.read_file = raw_input("Would you like to use the previously "
                                       "found cut ribbon path locations (y/n)?")
            if self.read_file != 'y' and self.read_file != 'n':
                print "You must answer 'y' or 'n'"
            elif self.read_file == 'y':
                filename = self._path + self._side + '_poses.config'
                self._read_file(filename)
                good_input = True
            else:
                print ("Move %s arm into home position - press Circle button to confirm"
                       % (self._side,))
                while(len(self.home_jp) == 0 and not rospy.is_shutdown()):
                    rospy.sleep(0.1)
                print ("Cool - Got it!")

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

    def move_safe(self):
        self._limb.set_joint_position_speed(1.0)
        self._limb.move_to_joint_positions(self.safe_jp, threshold=0.1)

    def move_home(self):
        self.move_safe()
        self._limb.set_joint_position_speed(1.0)
        self._limb.move_to_joint_positions(self.home_jp)


    def cut(self):
        #Eyes closed
        img = cv2.imread(self._images + '/closed.png')
        self.image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        rospy.sleep(0.2)
        #Eyes open
        img = cv2.imread(self._images + '/open.png')
        self.image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        rospy.sleep(0.15)

        #Eyes look right
        img = cv2.imread(self._images + '/look_right.png')
        self.image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        self._gripper.open()
        rospy.sleep(0.2)

        #Eyes open
        img = cv2.imread(self._images + '/open.png')
        self.image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

        self.move_safe()
        self._limb.set_joint_position_speed(1.0)
        self._limb.move_to_joint_positions(self.ribbon_location,
                                           threshold=0.01745)
        #Eyes look down
        img = cv2.imread(self._images + '/suprised.png')
        self.image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

        self._limb.set_joint_position_speed(1.0)
        self._limb.move_to_joint_positions(self.cut_location,
                                           threshold=0.01745)
        self._gripper.command_position(0.0)
        rospy.sleep(0.5)

        #Eyes appear looking straight forward
        img = cv2.imread(self._images + '/open.png')
        self.image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        rospy.sleep(0.25)

        #Eyes wink
        img = cv2.imread(self._images + '/right_wink.png')
        self.image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        rospy.sleep(0.2)

        #Eyes appear looking straight forward
        img = cv2.imread(self._images + '/open.png')
        self.image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        rospy.sleep(0.25)
   
        #Eyes close
        img = cv2.imread(self._images + '/closed.png')
        self.image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        rospy.sleep(0.1)

        self.celebrate()
        rospy.sleep(0.2)

        #Eyes appear looking straight forward
        img = cv2.imread(self._images + '/open.png')
        self.image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

        self.move_safe()
        self.move_home()
        rospy.sleep(0.2)

    def print_acr_image(self):
        img = cv2.imread(self._images + '/ACR.png')
        self.image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

    def display_default_image(self):
        img = cv2.imread(self._images + '/default.png')
        self.image_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))


    def celebrate(self):

        head = baxter_interface.Head()
        for _ in xrange(3):
            head.command_nod()
            self._gripper.command_position(100.0, block=True)
        #return


    def nod(self):
        head = baxter_interface.Head()
        for _ in xrange(2):
            head.command_nod()

	def _check_calibration(self, value):
		if self._gripper.calibrated():
		    return True
		elif value == 'electric':
		    rospy.loginfo("calibrating %s...",
		                  self._gripper.name.capitalize())
		    return (self._gripper.calibrate() == True)
		else:
		    return False

def main():
    rospy.init_node("ribbon_cutting")

    rs = baxter_interface.RobotEnable()
    print("Enabling robot... ")
    rs.enable()


if __name__ == "__main__":
    main()
