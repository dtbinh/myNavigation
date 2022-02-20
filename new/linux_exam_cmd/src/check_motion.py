#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Float32

class CheckTakeoff(object):
    def __init__(self):
        self.check_takeoff = Bool()
        self.check_takeoff.data = False
        self.check_final_pose = Bool()
        self.check_final_pose.data = False
        self.check_land = Bool()
        self.check_land.data = False
        #self.get_init_position()

        self.check_takeoff_pub = rospy.Publisher('/check_takeoff', Bool, queue_size=1)
        self.check_land_pub = rospy.Publisher('/check_land', Bool, queue_size=1)
        self.check_final_pose_pub = rospy.Publisher('/check_final_pose', Bool, queue_size=1)
        rospy.Subscriber("/drone/gt_pose", Pose, self.pose_callback)
        self.drone_altitude = Float32()
        self.drone_pose_x = Float32()

        self.rate = rospy.Rate(1)

    def pose_callback(self,msg):
        self.drone_altitude = msg.position.z
        self.drone_pose_x = msg.position.x

    def check_if_takeoff(self):
        if self.drone_altitude > 1.0:
            self.check_takeoff.data = True
        else:
            self.check_takeoff.data = False

        self.publish_if_takeoff(self.check_takeoff)

    def check_if_land(self):
        if self.drone_altitude < 1.0:
            self.check_land.data = True
        else:
            self.check_land.data = False

        self.publish_if_land(self.check_land)

    def check_if_final_pose(self):
        if self.drone_pose_x > 4.0 and self.drone_pose_x < 6.0:
            self.check_final_pose.data = True
        else:
            self.check_final_pose.data = False

        self.publish_if_final_pose(self.check_final_pose)

    def publish_if_takeoff(self, pose):
        """
        Loops untils closed, publishing moved distance
        """
        while not rospy.is_shutdown():
            self.check_takeoff_pub.publish(pose)
            self.rate.sleep()

    def publish_if_land(self, pose):
        """
        Loops untils closed, publishing moved distance
        """
        while not rospy.is_shutdown():
            self.check_land_pub.publish(pose)
            self.rate.sleep()

    def publish_if_final_pose(self, pose):
        """
        Loops untils closed, publishing moved distance
        """
        while not rospy.is_shutdown():
            self.check_final_pose_pub.publish(pose)
            self.rate.sleep()

    def get_init_position(self):
        data_pose = None
        while data_pose is None:
            try:
                data_pose = rospy.wait_for_message("/drone/gt_pose", Pose, timeout=1)
            except:
                rospy.loginfo("Current bb8 odom not ready yet, retrying for setting up init pose")

        self._init_position = Pose()
        self._init_position.x = data_pose.position.x
        self._init_position.y = data_pose.position.y
        self._init_position.z = data_pose.position.z


    def get_current_positin(self, new_position):
        self._current_position = Pose()
        self._current_position.x = new_position.x
        self._current_position.y = new_position.y
        self._current_position.z = new_position.z

    def check_final_distance(self):
        final_pose = self.drone_pose_x
        if final_pose > 4 and final_pose < 6:
            pass

    def calculate_distance(self, new_position, old_position):
        x2 = new_position.x
        x1 = old_position.x
        #y2 = new_position.y
        #y1 = old_position.y
        #dist = math.hypot(x2 - x1, y2 - y1)
        dist = x2 - x1
        return dist

    def wait_until_bb8_moved_distance(self, distance):
        """
        It will write in a file the distance moved by BB8 from the moment of
        starting this class
        """
        rate = rospy.Rate(5)
        while self._bb8_mved_distance.data < distance:
            rate.sleep()
        rospy.loginfo("BB8 has moved disatance="+str(self._bb8_mved_distance.data))

        # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()

if __name__ == '__main__':
    rospy.init_node('bb8_movement_detector_node', anonymous=True)
    check_takeoff_obj = CheckTakeoff()
    #distance = 1.0
    time.sleep(2)
    try:
        check_takeoff_obj.check_if_final_pose()
    except rospy.ROSInterruptException:
        pass