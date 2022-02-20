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
        #self.get_init_position()

        self.check_takeoff_pub = rospy.Publisher('/check_takeoff', Bool, queue_size=1)
        rospy.Subscriber("/drone/gt_pose", Pose, self.pose_callback)
        self.drone_altitude = Float32()

        self.rate = rospy.Rate(1)

    def pose_callback(self, msg):
        self.drone_altitude = msg.position.z

    def check_if_takeoff(self):

        if self.drone_altitude > 1.0:
            self.check_takeoff.data = True

        else:
            self.check_takeoff.data = False


        self.publish_if_takeoff(self.check_takeoff)

    def publish_if_takeoff(self, pose):
        """
        Loops untils closed, publishing moved distance
        """
        while not rospy.is_shutdown():

            self.check_takeoff_pub.publish(pose)
            self.rate.sleep()

    def get_init_position(self):
        data_odom = None
        while data_odom is None:
            try:
                data_odom = rospy.wait_for_message("/odom", Odometry, timeout=1)
            except:
                rospy.loginfo("Current bb8 odom not ready yet, retrying for setting up init pose")

        self._current_position = Point()
        self._current_position.x = data_odom.pose.pose.position.x
        self._current_position.y = data_odom.pose.pose.position.y
        self._current_position.z = data_odom.pose.pose.position.z


    def updatecurrent_positin(self, new_position):
        self._current_position.x = new_position.x
        self._current_position.y = new_position.y
        self._current_position.z = new_position.z

    def calculate_distance(self, new_position, old_position):
        x2 = new_position.x
        x1 = old_position.x
        y2 = new_position.y
        y1 = old_position.y
        dist = math.hypot(x2 - x1, y2 - y1)
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
    rospy.init_node('check_takeoff', anonymous=True)
    check_takeoff_obj = CheckTakeoff()
    #distance = 1.0
    time.sleep(2)
    #check_takeoff_obj.check_if_takeoff()
    try:
        check_takeoff_obj.check_if_takeoff()
    except rospy.ROSInterruptException:
        pass