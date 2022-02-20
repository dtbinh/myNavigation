#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import time

class MoveDrone():

    def __init__(self):
        self.drone_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('/drone/land', Empty, queue_size=1)

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        self.rate = rospy.Rate(10) # 10hz

    def publish_once_in_cmd_vel(self, cmd):
        """
        This is because publishing in topics sometimes fails teh first time you publish.
        In continuos publishing systems there is no big deal but in systems that publish only
        once it IS very important.
        """
        while not self.ctrl_c:
            connections = self.drone_vel_publisher.get_num_connections()
            if connections > 0:
                self.drone_vel_publisher.publish(cmd)
                rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def publish_once_in_takeoff(self, cmd):

        while not self.ctrl_c:
            connections = self.pub_takeoff.get_num_connections()
            if connections > 0:
                self.pub_takeoff.publish(cmd)
                rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def publish_once_in_land(self, cmd):

        while not self.ctrl_c:
            connections = self.pub_land.get_num_connections()
            if connections > 0:
                self.pub_land.publish(cmd)
                rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
            # works better than the rospy.is_shut_down()
            self.stop_drone()
            self.ctrl_c = True


    # function that makes the drone stop
    def stop_drone(self):
      rospy.loginfo("Stopping...")
      self._move_msg = Twist()
      self._move_msg.linear.x = 0.0
      self._move_msg.angular.z = 0.0
      self.publish_once_in_cmd_vel(self._move_msg)

    # function that makes the drone turn
    def turn_drone(self):
      rospy.loginfo("Turning...")
      self._move_msg = Twist()
      self._move_msg.linear.x = 0.0
      self._move_msg.angular.z = 1.0
      self.publish_once_in_cmd_vel(self._move_msg)

    # function that makes the drone move forward
    def move_forward_drone(self):
      rospy.loginfo("Moving forward...")
      self._move_msg = Twist()
      self._move_msg.linear.x = 1.0
      self._move_msg.angular.z = 0.0
      self.publish_once_in_cmd_vel(self._move_msg)

    def takeoff(self):
      rospy.loginfo("Taking Off...")
      self.takeoff_msg = Empty()
      self.publish_once_in_takeoff(self.takeoff_msg)

    def land(self):
      rospy.loginfo("Landing...")
      self.land_msg = Empty()
      self.publish_once_in_land(self.land_msg)


if __name__ == '__main__':
    rospy.init_node('move_drone', anonymous=True)
    movedrone_object = MoveDrone()

    try:
        movedrone_object.land()
    except rospy.ROSInterruptException:
        pass