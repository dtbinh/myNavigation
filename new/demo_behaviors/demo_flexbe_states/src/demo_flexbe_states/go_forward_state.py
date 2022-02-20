#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class DemoFState(EventState):
    '''
	Driving state for a ground robot. This state allows the robot to drive forward a certain distance
    at a specified velocity/ speed.

	-- speed 	float 	Speed at which to drive the robot
    -- travel_dist float   How far to drive the robot before leaving this state
    -- obstacle_dist float Distance at which to determine blockage of robot path

	<= failed 			    If behavior is unable to ready on time
	<= done 				Example for a failure outcome.

	'''
    def __init__(self, speed, travel_dist, obstacle_dist):
        super(DemoFState, self).__init__(outcomes=['failed', 'done'])
        self._start_time = None
        self.data = None
        self._speed = speed
        self._travel_dist = travel_dist
        self._obstacle_dist = obstacle_dist

        self.vel_topic = '/cmd_vel'
        self.scan_topic = '/kobuki/laser/scan'

        #create publisher passing it the vel_topic_name and msg_type
        self.pub = ProxyPublisher({self.vel_topic: Twist})
        #create subscriber
        self.scan_sub = ProxySubscriberCached({self.scan_topic: LaserScan})
        self.scan_sub.set_callback(self.scan_topic, self.scan_callback)
    
    def execute(self, userdata):
        if not self.cmd_pub:
            return 'failed'
        #run obstacle checks [index 0: left, 360: middle, 719: right]
        if self.data is not None:
            #self.data = self.scan_sub.get_last_msg(self.scan_topic)
            #self.scan_sub.remove_last_msg(self.scan_topic)
            Logger.loginfo('FWD obstacle distance is: %s' % self.data.ranges[360])
            if self.data.ranges[360] <= self._obstacle_dist:
                self.scan_sub.remove_last_msg(self.scan_topic, True)
                self.data = None
                return 'failed'
    
            #measure distance travelled
            elapsed_time = (rospy.Time.now() - self._start_time).to_sec()
            distance_travelled = (elapsed_time/ 60.0) * self._speed
    
            if distance_travelled >= self._travel_dist:
                return 'done'
        
        #drive
        self.pub.publish(self.vel_topic, self.cmd_pub)

    def on_enter(self, userdata):
        rospy.sleep(0.2)
        Logger.loginfo("Drive FWD STARTED!")
        #set robot speed here
        self.cmd_pub = Twist()
        self.cmd_pub.linear.x = self._speed
        self.cmd_pub.angular.z = 0.0
        
    def on_exit(self, userdata):
        self.cmd_pub.linear.x = 0.0
        self.pub.publish(self.vel_topic, self.cmd_pub)
        self.scan_sub.unsubscribe_topic(self.scan_topic)
        Logger.loginfo("Drive FWD ENDED!")
        
    def on_start(self):
        Logger.loginfo("Drive FWD READY!")
        self._start_time = rospy.Time.now() #bug detected! (move to on_enter)
    
        
    def on_stop(self):
		Logger.loginfo("Drive FWD STOPPED!")
		self.cmd_pub.linear.x = 0.0
		self.pub.publish(self.vel_topic, self.cmd_pub)

    def scan_callback(self, data):
        self.data = data