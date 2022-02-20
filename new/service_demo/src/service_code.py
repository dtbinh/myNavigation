#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse # you import the service message python classes generated from Empty.srv.
from geometry_msgs.msg import Twist


def my_callback(request):

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(1)
    move = Twist()
    move.linear.x = 0.5
    move.angular.z = 1
    for i in range(4):
        pub.publish(move)
        rate.sleep()

    move.linear.x = 0
    move.angular.z = 0
    pub.publish(move)

    return EmptyResponse() # the service Response class, in this case EmptyResponse
    #return MyServiceResponse(len(request.words.split()))

rospy.init_node('service_test_node')
my_service = rospy.Service('/service_demo', Empty , my_callback) # create the Service called my_service with the defined callback
rospy.spin() # mantain the service open.