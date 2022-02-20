import rospy, math
from geometry_msgs.msg import Twist

class VelocityController():
    def __init__(self, topic):
        self.cmd_vel = rospy.Publisher(topic, Twist, queue_size=10)
        rospy.sleep(0.1)

    def move(self, linear_velocity=0.0, angular_velocity=0.0):
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        self.cmd_vel.publish(msg)
        
def init():
    global vel
    rospy.init_node('demo')
    rospy.sleep(1)
    vel = VelocityController('/cmd_vel')
    
def forward():
    vel.move(0.5)
    
def backward():
    vel.move(-0.5)
    
def stop():
    vel.move(0,0)
    
def turn_left():
    vel.move(0,1)
    
def turn_right():
    vel.move(0,-1)
    
def square():
    for _ in range(4):
        forward()
        rospy.sleep(2)
        turn_left()
        rospy.sleep(math.pi/2)
    stop()
    
def circle(w=1):
    vel.move(0.5,w)
    rospy.sleep(2*math.pi)
    stop()
    
def eight():
    circle()
    circle(-1)
    