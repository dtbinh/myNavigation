import rospy, math, cv2, numpy as np

from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState, SetModelState

def pause_physics():
    try:
        resetSimSrv = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        resetSimSrv()
    except rospy.ServiceException, e:
        print "/gazebo/pause_physics call failed: %s"%e

def unpause_physics():
    try:
        resetSimSrv = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        resetSimSrv()
    except rospy.ServiceException, e:
        print "/gazebo/unpause_physics call failed: %s"%e

def reset_simulation():
    try:
        resetSimSrv = rospy.ServiceProxy("/gazebo/reset_simulation", Empty)
        resetSimSrv()
    except rospy.ServiceException, e:
        print "/gazebo/reset_simulation call failed: %s"%e

def reset_world():
    try:
        resetWorldSrv = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        resetWorldSrv()
    except rospy.ServiceException, e:
        print "/gazebo/reset_world call failed: %s"%e

def spawn_coke_can(name, pose):
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    #with open("/usr/share/gazebo/models/coke_can/model.sdf", "r") as f:
    with open("/home/user/.gazebo/models/coke_can/model.sdf", "r") as f:
        model_xml = f.read()
    spawn_model(name, model_xml, "", pose, "world")

def spawn_table(name, pose):
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    #with open("/usr/share/gazebo/models/table/model.sdf", "r") as f:
    with open("/home/user/.gazebo/models/table/model.sdf", "r") as f:
        model_xml = f.read()
    spawn_model(name, model_xml, "", pose, "world")

def set_model_state(name, pose):
    set_model_state_service = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    new_model_state = ModelState()
    new_model_state.model_name = name
    new_model_state.pose = pose
    set_model_state_service(new_model_state)
    
def get_model_state(name):
    get_model_state_service = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    return get_model_state_service(name, 'world')

def set_position(x=0.0, y=0.0, angle=0.0):
    """Set the position and orientation of the robot in the simulator.
    Keyword arguments:
    x -- the X-coordinate in meters (default 0.0)
    y -- the Y-coordinate in meters (default 0.0)
    angle -- the angle in degrees (default 0.0)
    """
    rad_angle = angle * math.pi / 180
    state_msg = ModelState()
    state_msg.model_name = 'mobile_base'
    state_msg.pose.position.x = x
    state_msg.pose.position.y = y
    state_msg.pose.position.z = 0
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = math.sin(rad_angle/2)
    state_msg.pose.orientation.w = math.cos(rad_angle/2)
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )
    except rospy.ServiceException, e:
        print "/gazebo/set_model_state call failed: %s" % e
    rospy.sleep(1.0)
    
def path2waypoints(path):
    x = 0
    y = 0
    theta = 0
    waypoints = [(x,y)]
    for (R,s) in path:
        if R==float('inf'):
            x += s * math.cos(theta)
            y += s * math.sin(theta)
        else:
            angle = s / R
            if angle>=0:
                dx = R * math.sin(angle)
                dy = R * (1 - math.cos(angle))
            else:
                dx = -R * math.sin(-angle)
                dy = R * (1 - math.cos(angle))
            x += dx * math.cos(theta) - dy * math.sin(theta)
            y += dx * math.sin(theta) + dy * math.cos(theta)
            theta += angle
        waypoints.append((x,y))
    return waypoints

def cleaned_area(trajectory):
    x, y = zip(*trajectory)
    r = 0.354/2
    n = len(x)
    height = 1500
    width = 1000
    image = np.zeros((height,width), np.uint8)
    for i in range(0,n):
        cv2.circle(image, (int(x[i]*100+500), int(y[i]*100+1300)), 
                   int(r*100), 255, -1)
    return cv2.countNonZero(image)/100.0**2
