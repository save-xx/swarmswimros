from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Range
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, TwistStamped, Vector3Stamped

import numpy as np
import asyncio

cv_bridge = CvBridge()

## Publisher packing for UE5 topics ----------

# image packer
def images_unpk(data, time):
    ''' Handle UE5 syntetic images from virtual cameras'''
    name = data['name']
    img  = data['data']
    image_message = cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")  # encode for ROS msg
    image_message.header.stamp =  time                             # self.get_clock().now().to_msg()
    image_message.header.frame_id = name                           # name of sim robot                         
    return name, image_message

# echo packer
def echo_unpk(data, time):
    ''' Handle UE5 vertical echosounder measuraments'''
    name = data['name']
    val  = data['data']
    echo_message = Range()
    echo_message.header.stamp =  time                             # self.get_clock().now().to_msg()
    echo_message.header.frame_id = name
    echo_message.range = float(val)
    return name, echo_message

def time_unpk(time):
    ''' Prepare the clock topic '''
    time_message = Clock()
    fract, integer  = np.modf(time)
    time_message.clock.sec = int(integer)
    time_message.clock.nanosec = int(fract * 1e9)
    return time_message

def update_cmds(sim,cmds):
    for cmd in cmds:
        name = cmd['agent']
        # extract associated agent
        agent = next((agent for agent in sim.agents if agent.name == name), None)
        if not agent: continue # skip commands refering to not-existing agents
        ## Depth command setting
        if "heave"==agent.depth_control: agent.cmd_heave = cmd['vertical']
        else: agent.cmd_depth = cmd['vertical']
        ## Heading command setting
        if "yawrate"==agent.heading_control: agent.cmd_yawrate = cmd['heading']
        else: agent.cmd_heading = cmd['heading']
        ## Planar command setting
        if "local_forces"==agent.planar_control: agent.cmd_forces = cmd['planar']
        elif "local_velocity"==agent.planar_control: agent.cmd_local_vel = cmd['planar']
        else: agent.cmd_planar = np.array(cmd['planar'])

def pack_NNDetections_into_rostopics(sim, list_of_updates):
    ''' returns a list of message objects to be published as sensors outputs'''
    list_of_msgs = []
    for name in list_of_updates:
        # extract associated agent
        agent = next((agent for agent in sim.agents if agent.name == name), None)
        detection_msg = Path()
        detection_msg.header.frame_id = name
        for key in agent.NNDetector:
            if key=='time_lapsed': continue
            # populate point as range, alpha, beta
            p = Point(x=float(agent.NNDetector[key][0]), 
                      y=float(agent.NNDetector[key][1]), 
                      z=float(agent.NNDetector[key][2]))
            single_detection = PoseStamped(pose=Pose(position=p))
            single_detection.header.frame_id = key # add name of detected agent
            detection_msg.poses.append(single_detection) # add to single agent detections
        list_of_msgs.append(detection_msg)
    return list_of_msgs

def gt_pack(sim):
    ''' pack groud truth topics '''
    list_of_msgs = []
    for agent in sim.agents:
        gt = TwistStamped()
        gt.header.frame_id = agent.name
        gt.twist.linear.x  = agent.pos[0]
        gt.twist.linear.y  = agent.pos[1]
        gt.twist.linear.z  = agent.pos[2]
        gt.twist.angular.z = agent.psi
        list_of_msgs.append(gt)
    return list_of_msgs

def heading_pack(sim):
    ''' pack measured headings topics '''
    list_of_msgs = []
    for agent in sim.agents:
        gt = Vector3Stamped()
        gt.header.frame_id = agent.name
        gt.vector.z  = agent.measured_heading
        list_of_msgs.append(gt)
    return list_of_msgs

def depth_pack(sim):
    ''' pack measured depths topics '''
    list_of_msgs = []
    for agent in sim.agents:
        gt = Vector3Stamped()
        gt.header.frame_id = agent.name
        gt.vector.z  = agent.measured_depth
        list_of_msgs.append(gt)
    return list_of_msgs

def pos_pack(sim):
    ''' pack measured positions topics '''
    list_of_msgs = []
    for agent in sim.agents:
        gt = Vector3Stamped()
        gt.header.frame_id = agent.name
        gt.vector.z  = agent.measured_pos
        list_of_msgs.append(gt)
    return list_of_msgs