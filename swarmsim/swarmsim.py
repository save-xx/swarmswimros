import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.parameter import Parameter
from ament_index_python.packages import get_package_share_directory

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Range
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TwistStamped 

import yaml, os, asyncio
from SimAPI import UE5_API

YAML_PATH = get_package_share_directory('swarmsim')
cv_bridge = CvBridge()

class SwarmSim(Node):
    def __init__(self):
        '''
        Define the mode of operation (ROS2 param): 
          "ue5" : use UE5, forces framerate to 60fps, real-time 
          "rt"  : use real-time
          "step": step-and-wait configuration, wait for confirmation for each step
        '''
        super().__init__('swarmsim')
        self.declare_parameters(
            namespace='', 
            parameters=[('mode', 'ue5')]
        )
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL , True)])

        # variable initializations
        self.cmd_subs = {}
        self.cmd_pubs = {}
        self.load_yaml()
        
        # check mode
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        if not self.mode in {'ue5', 'rt', 'step'}: raise ValueError

        self.clock_publisher = self.create_publisher(Clock, '/clock', 10)  

        # API Related functions
        self.server = UE5_API()         # Initialize the API instance
        self.create_publishers()        # create a publisher for each agent/topic combination
        self.register_callbacks()       # Register callbacks

    def load_yaml(self):
        ''' Load ROS2 Specific parameters '''
        with open(os.path.join(YAML_PATH,'settings.yaml'),'r') as file:
            parameters = yaml.safe_load(file)
            self.GT_RATE = parameters['groud_truth_rate']
            self.H_RATE  = parameters['heading_rate']
            self.D_RATE  = parameters['depth_rate']

    # ---------------------------- CALLBACK DEFINITION --------------------------------

    def images_clbk(self,data):
        ''' Handle UE5 syntetic images from virtual cameras'''
        name = data['name']
        img  = data['data']
        image_message = cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")       # encode for ROS msg
        image_message.header.stamp = self.get_clock().now().to_msg()        # set timestamp
        image_message.header.frame_id = name                                # name of sim robot                         
        self.cmd_pubs['view'][name].publish(image_message) 

    def echo_clbk(self,data):
        ''' Handle UE5 vertical echosounder measuraments'''
        name = data['name']
        val  = data['data']
        echo_message = Range()
        echo_message.header.stamp = self.get_clock().now().to_msg()
        echo_message.header.frame_id = name
        echo_message.range = float(val)
        self.cmd_pubs['echo'][name].publish(echo_message)

    def time_clbk(self,data):
        ''' Get time '''
        time_message = Clock()
        time_message.clock.sec = int(data)
        time_message.clock.nsec = int( (data - int(data)) * 1e9)
        self.clock_publisher.publish(time_message)

    def register_callbacks(self):
        '''Register API callback for different types of sensors.'''
        self.server.register_callback("view", self.images_clbk)
        self.server.register_callback("echo",   self.echo_clbk)
        self.server.register_callback("time",   self.time_clbk)

    # ----------------------------- PUBLISHER DEFINITION --------------------------------

    def create_pubs(self, topic: str, type):
        ''' create a topic sensor named topic for each agent'''
        self.cmd_pubs[topic] = {}
        for agent in self.server.sim.agents:
           self.cmd_pubs[topic][agent.name] = self.create_publisher(type, f'{agent.name}/{topic}', 10)

    def create_publishers(self):
        ''' Register publishers to be defined. '''
        self.create_pubs('echo', Range)
        self.create_pubs('view', Image)

    # ------------------------------ SUBSCRIBER DEFINITION ------------------------------

    def cmd_clbk(self, msg, topic):
        ''' Send cmd via API to simulated agents'''
        parent = topic.split('/')[0]
        data = {
            'agent': parent,
            'frame': msg.header.frame_id,
            'planar': [msg.twist.linear.x, msg.twist.linear.y],
            'vertical': msg.twist.linear.z,
            'heading': msg.twist.angular.z
        }
        # TODO: submit to simulator (when hardware-in-the-loop)

    def create_subscribers(self):
         ''' create a command suscriber for each agent'''
         for agent in self.server.sim.agents:
            topic_name = agent.name+'/cmd'
            self.cmd_subs[agent.name] = self.create_subscription(
                TwistStamped, topic_name,
                lambda msg, topic=topic_name: self.cmd_clbk(msg, topic), 1)
            
    # ---------------------------------------------------------------------------------
    async def ros_spin(self):
        '''Async wrapper for spinning the ROS node.'''
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            await asyncio.sleep(0)  # Yield control to the event loop
        rclpy.shutdown()
            
    async def main(self):
        ''' Async Wrapper of all processes. '''
        await asyncio.gather(
            self.server.call(),
            self.ros_spin()
        )

def main(args=None):
    
    rclpy.init(args=args)
    swarm_sim = SwarmSim()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    try:
        loop.run_until_complete(swarm_sim.main())
    except KeyboardInterrupt:
        swarm_sim.get_logger().warning("UserInterrupt")
    finally:
        swarm_sim.destroy_node()
        loop.close()

if __name__ == '__main__':
    main()