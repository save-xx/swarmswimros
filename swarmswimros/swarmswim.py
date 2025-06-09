import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.parameter import Parameter
from ament_index_python.packages import get_package_share_directory

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Range, CompressedImage, JointState
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from example_interfaces.msg import Bool
from nav_msgs.msg import Path

import yaml, os, sys, asyncio, time
import importlib.resources

# Dynamically add the SwarmSwIM submodule to the Python path
swarm_swim_path = os.path.join(os.path.dirname(__file__), 'SwarmSwIM')
if swarm_swim_path not in sys.path:
    sys.path.insert(0, swarm_swim_path)

from swarmswimros.swarmswim_functions import *
from SwarmSwIM import UE5_API
from SwarmSwIM import Simulator, CNNDetection, AcousticChannel

CONFIG_PATH = get_package_share_directory('swarmswimros')
cv_bridge = CvBridge()

class SwarmSwim(Node):
    def __init__(self):
        """
        Define the mode of operation (ROS2 param).

          "ue5" : use UE5, forces framerate to 60fps, real-time 
          "rt"  : use real-time
          "step": step-and-wait configuration, wait for confirmation for each step
        """
        super().__init__('swarmswim')
        self.declare_parameters(
            namespace='', 
            parameters=[('mode', 'step')]
        )
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL , True)])
        self.lock = asyncio.Lock()

        # check mode
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        if not self.mode in {'ue5', 'rt', 'step'}: raise ValueError

        # variable initializations
        self.cmd_subs = {}
        self.cmd_pubs = {}
        self.load_yaml()
        self.cmds = []
        if 'ue5'== self.mode: self.Dt=1/60 # UE5 sim overwrite default

        # create a clock topic for the sim_time
        self.clock_publisher = self.create_publisher(Clock, '/clock', 10)  

        # create Simulator Instance
        if self.sim_filename:           # If filename was specified search the full path
            sim_path = os.path.join(CONFIG_PATH, self.sim_filename + '.xml')
        else: 
            sim_path = os.path.join(CONFIG_PATH, 'simulation.xml')        
        self.sim = Simulator(self.Dt, sim_xml=sim_path)
        self.get_logger().info(f'Initiated Simulator with file {sim_path}')

        # API Related functions
        if 'ue5'==self.mode:  self.server = UE5_API()         # Initialize the API instance
        self.create_publishers()        # create a publisher for each agent/topic combination
        self.register_callbacks()       # Register callbacks
        self.create_subscribers()       # add cmd Subscribers

        # Plugin functions
        if "CNNDetector" in self.plugins: self.Detection = CNNDetection()
        if "Acoustic" in self.plugins: self.Acoustic = AcousticChannel()

        # Subscriber to send acoustics
        if "Acoustic" in self.plugins:
            # Type used to store the byte array of the payload
            # the frame_id contains the sender name
            self.acoustic_subscriber = self.create_subscription(
                CompressedImage, 
                'acoustic_send', 
                self.send_acoustic_clbk, 1)


        # Sensor timers
        self.gt_timer = self.create_timer(self.GT_RATE, self.gt_callback)
        self.gt_timer = self.create_timer(self.GT_RATE, self.heading_callback)
        self.gt_timer = self.create_timer(self.GT_RATE, self.depth_callback)

    async def cycle(self):
        '''The function that ticks the simulation.'''
        async with self.lock:           # Ensure thread-safe access to the simulation
            update_cmds(self.sim,self.cmds) # Update commands to agents
            self.sim.tick()             # Dynamic clock advanment
            # Handle NN Detector
            if "CNNDetector" in self.plugins: 
                list_of_detections = self.Detection(self.sim)
                if list_of_detections:
                    detection_msgs = pack_NNDetections_into_rostopics(self.sim,list_of_detections)
                    self.NN_callback(detection_msgs)
            # Handle Acoustics
            if "Acoustic" in self.plugins:
                delivered = self.Acoustic(self.sim)
                if delivered:
                    comm_msg, ac_range_msg = pack_Acoustic(self.sim,delivered)
                    self.acoustic_callback(comm_msg, ac_range_msg)
                    pass ##TODO

            self.clock_publisher.publish(time_unpk(self.sim.time)) # Publish clock at each tick
        await asyncio.sleep(0)          # Allows other tasks to run

    def load_yaml(self):
        ''' Load ROS2 Specific parameters '''
        with open(os.path.join(CONFIG_PATH, 'settings.yaml'),'r') as file:
            parameters = yaml.safe_load(file)
            self.sim_filename = parameters['sim_filename']
            self.Dt = parameters['Dt']

            self.GT_RATE = parameters['groud_truth_rate']
            self.H_RATE  = parameters['heading_rate']
            self.D_RATE  = parameters['depth_rate']

            self.plugins = parameters['plugins']

    # ---------------------------- CALLBACK DEFINITION --------------------------------

    def step_callback(self, data):
        ''' Special callback to allow a step based simulation progression '''
        # self.get_logger().info('Step received, running one cycle.')
        if data.data: asyncio.create_task(self.cycle())  # Run cycle asynchronously

    def images_clbk(self,data):
        name, image_message = images_unpk(data, self.get_clock().now().to_msg())                         
        self.cmd_pubs['view'][name].publish(image_message) 

    def echo_clbk(self,data):
        name, echo_message = echo_unpk(data, self.get_clock().now().to_msg()) 
        self.cmd_pubs['echo'][name].publish(echo_message)

    def NN_callback(self,datalist):
        for data in datalist:
            name = data.header.frame_id
            self.cmd_pubs['NNDetection'][name].publish(data)

    def gt_callback(self):
        list_of_gt_msg = gt_pack(self.sim)
        for msg in list_of_gt_msg:
            name = msg.header.frame_id
            self.cmd_pubs['gt'][name].publish(msg)
            
    def heading_callback(self):
        list_of_heding_msg = heading_pack(self.sim)
        for msg in list_of_heding_msg:
            name = msg.header.frame_id
            self.cmd_pubs['heading'][name].publish(msg)

    def depth_callback(self):
        list_of_dpt_msg = depth_pack(self.sim)
        for msg in list_of_dpt_msg:
            name = msg.header.frame_id
            self.cmd_pubs['depth'][name].publish(msg)

    def acoustic_callback(self, comm_msg, ac_range_msg):
        for msg, rng in zip(comm_msg,ac_range_msg):
            name = msg.header.frame_id
            self.cmd_pubs['acoustic'][name].publish(msg)
            if rng: self.cmd_pubs['ac_ranges'][name].publish(rng)
            print(rng)

    def register_callbacks(self):
        '''Register API callback for different types of sensors.'''
        if 'ue5'==self.mode: 
            self.server.register_callback("view", self.images_clbk)
            self.server.register_callback("echo",   self.echo_clbk)
        if 'step'==self.mode:
            self.create_subscription(Bool, 'sim_step', self.step_callback, 1)
        


    # ----------------------------- ITERATIVE PUBLISHER DEFINITION --------------------------------

    def create_pubs(self, topic: str, type):
        ''' create a topic sensor named topic for each agent'''
        self.cmd_pubs[topic] = {}
        for agent in self.sim.agents:
           self.cmd_pubs[topic][agent.name] = self.create_publisher(type, f'{agent.name}/{topic}', 10)

    def create_publishers(self):
        ''' Register publishers to be defined. '''
        self.create_pubs('gt', TwistStamped)
        self.create_pubs('depth', Vector3Stamped)
        self.create_pubs('heading', Vector3Stamped)
        if 'ue5'==self.mode: 
            self.create_pubs('echo', Range)
            self.create_pubs('view', Image)
        if "CNNDetector" in self.plugins: 
            self.create_pubs('NNDetection', Path)
        if "Acoustic" in self.plugins:
            self.create_pubs('acoustic', CompressedImage)
            self.create_pubs('ac_ranges', JointState)

    # ------------------------------ ITERATIVE SUBSCRIBER DEFINITION ------------------------------

    def create_subscribers(self):
         ''' create a command suscriber for each agent'''
         for agent in self.sim.agents:
            topic_name = agent.name+'/cmd'
            self.cmd_subs[agent.name] = self.create_subscription(
                TwistStamped, topic_name,
                lambda msg, topic=topic_name: self.cmd_clbk(msg, topic), 1)

    def cmd_clbk(self, msg, topic):
        ''' Send cmd via API to simulated agents'''
        parent = topic.split('/')[0]
        # Schedule the modification of self.cmds to run in the asyncio event loop
        asyncio.run_coroutine_threadsafe(self._handle_cmd(parent, msg), asyncio.get_event_loop())
        

    async def _handle_cmd(self, parent, msg):
        ''' self.cmd update protected from race conditions '''
        async with self.lock: 
            self.cmds.append( {'agent': parent,  'frame': msg.header.frame_id,
                'planar': [msg.twist.linear.x, msg.twist.linear.y],
                'vertical': msg.twist.linear.z,  'heading': msg.twist.angular.z}
            )

    def send_acoustic_clbk(self, msg):
        ''' Send communication to acoustic channel '''
        name = msg.header.frame_id
        agent = next((agent for agent in self.sim.agents if agent.name == name), None)
        payload  = msg.data
        if agent: asyncio.run_coroutine_threadsafe(self._handle_acoustic(agent, payload), asyncio.get_event_loop())
    
    async def _handle_acoustic(self, agent, payload):
        ''' Load communication in the acoustic enviroment, protected by race conditions '''
        async with self.lock: 
            out = self.Acoustic.send(agent, self.sim , 1.0,  payload)

    ## Handler related to API and ROS2 anysincronous operations       
    # ---------------------------------------------------------------------------------
    async def ros_spin(self):
        '''Async wrapper for spinning the ROS node.'''
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            await asyncio.sleep(0)  # Yield control to the event loop
        rclpy.shutdown()
            
    async def realtime_tick(self):
        '''Async wrapper spinning the clock and Simulation tick in Real-Time "rt" settings '''
        while rclpy.ok():
            start_time = time.monotonic()  # Record the start time
            await self.cycle()  # Execute the simulation tick
            elapsed_time = time.monotonic() - start_time  # Measure elapsed time
            # Sleep for the remaining time to maintain the period
            sleep_time = max(0, self.Dt - elapsed_time)
            await asyncio.sleep(sleep_time)

    async def main(self):
        ''' Async Wrapper of all processes. '''
        tasks = [self.ros_spin()]
        # conditionally add the API task if UE5 is active
        if 'ue5'==self.mode: tasks.append(self.server.call())
        if 'rt'==self.mode:  tasks.append(self.realtime_tick())
        await asyncio.gather(*tasks)

def main(args=None):
    
    rclpy.init(args=args)
    swarm_swim = SwarmSwim()
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    try:
        loop.run_until_complete(swarm_swim.main())
    except KeyboardInterrupt:
        swarm_swim.get_logger().warning("UserInterrupt")
    finally:
        swarm_swim.destroy_node()
        loop.close()

if __name__ == '__main__':
    main()
