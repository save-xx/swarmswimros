import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String, Header
from geometry_msgs.msg import TwistStamped 
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
import requests, yaml, os

YAML_PATH = get_package_share_directory('swarmsim')
DEFAULT_API="http://localhost:5555/"

class SwarmSim(Node):
    def __init__(self):
        super().__init__('swarmsim')
        ## TODO extract actual list from simulator(API)
        self.agent_names = ['A01','A02']
        self.cmd_subs = {}
        self.cmd_pubs = {}
        ###
        self.create_pubs('echosounder', Header)
        self.create_pubs('img', Image)


        ## LOAD SETTINGS
        self.load_yaml()
        self.declare_parameter('use_sim_time', True)    # Enable use of Simulated Time
        self.clock_publisher = \
            self.create_publisher(Clock, '/clock', 10)  # Publisher for the /clock topic
        self.create_subscribers()                       # create a cmd subscriber for each agent

        # set timer callbacks
        self.GROUND_THRUTH_RATE = 0.1
        self.DEPTH_RATE = 0.2
        self.HEADING_RATE = 0.2
        
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def load_yaml(self):
        with open(os.path.join(YAML_PATH,'settings.yaml'),'r') as file:
            parameters = yaml.safe_load(file)
            self.GT_RATE = parameters('groud_truth_rate')
            self.H_RATE = parameters('heading_rate')
            self.D_RATE = parameters('depth_rate')

    def create_subscribers(self):
         ''' create a command suscriber for each agent'''
         for name in self.agent_names:
            topic_name = name+'/cmd'
            self.cmd_subs[name] = self.create_subscription(
                TwistStamped,topic_name,
                lambda msg, topic=topic_name: self.cmd_clbk(msg, topic), 1)

    def cmd_clbk(self, msg, topic):
        ''' Send cmd  via API to simulated agents'''
        parent = topic.split('/')[0]
        data = {
            'agent': parent,
            'frame': msg.header.frame_id,
            'planar': [msg.twist.linear.x, msg.twist.linear.y],
            'vertical': msg.twist.linear.z,
            'heading': msg.twist.angular.z
        }

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def get_item(self, addr: str):
        ''' Get item from a specific address'''
        url = DEFAULT_API + addr
        response = requests.get(url)
        # Handle incorrect requests
        if not 200 == response.status_code: 
            self.get_logger().warning("Incorrect API response from Simulator",throttle_duration_sec=1) 
            return None
        items = response.json()
        return items

    def post_item(self, addr: str, data):
        ''' send information to the API server'''
        response = requests.post(DEFAULT_API + addr, json=data)
        # Warning if command was not sent succesfully
        if not 200 == response.status_code:
            self.get_logger().warning(f"Send request failed with status code: {response.status_code}")

    def create_pubs(self, topic: str, type):
        self.cmd_pubs[topic] = {}
        for agent in self.agent_names:
           self.cmd_pubs[topic][agent].append(self.create_publisher(type, f'{agent}/{topic}', 10))

def main(args=None):
    rclpy.init(args=args)
    swarm_sim = SwarmSim()
    rclpy.spin(swarm_sim)
    swarm_sim.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()