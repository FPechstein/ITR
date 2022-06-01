import rclpy
import numpy as np
from rclpy.node import Node

from driving_swarm_messages.msg import Range
from geometry_msgs.msg import PointStamped


class LocatorNode(Node):

    def __init__(self):
        super().__init__('locator_node')
        self.anchor_ranges = []
        self.create_subscription(Range, 'range', self.range_cb, 10)
        #self.create_subscription(Range, 'anchor', self.range_cb, 10)
        self.position_pub = self.create_publisher(PointStamped, 'position', 10)
        self.initialized = False
        self.create_timer(1.0, self.timer_cb)
        self.get_logger().info('locator node started')
        self.d=0
        self.a=0
        self.x2=0
        self.x3=0
        self.y2=0
        self.y3=0
        self.x4=0
        self.y4=0
        self.schnittpunktey=[]
        self.schnittpunktex=[]
        self.x=0.0
        self.y=0.0
        self.x1=0.0
        self.y1=0.0
	
	
    def range_cb(self, msg):
        self.anchor_ranges.append(msg)
        self.anchor_ranges = self.anchor_ranges[-10:]
        #for r in self.anchor_ranges:
        #	self.get_logger().info(f'{r.anchor.x}')
        if not self.initialized:
            self.initialized = True
            self.get_logger().info('first range received')
            

    def timer_cb(self):
        if not self.initialized:
            return
        msg = PointStamped()
        msg.point.x, msg.point.y, msg.point.z = self.calculate_position()
        msg.header.frame_id = 'world'
        self.position_pub.publish(msg)
    
    def calculate_position(self):
        if len(self.anchor_ranges)<5:
            return 0.0, 0.0, 0.0
        
        # YOUR CODE GOES HERE:
         
        self.d=np.sqrt((self.anchor_ranges[2].anchor.x-self.anchor_ranges[1].anchor.x)**2+(self.anchor_ranges[2].anchor.y-self.anchor_ranges[1].anchor.y)**2)
        self.a=(self.anchor_ranges[1].range**2-self.anchor_ranges[2].range**2+self.d**2)/(2*self.d)
        self.h=np.sqrt(self.anchor_ranges[1].range**2-self.a**2)
        self.x2=self.anchor_ranges[1].anchor.x+self.a*(self.anchor_ranges[2].anchor.x-self.anchor_ranges[1].anchor.x)/self.d
        self.y2=self.anchor_ranges[1].anchor.y+self.a*(self.anchor_ranges[2].anchor.y-self.anchor_ranges[1].anchor.y)/self.d
        self.x3=self.x2+self.h*(self.anchor_ranges[2].anchor.y-self.anchor_ranges[1].anchor.y)/self.d
        self.y3=self.y2-self.h*(self.anchor_ranges[2].anchor.x-self.anchor_ranges[1].anchor.x)/self.d
        self.x4=self.x2-self.h*(self.anchor_ranges[2].anchor.y-self.anchor_ranges[1].anchor.y)/self.d
        self.y4=self.y2+self.h*(self.anchor_ranges[2].anchor.x-self.anchor_ranges[1].anchor.x)/self.d
        
        self.schnittpunktex.append(self.x3)
        self.schnittpunktex.append(self.x4)
        self.schnittpunktey.append(self.y3)
        self.schnittpunktey.append(self.y4)
        
        self.d=np.sqrt((self.anchor_ranges[4].anchor.x-self.anchor_ranges[3].anchor.x)**2+(self.anchor_ranges[4].anchor.y-self.anchor_ranges[3].anchor.y)**2)
        self.a=(self.anchor_ranges[4].range**2-self.anchor_ranges[3].range**2+self.d**2)/(2*self.d)
        self.h=np.sqrt(self.anchor_ranges[3].range**2-self.a**2)
        self.x2=self.anchor_ranges[3].anchor.x+self.a*(self.anchor_ranges[4].anchor.x-self.anchor_ranges[3].anchor.x)/self.d
        self.y2=self.anchor_ranges[3].anchor.y+self.a*(self.anchor_ranges[4].anchor.y-self.anchor_ranges[3].anchor.y)/self.d
        self.x3=self.x2+self.h*(self.anchor_ranges[4].anchor.y-self.anchor_ranges[3].anchor.y)/self.d
        self.y3=self.y2-self.h*(self.anchor_ranges[4].anchor.x-self.anchor_ranges[3].anchor.x)/self.d
        self.x4=self.x2-self.h*(self.anchor_ranges[4].anchor.y-self.anchor_ranges[3].anchor.y)/self.d
        self.y4=self.y2+self.h*(self.anchor_ranges[4].anchor.x-self.anchor_ranges[3].anchor.x)/self.d
        
        self.schnittpunktex.append(self.x3)
        self.schnittpunktex.append(self.x4)
        self.schnittpunktey.append(self.y3)
        self.schnittpunktey.append(self.y4)
        
        
        if self.schnittpunktex[0]==self.schnittpunktex[1] or self.schnittpunktex[0]==self.schnittpunktex[2] or self.schnittpunktex[0]==self.schnittpunktex[3]:
        	self.x=self.schnittpunktex[0]
        elif self.schnittpunktex[2]==self.schnittpunktex[1] or self.schnittpunktex[1]==self.schnittpunktex[3]:
        	self.x=self.schnittpunktex[1]
        elif self.schnittpunktex[2]==self.schnittpunktex[3]:
        	self.x=self.schnittpunktex[2]
        else :
        	self.x=self.x1
        
        if self.schnittpunktey[0]==self.schnittpunktey[1] or self.schnittpunktey[0]==self.schnittpunktey[2] or self.schnittpunktey[0]==self.schnittpunktey[3]:
        	self.y=self.schnittpunktey[0]
        elif self.schnittpunktey[2]==self.schnittpunktey[1] or self.schnittpunktey[1]==self.schnittpunktey[3]:
        	self.y=self.schnittpunktey[1]
        elif self.schnittpunktey[2]==self.schnittpunktey[3]:
        	self.y=self.schnittpunktey[2]
        else :
        	self.y=self.y1
        
       
        
        self.get_logger().info(f'{self.schnittpunktex[0]}')
        self.get_logger().info(f'{self.schnittpunktex[1]}')
        self.get_logger().info(f'{self.schnittpunktex[2]}')
        self.get_logger().info(f'{self.schnittpunktex[3]}')
        #self.get_logger().info(f'{x}')
        self.schnittpunktex.clear()
        self.schnittpunktey.clear()

        #x = np.mean([r.range for r in self.anchor_ranges]) - 0.5
        
     
        self.x1=self.x
        self.y1=self.y
        #self.get_logger().info(f'{x}')
        return self.x, self.y, 0.5


def main(args=None):
    rclpy.init(args=args)

    node = LocatorNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
