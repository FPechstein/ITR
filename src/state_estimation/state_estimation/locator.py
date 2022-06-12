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
        self.position_pub = self.create_publisher(PointStamped, 'position', 10)
        self.initialized = False
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('locator node started')
        self.rangesToGuess=np.array([])
        self.residium=np.array([])
        self.gradient=np.array([],dtype='float64')
        self.x=np.array([[1.0],[1.5],[0.0]])
        self.counter=0
     
        
	
	
    def range_cb(self, msg):
    	if msg.range<=0 or msg.range>=10:
    		print("gelöscht")
    		return
    	self.anchor_ranges.append(msg)
    	self.anchor_ranges = self.anchor_ranges[-10:]
    	if not self.initialized:
        	self.initialized = True
        	self.get_logger().info('first range received')
	
    def residiumf(self):
    	#print("residium")
    	
    	if len(self.anchor_ranges)>0:
    		
    			
    				
    		reslength=len(self.anchor_ranges)
    		self.rangesToGuess=np.arange(reslength).reshape(reslength,1)
    		self.rangesToGuess=np.array(self.rangesToGuess,dtype=np.float)
    		self.residium=np.arange(reslength).reshape(reslength,1)
    		self.residium=np.array(self.rangesToGuess,dtype=np.float)
    		self.counter=len(self.anchor_ranges)
    		for r in range(0,len(self.anchor_ranges)): 	
    			
    				self.rangesToGuess[r][0]=np.sqrt((self.x[0][0]-self.anchor_ranges[r].anchor.x)**2+(self.x[1][0]-self.anchor_ranges[r].anchor.y)**2+(self.x[2][0]-self.anchor_ranges[r].anchor.z)**2)
    				
    				self.residium[r][0]=(self.anchor_ranges[r].range-self.rangesToGuess[r])
    		
        			
        		
        			
        		
        	
        		      		
    def gradientf(self):
    	#print("gradient")
    	length=len(self.anchor_ranges)
    	
    	self.gradient=np.arange(3*length,dtype=np.float).reshape(length,3)
    	
    	for r in range(0,len(self.anchor_ranges)):
    		#print(self.rangesToGuess
    		fx=-((1/self.rangesToGuess[r][0])*(self.x[0][0]-self.anchor_ranges[r].anchor.x))
    		fy=-((1/self.rangesToGuess[r][0])*(self.x[1][0]-self.anchor_ranges[r].anchor.y))
    		fz=-((1/self.rangesToGuess[r][0])*(self.x[2][0]-self.anchor_ranges[r].anchor.z))
    		
    		self.gradient[r][0]= fx
    		self.gradient[r][1]= fy
    		self.gradient[r][2]= fz
    	#print(self.gradient)
    	
    def timer_cb(self):
        if not self.initialized:
            return
        #print("hallo")
        msg = PointStamped()
        msg.point.x, msg.point.y, msg.point.z = self.calculate_position()
        msg.header.frame_id = 'world'
        self.position_pub.publish(msg)
    
    def calculate_position(self):

        if len(self.anchor_ranges)<5:
            return 0.0, 0.0, 0.0 
        for i in range(0,9):
	  	
        	self.residiumf()
        	self.gradientf()
        	#y=np.matmul(np.linalg.pinv(self.gradient),self.residium)
        	#print(y)
       	self.x=self.x-np.dot(np.linalg.pinv(self.gradient),self.residium)
       	self.x[2][0]=0
        #print("calculate")
        #print(self.x)
       
     	
        return self.x[0][0],self.x[1][0],0.0


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
