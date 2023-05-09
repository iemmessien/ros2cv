# Import the necessary libraries
import rclpy						# ROS 2 Client Library for Python
from rclpy.node import Node			# Inherit from the Node Super Class to handle the creation of nodes

from std_msgs.msg import String		# Import built-in String type for a message type of String
from sensor_msgs.msg import Image	# Import built-in Image type for a message type of Image

from cv_bridge import CvBridge 		# Import CVBridge from cv_bridge for converting between ROS 2 Images and OpenCV Images
import cv2 							# Import the OpenCV library


# Create a subscriber class which is a subclass of the Node super class
class VisionSubscriber(Node):
	
	# Define the VisionSubscriber class's Constructor to set up the node which is an object of the Node super class
	def __init__(self):
		
		# Initiate the Node class's constructor and give it a node name
		super().__init__('vision_subscriber_node')
		
		# Create the subscriber with the type of message to subscribe to and the name of the topic to subscribe to
		self.subscription = self.create_subscription(String, 'video_frames_topic', self.listener_callback, 10)
		
		self.subscription	# Prevent unused variable warning
		
		# Create an object of the CVBridge class to convert from ROS 2 Image to OpenCV Image format
		#self.ros_cvbridge = CvBridge()


	# Create a Callback function that gets called at the specified publishing time period
	def listener_callback(self, message):
		# This function gets called at each time period
		
		# Display the received message on the console
		self.get_logger().info('I received: "%s"' % message.data)      #.info('Receiving...')
		
		# Retrieve the image data and the optional message
		# The 'imgmsg_to_cv2' method converts a ROS 2
		# image message to an OpenCV image
		#current_frame = self.ros_cvbridge.imgmsg_to_cv2(message)
		
		# Display the image data
		#cv2.imshow("Stream", current_frame)
		
		#cv2.waitKey(1)


def main(args=None):
	
	# Initialize the ROS Client Library for Python
	rclpy.init(args=args)
	
	# Create the node which is an object or instance of the VisionSubscriber class
	my_vision_subscriber = VisionSubscriber()
	
	# Spin the node so that the callback function is called
	rclpy.spin(my_vision_subscriber)
	
	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	my_vision_subscriber.destroy_node()
	
	# Shutdown the ROS Client Library for Python
	rclpy.shutdown()


if __name__ == '__main__':
	main()
	
