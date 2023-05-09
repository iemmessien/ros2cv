# Import the necessary libraries
import rclpy						# ROS 2 Client Library for Python
from rclpy.node import Node			# Inherit from the Node Super Class to handle the creation of nodes

from std_msgs.msg import String		# Import built-in String type for a message type of String
from sensor_msgs.msg import Image	# Import built-in Image type for a message type of Image

from cv_bridge import CvBridge 		# Import CVBridge from cv_bridge for converting between ROS 2 Images and OpenCV Images
import cv2 							# Import the OpenCV library

'''
# Import other necessary packages
from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import numpy as np
from numpy import array
import imutils
import cv2
import math

# Compute the mid-point of two axes to find the center of intersection
def midpoint(ptA, ptB):
	return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)
'''

# Create a publisher class which is a subclass of the Node super class
class VisionPublisher(Node):
	
	# Define the VisionPublisher class's Constructor to set up the node which is an object of the Node super class
	def __init__(self):
		
		# Initiate the Node class's constructor and give it a node name
		super().__init__('vision_publisher_node')
		
		# Create the publisher with the type of message to be published and the name of the topic to publish on
		# The last argument is the queue size; which is given a queue size of 10.
		# "Queue size" is a required QoS (quality of service) setting that limits the amount of queued messages
		# if a subscriber is not receiving them fast enough.
		self.publisher_ = self.create_publisher(String, 'video_frames_topic', 10)
		
		timer_period = 0.5	# Publish a message every 0.5 seconds (A frequency of 2Hz)
		
		# Create the timer
		self.timer = self.create_timer(timer_period, self.timer_callback)
		
		# Create a VideoCapture object from the OpenCV libraray
		#self.vid_cap = cv2.VideoCapture(0)
		self.counter = 0
		
		# Create an object of the CVBridge class to convert from OpenCV Image to ROS 2 Image format
		#self.cv_rosbridge = CvBridge()
	
	#def capture_video():
	
	# Create a Callback function that gets called at each time period
	def timer_callback(self):
		# This function gets called at each time period
		
		my_message = String()
		my_message.data = 'Hello, World: %d' % self.counter
		
		'''
		# Capture each frame from the camera
		ret, image = self.vid_cap.read()
		
		# Check if a frame is returned
		if ret == True:
			
			orig = image.copy()
	
			# Convert the frame to grayscale, blur it, and find edges
			gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
			gray = cv2.GaussianBlur(gray, (3, 3), 0)
			edged = cv2.Canny(image, 50, 100)
			
			# Perform dilation + erosion to close gaps in between edges
			edged = cv2.dilate(edged, None, iterations=1)
			edged = cv2.erode(edged, None, iterations=1)

			# Find contours in the edge map
			cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
				cv2.CHAIN_APPROX_SIMPLE)
			cnts = imutils.grab_contours(cnts)

			# Sort the contours from left-to-right and initialize the
			# 'pixels per metric' calibration variable
			cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:5]
			pixelsPerMetric = 100
				
			objectNum = 0
			dimA = 0.0
			dimB = 0.0
			centerX = 0.0
			centerY = 0.0
			orientation = 0.0

			# Loop over the contours individually
			for c in cnts:
				
				# If the contour is not sufficiently large, ignore it
				if cv2.contourArea(c) < 3200:
					continue
				
				# Compute the rotated bounding box of the contour
				box = cv2.minAreaRect(c)
				box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
				box = np.array(box, dtype="int")
				
				# Order the points in the contour such that they appear
				# in top-left, top-right, bottom-right, and bottom-left
				# order, then draw the outline of the rotated bounding
				# box
				box = perspective.order_points(box)
				cv2.drawContours(orig, [box.astype("int")], -1, (0, 255, 0), 2)
				
				# Loop over the original points and draw them
				for (x, y) in box:
					cv2.circle(orig, (int(x), int(y)), 5, (0, 0, 255), -1)
				
				# Unpack the ordered bounding box, then compute the mid-point
				# between the top-left and top-right coordinates, followed by
				# the mid-point between bottom-left and bottom-right coordinates
				(tl, tr, br, bl) = box
				(tltrX, tltrY) = midpoint(tl, tr)
				(blbrX, blbrY) = midpoint(bl, br)
				
				# Compute the mid-point between the top-left and top-right points,
				# followed by the mid-point between the top-righ and bottom-right
				(tlblX, tlblY) = midpoint(tl, bl)
				(trbrX, trbrY) = midpoint(tr, br)
				
				# Draw the mid-points on the image
				cv2.circle(orig, (int(tltrX), int(tltrY)), 5, (255, 0, 0), -1)
				cv2.circle(orig, (int(blbrX), int(blbrY)), 5, (255, 0, 0), -1)
				cv2.circle(orig, (int(tlblX), int(tlblY)), 5, (255, 0, 0), -1)
				cv2.circle(orig, (int(trbrX), int(trbrY)), 5, (255, 0, 0), -1)
				
				# Draw lines between the mid-points
				cv2.line(orig, (int(tltrX), int(tltrY)), (int(blbrX), int(blbrY)),
					(255, 0, 255), 2)
				cv2.line(orig, (int(tlblX), int(tlblY)), (int(trbrX), int(trbrY)),
					(255, 0, 255), 2)
				
				# Compute angle at which to pick up object
				angle = int(math.atan((-tlblY + trbrY) / (trbrX - tlblX)) * 180 / math.pi)
			
				# Compute the Euclidean distance between the mid-points
				dA = dist.euclidean((tltrX, tltrY), (blbrX, blbrY))
				dB = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))
					
				if dB >= dA:
					orientation = angle + 90
			
				# Compute the size of the object
				dimA = dA / pixelsPerMetric
				dimB = dB / pixelsPerMetric
				
				# Find the mid-point of the bounding box for each object
				(centerX, centerY) = midpoint(tl, br)
				
				objectNum += 1
				
				# Draw the object sizes on the image
				cv2.putText(orig, "{:.1f}in".format(dimA),
					(int(tltrX - 15), int(tltrY - 10)), cv2.FONT_HERSHEY_SIMPLEX,
					0.65, (0, 0, 255), 1)
				cv2.putText(orig, "{:.1f}in".format(dimB),
					(int(trbrX + 10), int(trbrY)), cv2.FONT_HERSHEY_SIMPLEX,
					0.65, (0, 0, 255), 1)
				cv2.putText(orig, "{}".format(objectNum),
					(int(tl[0] - 20), int(tl[1] - 10)), cv2.FONT_HERSHEY_SIMPLEX,
					1.0, (255, 0, 255), 2)
			
			# Publish the image data with an optional message
			# The 'cv2_to_imgmsg' method of the CVBridge class converts an OpenCV
			# Image to a ROS 2 Image format
			self.publisher_.publish(self.cv_rosbridge.cv2_to_imgmsg(orig))
		
		# Capture frame end
		'''
		
		# Display a message on the console
		self.publisher_.publish(my_message)
		self.get_logger().info('Publishing: "%s"' % my_message.data)       #.info('Publishing...')
		self.counter += 1		# Keep count at each publishing time interval


def main(args=None):
	
	# Initialize the ROS Client Library for Python
	rclpy.init(args=args)
	
	# Create the node which is an object or instance of the VisionPublisher class
	my_vision_publisher = VisionPublisher()
	
	# Spin the node so that the callback function is called
	rclpy.spin(my_vision_publisher)
	
	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	my_vision_publisher.destroy_node()
	
	# Shutdown the ROS Client Library for Python
	rclpy.shutdown()


if __name__ == '__main__':
	main()
	
