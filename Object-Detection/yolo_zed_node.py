# -*- coding: utf-8 -*-
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import time
import rosbag
import rospy
from sensor_msgs.msg import Image, CompressedImage, PointCloud2
from nav_msgs.msg import Odometry

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#####import for marker message###########
from visualization_msgs.msg import Marker
#########################################


############################ Extracting data from bag files #####################################

#bagfile = sys.argv[1]
left_images = [] # list to store the image numpy arrays extracted from bag file
depth_arr = []  # list to store the depth numpy arrays extracted from bag file

# extracting data from 'image_raw_color' topic
objects_locations = {}


def movebase_client(xgoal,xangel):

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = xgoal
   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = xangel

   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   
















def get_odmetry(odom):
       global current_odom
       
       current_odom = odom
       print("##geting odometry data")
       ########################## Obect Detection and Depth Calculation ##############################

def get_depth(x, y, w, h): # function to return distance of objects detected
	u = x+w//2  # center of image in u direction
	v = y+h//2  # center ofimage in v direction
	dist = -1
	depth_array = FinalDepth
	
	# to take care of boundary conditions for the bounding boxes
	left = u-15
	if (left<0): left=0
	right = u+15
	if (right>=1280): right=1280
	
	up = v-15
	if (up<0): up = 0
	down = v+15
	if (down>720): down = 720
	
	# extracting the distance value around the center pixel of the bounding box
	for row in range(up,down):		
		for col in range(left, right):
			# verifying if the value is not 'nan'
			if (not(math.isnan(float(depth_array[row][col])))):
				dist = depth_array[row, col]
				return dist  # returning when index with distance value is found
	return dist  # returning -1 as distance for this particular object



#FinalImage=[]
#FinalDepth=[]

image_ready=False
def zed_image_callback(data):
	#print("### image callback ###")
	bridge = CvBridge()
	imageAfterCVbrige = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
	global FinalImage
	FinalImage=imageAfterCVbrige
	#np_arr = np.frombuffer(data.data, dtype  = np.uint8)
	#if CompressedImage use cv2.imdecode
	#self.image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
	global image_ready
	image_ready = True

	#if self.debug:
	#	cv2.imshow('Image', self.image)
	#	if cv2.waitKey(25) & 0xFF == ord('q'):
	#		cv2.destroyAllWindows()

depth_ready=False
def zed_depth_callback( data):
	#print("### depth callback ###")
	bridge = CvBridge()
	depth_image = bridge.imgmsg_to_cv2(data,"32FC1")
	depth = np.array(depth_image, dtype=np.float32)
	global FinalDepth
	FinalDepth=depth
	#np_arr = np.frombuffer(data.data,dtype=np.float32)
	
	#self.depth = np_arr.reshape(720, 1280, 4)
	#print(np.unique(self.depth))
	
	global depth_ready
	depth_ready = True








#print('a7a')

pub = rospy.Publisher("zed_yolo_detect",String,queue_size=15)
rospy.init_node("imagetimer111", anonymous=True)
pub = rospy.Publisher("zed_yolo_detect",String,queue_size=15)
        #rospy.Subscriber('/left/image_rect_color', Image, self.zed_image_callback, queue_size=1)
rospy.Subscriber('/zed/camera/zed/zed_node/left_raw/image_raw_color', Image, zed_image_callback, queue_size=1)
        #rospy.Subscriber('/depth/depth_registered', Image, zed_depth_callback, queue_size=1)
rospy.Subscriber('/zed/camera/zed/zed_node/depth/depth_registered', Image, zed_depth_callback, queue_size=100)	
	
rospy.Subscriber('/zed/camera/zed/zed_node/odom', Odometry, get_odmetry)

####################instantiating the marker publisher#######################
marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)
#############################################################################


conf_thres = 0.5  #confidence threshold
nms_thres = 0.4  #Non-Maximum Suppression Threshold

labelsPath = 'coco.names'  # labels of the COCO dataset
labels = open(labelsPath).read().strip().split('\n')

# picking random values for the bounding boxes of different class of objects
colors = np.random.randint(0, 255, size=(len(labels), 3), dtype="uint8")

# loading the confi and weights file
model_config = 'yolov3.cfg'
model_weights = 'yolov3.weights'
# to read network model stored in Darknet model files
net = cv2.dnn.readNetFromDarknet(model_config, model_weights)

layerName = net.getLayerNames()
layerName = [layerName[i[0] - 1] for i in net.getUnconnectedOutLayers()]

# opening the writer to write the output into an avi file
count = 0
writer = None
(H, W) = (None, None)
#start = time.time()
flag=0
flagcount=0
accumlatedDistance=0
# traversing through the images captured in sequence
while (not rospy.is_shutdown()):
	rospy.Subscriber('/zed/camera/zed/zed_node/left_raw/image_raw_color', Image, zed_image_callback, queue_size=1)
        #rospy.Subscriber('/depth/depth_registered', Image, zed_depth_callback, queue_size=1)
	rospy.Subscriber('/zed/camera/zed/zed_node/depth/depth_registered', Image, zed_depth_callback, queue_size=100)	
	print('hii')
	if (image_ready ==True)and(depth_ready == True):
		print('Hello')
		img = FinalImage
		# extrating BGR file from a BGRA image
		b, g, r, a = cv2.split(img)
		image = np.stack((b, g, r), -1)
		image = cv2.resize(image, (img.shape[1], img.shape[0]))
		#print(image.shape)

		if W is None or  H is None: (H, W) = image.shape[:2]
		
		# creating 4-dimensional blob from series of images
		blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB = True, crop = False)
		net.setInput(blob)
		# running forward pass to compute output of layer
		layersOutputs = net.forward(layerName)

		boxes = []
		confidences = []
		classIDs = []

		for output in layersOutputs:
			for detection in output:
				scores = detection[5:]
				classID = np.argmax(scores)
				confidence = scores[classID]
				# picking the detectiong with confidence greater than threshold value
				if confidence > conf_thres:
					box = detection[0:4] * np.array([W, H, W, H])
					(centerX, centerY,  width, height) = box.astype('int')
					x = int(centerX - (width/2))
					y = int(centerY - (height/2))
				
					# storing object's coordinates, confidence and classID
					boxes.append([x, y, int(width), int(height)])
					confidences.append(float(confidence))
					classIDs.append(classID)

		#Non Maxima Suppression
		detectionNMS = cv2.dnn.NMSBoxes(boxes, confidences, conf_thres, nms_thres)

		if(len(detectionNMS) > 0):
			for i in detectionNMS.flatten():
				(x, y) = (boxes[i][0], boxes[i][1])
				(w, h) = (boxes[i][2], boxes[i][3])
				# extracting depth of every object in the frame
				d = get_depth(x, y, w, h)

				color = [int(c) for c in colors[classIDs[i]]]
				cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)

				######## instantiating the marker object and specifying its attributes ############
				
				marker = Marker()
				marker.header.frame_id = "map"
				marker.header.stamp = rospy.Time.now()

				# set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
				marker.type = 9
				marker.id = 0

				# Set the scale of the marker
				marker.scale.x = 0.2
				marker.scale.y = 0.2
				marker.scale.z = 0.2

				# Set the color
				marker.color.r = 0.0
				marker.color.g = 1.0
				marker.color.b = 0.0
				marker.color.a = 1.0
				
				
				###################################################################################
				
				if (d == -1):  # if distance=-1, it means  that no value for depth was found
					text = '{}: ~~~'.format(labels[classIDs[i]][:-1])
				else:
					text = '{}: {:.3f}m'.format(labels[classIDs[i]][:-1], d)
					##object_pose = current_odom.pose.pose
					############ Set the pose of the marker in which it will appear on map ###############
					marker.text = labels[classIDs[i]][:-1]
					if marker.text=='chai':
						
						flagcount+=1
						
						accumlatedDistance+=d/10
						print(flagcount,accumlatedDistance)

					#marker.pose.position.x = d
					#marker.pose.position.y = float(object_pose[2])
					#marker.pose.position.z = float(object_pose[3])
					#marker.pose.orientation.x = float(object_pose[4])
					#marker.pose.orientation.y = float(object_pose[5])
					#marker.pose.orientation.z = float(object_pose[6])
					#marker.pose.orientation.w = float(object_pose[7])
						if flagcount==10:
							print('ana waslt')
							marker.pose = current_odom.pose.pose
							marker.pose.position.x = accumlatedDistance
							movebase_client(accumlatedDistance-0.2,1.0)
							movebase_client(0,0.5)
					# break
					# if not (np.isinf(d) or np.isnan(d)):
							marker_pub.publish(marker)
					############-~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~-###############
					
				
				pub.publish(text)	
				# writing object class and its corresponding distance on the image
				cv2.putText(image, text, (x, y+5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
				cv2.imshow("ZED", img)
                
		
		#if writer is None:
		#	fourcc = cv2.VideoWriter_fourcc(*"MJPG")
		#	writer = cv2.VideoWriter("output.avi", fourcc, 10, (W,H), True)
		#writer.write(image)
		#print ("writing frame", count)
		#count=count+1
	# file finished writing
		#writer.release()

#end = time.time()
#print "time taken:", end-start
