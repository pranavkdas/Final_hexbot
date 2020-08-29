#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
from time import sleep 
import os


left=20
front=20
command="start"
torus_colors=[]
picked=0
temp_command2="initial"


def go_left(value):
	global command
	msg = Twist()
	state_description = 'going_left'
	if command=='left':
		if(value>0.75):
			rospy.loginfo(state_description)
			msg.linear.y = 10
			velocity_pub.publish(msg)
		else:
			msg.linear.y=0
			velocity_pub.publish(msg)
			command='front'

def go_front(value):
	global command
	msg = Twist()
	state_description = "going_forward" 
	if command=='front':
		if(value>2):
			rospy.loginfo(state_description)
			msg.linear.x = 10
			velocity_pub.publish(msg)
		elif (value<2 and value>1):
			rospy.loginfo(state_description)
			msg.linear.x = 1
			velocity_pub.publish(msg)
		else:
			msg.linear.x=0
			velocity_pub.publish(msg)
			command='start_scanning_objects'

def go_right(value):
	global command
	global right_value
	msg = Twist()
	state_description = "going_right" 
	if command=='right':
		if(value>1):
			rospy.loginfo(state_description)
			msg.linear.y = -5
			velocity_pub.publish(msg)
		else:
			msg.linear.y=0
			velocity_pub.publish(msg)
			command='turn_around'

def about_turn() :
	global command
	sleep(1)
	msg = Twist()
	state_description = "turning_around"
	theta=0
	t0=rospy.Time.now().to_sec()
	while rospy.Time.now().to_sec()<t0+3.35/8:
		velocity_pub.publish(msg)
		rospy.loginfo(state_description)
		msg.angular.z=8
		print((rospy.Time.now().to_sec()-t0)*8)
		
	msg.angular.z=0
	velocity_pub.publish(msg)
	sleep(1)
	command="back"


def go_back(value):
	global command
	global temp_command
	msg = Twist()
	global temp_command2
	state_description = "going_backward" 
	if temp_command2=="initial":
		if(value<3):
			rospy.loginfo(state_description)
			msg.linear.x = 8
			velocity_pub.publish(msg)
		else:
			temp_command2="off"
	else:
		if value>3:
			rospy.loginfo(state_description)
			msg.linear.x = 8
			velocity_pub.publish(msg)
		elif value>1 and value<3:
			rospy.loginfo(state_description)
			msg.linear.x = 1
			velocity_pub.publish(msg)
		else:
			msg.linear.x=0
			velocity_pub.publish(msg)
			command="move_around_the_rods"	
			temp_command="rod_not_detected"


def callback_frontlaser(msg):
	global command
	global front
	global left
	global back
	global right
	
	left = min(msg.ranges[710:])
	front = min(msg.ranges[355:365])
	right = min(msg.ranges[:10])
	if command=='start':
		command='left'
	if command=='left':
		go_left(left)
	if command=='front':
		go_front(front)
	if command=="back":
		go_back(front)
	if command=="right":
		go_right(right)
	if command=="turn_around":
		about_turn()
	
x = 0
def isCentralised(cv_image):
	global x
	image = cv_image.copy()
	imfinal = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	ret,thresh = cv2.threshold(imfinal,230,255,cv2.THRESH_BINARY_INV)
	_, contours, hierarchy= cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	if(len(contours)>0):
		cv_image = cv2.drawContours(cv_image, contours, 0, (0,255,0), 3)
		M = cv2.moments(contours[0])
		if M['m00']!=0:
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])

			cv_image = cv2.circle(cv_image, (cx,cy), radius=5, color=(255,0,0), thickness=-1)
			cv_image = cv2.circle(cv_image, (cv_image.shape[1]//2,cv_image.shape[0]//2), radius=10, color=(0,255,0), thickness=-1)
			a = cv_image.shape[1]//2
			b = cx
			x=x+1
			cv2.imshow('frame',cv_image)
			cv2.waitKey(1)
			return abs(a-b)<5
		else:
			cv2.imshow('frame',cv_image)
			cv2.waitKey(1)
			return False


	else:
		cv2.imshow('frame',cv_image)
		cv2.waitKey(1)
		return False


def move_right() :
	v=Twist()
	v.linear.y= -0.5
	velocity_pub.publish(v)

def cv_to_detect_shape_color(img):

	img_rgb=img.copy()
	img_hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	_,thresh = cv2.threshold(gray,200,255,cv2.THRESH_BINARY_INV)
	_, contours, hierarchy= cv2.findContours(thresh,cv2.RETR_TREE,
		cv2.CHAIN_APPROX_SIMPLE)
	#print(len(contours))
	area=0
	for cnts in contours:
		if cv2.contourArea(cnts)>area:
			cnt=cnts
			area=cv2.contourArea(cnts)
	peri=cv2.arcLength(cnt,True)
	epsilon=0.01*peri
	approx=cv2.approxPolyDP(cnt,epsilon,True)
	img=cv2.drawContours(img,[cnt],0,(0,255,0),3)
	x,y,w,h=cv2.boundingRect(cnt)
	_,(wr,hr),_=cv2.minAreaRect(cnt)
	ellipse=cv2.fitEllipse(cnt)
	img = cv2.ellipse(img,ellipse,(0,0,255),2)
	a=ellipse[1][0]
	b=ellipse[1][1]
	if len(approx)<=6 :
		shape="cuboid"
	elif 0.95<w*1.0/h<1.05 :
		shape="sphere"
	elif wr*hr>w*h+0.1*area :
		shape="cone"
	elif w*1.0/h<1.5 :
		shape="it's tall not wide"
	elif abs(0.786*a*b-area)<0.05*area:
		shape="torus"
	else :
		shape="I don't know"
	mask=np.zeros_like(gray)
	mask=cv2.drawContours(mask,[cnt],0,255,-1)
	cv2.imshow('mask',mask)
	cv2.waitKey(1)
	mask=mask/255.0
	num=np.sum(mask)
	r=np.sum(img_rgb[:,:,2]*mask)/num
	g=np.sum(img_rgb[:,:,1]*mask)/num
	b=np.sum(img_rgb[:,:,0]*mask)/num
	color="unidentified"
	if (r-g)**2+(g-b)**2+(b-r)**2<1000 :
		bright=np.sum(gray*mask)/num
		if bright<10 :
			color="black"
		else :
			color="gray"
	else :
		img_hsv=img_hsv*1.0
		hue=np.sum(mask*img_hsv[:,:,0])/np.sum(mask)
		print(hue)
		if hue<=7 :
			color="red"
		elif hue<=20 :
			color="brown"
		elif hue<=45:
			color="yellow"
		elif hue<=75:
			color="green"
		elif hue<=105:
			color="cyan"
		elif hue<=150:
			color="navy_blue"
		else :
			color="red"

		#else :
			#colors=["green","navy_blue","cyan","yellow","brown"]
			#hue_values=[60,120,90,30,15]
			
	return shape, color
	cv2.destroyAllWindows()

temp_command="object_not_detected"
placed = 0

def callback_opencv(data):
	global command
	global picked
	global temp_command
	global torus_colors
	global placed

	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
	
	'''
	if command=='start_scanning_objects':
		command='start_opencv'
	if command=='start_opencv':
		bridge = CvBridge()
		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
		centrefinding(cv_image)
	elif command=='torus_search':
		bridge = CvBridge()
		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
		cv_to_detect_shape_color()[0]
	'''
	
	if command=="start_scanning_objects":
		if picked<4 :#right>0.75:
			if not isCentralised(cv_image) :
				temp_command="object_not_detected"
				move_right()
			elif temp_command=="object_not_detected" :
				shape,color=cv_to_detect_shape_color(cv_image)
				if shape=="torus" :
					v=Twist()
					velocity_pub.publish(v)
					torus_colors+=[color]
					print("Nuclear core of color",color,"detected")
					print("Moveit activated for pickup")
					sleep(6)
					print("Collected the core in 2s")
					print(3-picked,"cores left to be picked up")
					sleep(2)
					'''
					Pick the object and place it on the bot
					'''
					picked+=1
					temp_command="object detected"
				#if temp_command=="object detected":
					#move_right()
				else:
					print('Nuclear waste of', color, 'color')
			else : 
				move_right()


		else :
			command="right"


	if command=="move_around_the_rods" :
		cv2.imshow("frame", cv_image)
		cv2.waitKey(1)
		if placed<4 :
			if not isCentralised(cv_image) :
				temp_command="rod_not_detected"
				move_right()
			elif temp_command=="rod_not_detected" :
				msg= Twist()
				msg.linear.y=0
				velocity_pub.publish(msg)
				print("ML checking for name of color")
				cv2.imwrite('/home/pranavkdas/simulation_ws/src/hexbot/ml_image.jpg',cv_image)
				cv2.waitKey(1)
				os.system('python3 /home/pranavkdas/simulation_ws/src/hexbot/ml_new.py')
				sleep(6)
				with open("result.txt",'r') as file:
					color=file.read()
				print("The color written on this base is :", color)
				sleep(5)
				print("Moveit Placing the rod")
				sleep(2)
				temp_command="rod_detected"
				placed=placed+1
			else : 
				move_right()
		else:
			command="The end"	








def main():
	global command
	global left
	command='start'
	global velocity_pub
	global sub_laser_left
	rospy.init_node('main')
	velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	sub_laser_front = rospy.Subscriber('/hexbot/laser/scan', LaserScan , callback_frontlaser)
	image_sub = rospy.Subscriber("/hexbot/camera1/image_raw",Image,callback_opencv)
	#image_sub2 = rospy.Subscriber("/hexbot/camera1/image_raw",Image, pickup)
	#image_sub3 = rospy.Subscriber("/hexbot/camera1/image_raw",Image, see_tag_and_put)
	
	rospy.spin()

if __name__ == '__main__':
	main()