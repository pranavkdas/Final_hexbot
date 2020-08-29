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
min_dist=10
max_dist=0
safe_distance=0

def go_left(value):
	global command
	global min_dist
	global max_dist
	msg = Twist()
	state_description = 'going_left'
	if command=='left':
		if(value>0.65):
			rospy.loginfo(state_description)
			msg.linear.y =5
			velocity_pub.publish(msg)
		else:
			msg.linear.y=0
			velocity_pub.publish(msg)
			command='front'

		min_dist=min(front,min_dist)
		max_dist=max(front,max_dist)

def go_front(value):
	global command
	msg = Twist()
	print(max_dist,min_dist)
	state_description = "going_forward" 
	if command=='front':
		if(value>max_dist-min_dist+1):
			rospy.loginfo(state_description)
			msg.linear.x = 10
			velocity_pub.publish(msg)
		elif (value>max_dist-min_dist+0.3):
			rospy.loginfo(state_description)
			msg.linear.x = 1
			velocity_pub.publish(msg)
		else:
			msg.linear.x=0
			velocity_pub.publish(msg)
			command='start_scanning_objects'

def adjusting_distance(value,dist,acc):
	global command
	global temp_command
	msg=Twist()

	s=np.sign(value-dist)
	if command=="after_torus_adjust_safe":
		s=6*s
	#	print(s)

	if abs(value-dist)>20*acc :
		msg.linear.x=5*s
		velocity_pub.publish(msg)
	elif abs(value-dist)>5*acc :
		msg.linear.x=2*s
		velocity_pub.publish(msg)
	elif abs(value-dist)>acc :
		msg.linear.x=0.5*s
		velocity_pub.publish(msg)
	else :
		msg.linear.x=0
		velocity_pub.publish(msg)
		if command=="adjust_torus":
			command="pickup_using_moveit"
		elif command=="after_torus_adjust_safe":
			command="start_scanning_objects"
			temp_command="object_detected"

def go_right(value):
	global command
	global right_value
	msg = Twist()
	state_description = "going_right" 
	if command=='right':
		if(value>0.75):
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
	while rospy.Time.now().to_sec()<t0+3.3/8:
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
	global min_dist
	global max_dist
	msg = Twist()
	global temp_command2
	state_description = "going_backward" 
	if temp_command2=="initial":
		if(value<2.5):
			rospy.loginfo(state_description)
			msg.linear.x = 8
			velocity_pub.publish(msg)
		else:
			temp_command2="off"
	else:
		if value>4:
			rospy.loginfo(state_description)
			msg.linear.x = 8
			velocity_pub.publish(msg)
		elif value>3 and value<4:
			rospy.loginfo(state_description)
			msg.linear.x = 1
			velocity_pub.publish(msg)
		else:
			msg.linear.x=0
			velocity_pub.publish(msg)
			#command="move_around_the_rods"	
			#temp_command="rod_not_detected"
			min_dist = 10
			max_dist=0
			command="scan_min_dist_target_rod"

def check_closest_rod(value):
	global command
	global min_dist
	global max_dist
	msg = Twist()
	state_description = 'checking_closest_rod'
	if command=='scan_min_dist_target_rod':
		if(value>0.75):
			rospy.loginfo(state_description)
			msg.linear.y=-5
			velocity_pub.publish(msg)
		else:
			msg.linear.y=0
			velocity_pub.publish(msg)
			command='go_front_to_rod'
			print(command)

		min_dist=min(front,min_dist)
		max_dist=max(front,max_dist)

def go_towards_rod(value):
	global command
	global temp_command
	msg = Twist()
	print(max_dist,min_dist)
	state_description = "going_forward" 
	if command=='go_front_to_rod':
		if(value>max_dist-min_dist+1):
			rospy.loginfo(state_description)
			msg.linear.x = 10
			velocity_pub.publish(msg)
		elif (value>max_dist-min_dist+0.3):
			rospy.loginfo(state_description)
			msg.linear.x = 1
			velocity_pub.publish(msg)
		else:
			msg.linear.x=0
			velocity_pub.publish(msg)
			print('started moving around rods')
			command="move_around_the_rods"	
			temp_command="rod_not_detected"

def moveit():
	global command
	print("Moveit picking up the torus")
	print(front)
	sleep(4)
	print(picked,'torus picked up')
	command="after_torus_adjust_safe"


def callback_frontlaser(msg):
	global command
	global front
	global left
	global back
	global right
	global safe_distance
	
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
	if command=="adjust_torus":
		adjusting_distance(front,0.16,0.01)
	if command=="pickup_using_moveit":
		moveit()
	if command=="after_torus_adjust_safe":
		adjusting_distance(front,safe_distance,0.05)
	if command=="scan_min_dist_target_rod":
		check_closest_rod(right)
	if command=="go_front_to_rod":
		go_towards_rod(front)
	
	

def isCentralised(cv_image):
	#cv_image= cv_image[300:700,200:600,:]
	imfinal = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	ret,thresh = cv2.threshold(imfinal,230,255,cv2.THRESH_BINARY_INV)
	image, contours, hierarchy= cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.imshow('error',thresh)
	cv2.waitKey(1)
	if len(contours)>0:
		ind=0
		dist=800
		x=0
		y=0
		for i in range(len(contours)):
			#cv_image = cv2.drawContours(cv_image, contours, 0, (0,255,0), 3)
			M = cv2.moments(contours[i])
			if M['m00']!=0:
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				if abs(cx-cv_image.shape[1]//2)<dist :
					dist=abs(cx-cv_image.shape[1]//2)
					ind=i
					x=cx
					y=cy


		#cv_image = cv2.circle(cv_image, (x,y), radius=5, color=(255,0,0), thickness=-1)
		#cv_image = cv2.circle(cv_image, (cv_image.shape[1]//2,cv_image.shape[0]//2), radius=10, color=(0,255,0), thickness=-1)
		a = cv_image.shape[1]//2
		cv2.imshow('frame',cv_image)
		cv2.waitKey(1)
		#print(abs(a-b<5), "is_centralised")
		return abs(a-x)<5

	else:
		cv2.imshow('frame',cv_image)
		cv2.waitKey(1)
		return False


def move_right() :
	v=Twist()
	v.linear.y= -0.5
	velocity_pub.publish(v)


def cv_to_detect_shape_color(cv_image):
	gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	_,thresh = cv2.threshold(gray,230,255,cv2.THRESH_BINARY_INV)
	_, contours, hierarchy= cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	ind=0
	dist=800
	for i in range(len(contours)) :
		M = cv2.moments(contours[i])
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])
		b=cv_image.shape[1]//2
		if abs(b-cx)<dist :
			dist=abs(b-cx)
			ind=i

	cnt=contours[ind]
	x,y,w,h=cv2.boundingRect(cnt)
	cx=x+w//2
	cy=y+h//2
	img=cv_image[int(max(cy-1.2*h,0)):int(min(cy+1.2*h,800)),int(max(cx-1.2*w,0)):int(min(cx+1.2*w,800))]
	
	img_rgb=img.copy()
	img_hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	_,thresh = cv2.threshold(gray,230,255,cv2.THRESH_BINARY_INV)
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
	#img=cv2.drawContours(img,[cnt],0,(0,255,0),3)
	x,y,w,h=cv2.boundingRect(cnt)
	_,(wr,hr),_=cv2.minAreaRect(cnt)
	ellipse=cv2.fitEllipse(cnt)
	#img = cv2.ellipse(img,ellipse,(0,0,255),2)
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
	cv2.imshow("mask",mask)
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
		elif hue<=23 :
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
	cv2.imshow('f2',img)
	cv2.waitKey(1)	
	#my_object="nuclear_fuel"
	if shape=="torus":
		my_object="nuclear_fuel"
	else :
		my_object="nuclear_waste"		
	return my_object, color

	#cv2.destroyAllWindows()

temp_command="object_not_detected"
placed = 0

def move_left():
	v=Twist()
	v.linear.y=0.5
	velocity_pub.publish(v)


def callback_opencv(data):
	global command
	global picked
	global temp_command
	global torus_colors
	global placed
	global front 
	global safe_distance

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
				print(shape,color)
				if shape=="nuclear_fuel" :
					v=Twist()
					velocity_pub.publish(v)
					torus_colors+=[color]
					safe_distance=front
					picked= picked+1
					command="adjust_torus"
				#if temp_command=="object detected":
					#move_right()
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
				move_left()
			elif temp_command=="rod_not_detected" :
				msg=Twist()
				msg.linear.y=0
				velocity_pub.publish(msg)
				print("ML checking for name of color")
				cv2.imwrite('/home/pranavkdas/simulation_ws/src/hexbot/ml_image.jpeg',cv_image)
				cv2.waitKey(1)
				os.system('python3 /home/pranavkdas/simulation_ws/src/hexbot/ml_new.py')
				with open("/home/pranavkdas/simulation_ws/src/hexbot/result.txt",'r') as file:
					color=file.read()
				sleep(6)
				print("The color written on this base is :", color)
				print("Moveit Placing the rod")
				sleep(6)
				temp_command="rod_detected"
				placed=placed+1
			else : 
				move_left()
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