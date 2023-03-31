import os
import json
import rospy
import socket
import h264decoder
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import String, Header

def start(PORT, TOPIC):

	pub = rospy.Publisher(TOPIC, Image, queue_size=30)		#Create a ROS publisher for the video topic
	rospy.init_node('pic_server', anonymous=True)			#Initialize the ROS node
	rate = rospy.Rate(300)
	bridge = CvBridge()						#Create a CvBridge to convert img between OpenCV & ROS
	decoder = h264decoder.H264Decoder()

	server_socket = socket.socket()
	server_socket.bind((('0.0.0.0'), PORT))
	server_socket.listen(0)

	connection = server_socket.accept()[0].makefile('rb')

	while  not rospy.is_shutdown():
		data = connection.read(1024)
		if not data:
			break

	frames = decoder.decode(data)
	for f in frames:
		(frame,w,h,ls) = f
		if frame is not None:
			img = Image()
			header = Header()
			header.stamp = rospy.Time.now()
			img.header = header
			img.data = frame
			img.width = w
			img.height = h
			img.step = ls
			img.encoding = "rgb8"
			pub.publish(img)

	rate.sleep()

if __name__ == '__main__':

	PORTS = [8000]
	TOPICS = ["/rpi_cam_node/image_raw"]

	NUM_PORTS = len(PORTS)

	if(len(TOPICS) != len(PORTS) or len(TOPICS) == 0 or len(PORTS) ==0):
		print("Error with RPI CAMERA FEED: Ports or topic list malformed")

	try:
		for i in range(0, NUM_PORTS):
			start(PORTS[i], TOPICS[i])
	except rospy.ROSInterruptException:
		pass
