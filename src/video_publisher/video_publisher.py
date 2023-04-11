import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from h264_image_transport.msg import h264Packet

def video_publisher():
	rospy.init_node('video_publisher', anonymous=True)
	video_pub = rospy.Publisher('video', Image, queue_size=10)
	bridge = CvBridge()

	while not rospy.is_shutdown():
		packet=rospy.wait_for_message('h264', H264Packet)
		frame = bridge.imgmsg_to_cv2(packet.to_image(), desired_encoding='passthrough')
		img_msg=bridge.cv2_to_imgmsg(frame, encoding='bgr8')
		video_pub.publish(img_msg)

if __name__ == '__main__':
	try:
		video_publisher()
	except rospy.ROSInterruptException:
		pass
