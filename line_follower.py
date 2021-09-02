"""
Team 7 - PutareAI
"""


#!/usr/bin/env python3

import rospy
import cv2, cv_bridge, numpy
import time

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class FollowTrack:

        def image_callback(self, msg):

                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                
                # Mask for detecting Yellow in the hsv form
                lower_yellow = numpy.array([ 10, 10, 10])
                upper_yellow = numpy.array([255, 255, 250])
                mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

                height, w, depth = image.shape
                top = int(3*height/4)
                #bottom = 3*height/4 + 20
                mask[0:top, 0:w] = 0
                #mask[bottom:height, 0:w] = 0
                
                #print("Width/2:" ,(w/2))

                M = cv2.moments(mask)
                
                if M['m00'] > 0:   # (To avoid the division by zero error)
                
                # Plotting the centroid of the detected blob (centre of the track in this case)
                  cx = int(M['m10']/M['m00'])
                  cy = int(M['m01']/M['m00'])
                  cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                  
                  i = cv2.resize(image, (400,400))
                  
                  error = cx - w/2
                  #print("\nError value:", error)
                  
                  if -500 <= error <= 500:
                    print("Moving forward....")
                    self.twist_obj.linear.x = 0.2
                    self.pub.publish(self.twist_obj)
                    cv2.imshow("bot_view", i)
                    cv2.waitKey(4)
                  
                  else:
                    if error < 0:
                      print("Turning left...")
                    else:
                      print("Turning right...")

                    self.twist_obj.angular.z = -float(error) / 5000
                    self.twist_obj.linear.x = 0.005
                    self.pub.publish(self.twist_obj)

                    cv2.imshow("bot_view", i)
                    cv2.waitKey(4)
                
        def __init__(self):
		
		 # CvBridge class object for conversion of ROS images into OpenCV images
                self.bridge = cv_bridge.CvBridge()
                
                # Publisher and Subscriber nodes
                self.sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
                self.pub = rospy.Publisher('/cmd_vel',Twist, queue_size=1)
                
                # Twist message Object declaration
                self.twist_obj = Twist()                 
                         
if __name__ == '__main__':
  
  try:
    while not rospy.is_shutdown():
  
      rospy.init_node('line_follower')
      follower_obj = FollowTrack()
      rospy.spin()
  
  except rospy.ROSInterruptException:
    pass
	
