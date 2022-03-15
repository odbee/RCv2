# Basic ROS 2 program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries

import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import Int32
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

import numpy as np
import os
from urllib.request import urlopen



def compareDescToDescriptorsAndReturnBestIndex(sourceDesc,ListOfDescriptors, thresh =15):
  matcher =cv2.BFMatcher()
  matchList=[]
  finalVal=-1
  try:
      for desc in ListOfDescriptors:
          matches = matcher.knnMatch(desc,sourceDesc,k=2)
          good = []
          for m,n  in matches:
              if m.distance < 0.75* n.distance:
                  good.append([m])
          matchList.append(len(good))
  except:
      pass
  if len(matchList)!=0:
      if max(matchList)> thresh:
        # TODO ERROR HANDLE max() arg is an empty sequence. PUT A TRY AND EXPECT
          finalVal= matchList.index(max(matchList))

  return finalVal


def url_to_image(url):
      # download the image, convert it to a NumPy array, and then read
      # it into OpenCV format
      resp = urlopen(url)
      image = np.asarray(bytearray(resp.read()), dtype="uint8")
      image = cv2.imdecode(image, cv2.IMREAD_COLOR)
      # return the image
      return image

class ImagePublisher(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_publisher')
      
    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.videopublisher_ = self.create_publisher(Image, 'video_frames', 10)
    self.objectpublisher_=self.create_publisher(Int32, 'which_object',1)
      
    # We will publish a message every 0.1 seconds
    timer_period = 0.2  # seconds
      
    

    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)
         
    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    self.cap = cv2.VideoCapture(0)
    self.orb=cv2.ORB_create(nfeatures=1000)
    # self.path='/home/ed/workspace/ocv_ws/src/cv_basics/cv_basics/ImageQuery'
    # self.images=[]
    # self.classNames=[]
    # self.myList=os.listdir(self.path)
    # for cl in self.myList:
    #   imgCur=cv2.imread(f'{self.path}/{cl}',0)
    #   self.images.append(imgCur)
    #   self.classNames.append(cl)
    
    # def findDescriptors(imgs):
    #   ListOfDescriptors=[]
    #   for img in imgs:
    #       kp, des = self.orb.detectAndCompute(img,None)
    #       ListOfDescriptors.append(des)
    #   return ListOfDescriptors # returns array of descriptors


    # self.descriptorList = findDescriptors(self.images)

      

    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   

  def timer_callback(self):
    
    """
    Callback function.
    This function gets called every 0.1 seconds.
    """
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    ret, frame = self.cap.read()
    ocvimage=frame
    which_object_msg=Int32()


    
    ret = True
    if ret == True:
      ocvimage= url_to_image("http://192.168.1.104:4242/current.jpg?type=color")

    self.videopublisher_.publish(self.br.cv2_to_imgmsg(ocvimage))
    self.objectpublisher_.publish(which_object_msg)

    # Display the message on the console
    self.get_logger().info('Publishing video frame')
  
def main(args=None):
  # Initialize the rclpy lubrary
  rclpy.init(args=args)
  
  # Create the node
  image_publisher = ImagePublisher()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_publisher)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_publisher.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()

