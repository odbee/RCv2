# Basics ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import Int32
from adam_msgs.srv import SendOrderReturnResult
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

from ament_index_python.packages import get_package_share_directory
import os


class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
    self.dir = os.path.join(get_package_share_directory('opencv_port'), 'ImageQuery')

    self.subscription = self.create_subscription(
      Image, 
      'video_frames', 
      self.listener_callback, 
      10)
    self.ocv_service=self.create_service(
      SendOrderReturnResult,
      'ocv_service',
      self.ocv_callback
      )
    
    self.ocvpublisher_=self.create_publisher(
      Int32,
      'objectseen',
      10
    )
    
    self.objectpublisher_=self.create_publisher(Int32, 'which_object',1)
    self.subscription # prevent unused variable warning
    self.which_object_msg=Int32()
    self.ready=False

    self.detector=cv2.ORB_create(nfeatures=300)
    # self.detector=cv2.SIFT_create(nfeatures=minHessian)
    self.matcher =cv2.BFMatcher()
    # self.matcher = cv2.DescriptorMatcher_create(cv2.DescriptorMatcher_FLANNBASED)
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    self.setup_featuredetection()
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    # self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    if self.ready:
      current_frame = self.br.imgmsg_to_cv2(data)
      kp1,des1=self.detector.detectAndCompute(current_frame,None)

      bestList=self.compareStreamToDescriptorsAndReturnList(des1,self.descriptorList)
      bestIndex=bestList.index(max(bestList))
      ll= len(bestList)
      for i in range(len(bestList)):
        strin = self.classNames[i] + ' ' + str(bestList[i])
        cv2.putText(current_frame,strin,(50,50+30*i),cv2.FONT_HERSHEY_COMPLEX,0.7,(0,0,255),2,4)
      # if bestIndex != -1:
      #   cv2.putText(current_frame,self.classNames[bestIndex],(50,50),cv2.FONT_HERSHEY_COMPLEX,1,(0,0,255),2,4)

      self.get_logger().info('list: %s' % str(bestList))


      self.which_object_msg.data=bestList.index(max(bestList))
      self.ocvpublisher_.publish(self.which_object_msg)
      # Display image
      cv2.imshow("camera", current_frame)
      
      cv2.waitKey(1)

  def ocv_callback(self,request, response):
    # if request.order == "a"
    response.result = self.which_object_msg.data
    return response

  def findDescriptors(self,imgs):
    ListOfDescriptors=[]
    for img in imgs:
        kp, des = self.detector.detectAndCompute(img,None)
        ListOfDescriptors.append(des)
    return ListOfDescriptors # returns array of descriptors


  def setup_featuredetection(self):

    self.images=[]
    self.classNames=[]
    self.myList=os.listdir(self.dir)
    for cl in self.myList:
      imgCur=cv2.imread(f'{self.dir}/{cl}',0)
      self.images.append(imgCur)
      self.classNames.append(cl)
    self.descriptorList= self.findDescriptors(self.images)
    self.ready=True

  
  def compareStreamToDescriptorsAndReturnList(self, sourceDesc,ListOfDescriptors, thresh =15):


    matchList=[]
    finalVal=-1
    try:
        for desc in ListOfDescriptors:
            matches = self.matcher.knnMatch(desc,sourceDesc,k=2)
            good = []
            for m,n  in matches:
                if m.distance < 0.75* n.distance:
                    good.append([m])
            matchList.append(len(good))
    except:
        self.get_logger().info('PASS IN COMPSTREAMTODESC')
        pass
    if len(matchList)!=0:
        if max(matchList)> thresh:
            finalVal= matchList.index(max(matchList))

    return matchList


    


  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
