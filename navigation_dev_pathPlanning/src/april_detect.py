#!/usr/bin/env python
import rospy
import cv2
import apriltag
from cv_bridge import CvBridge
from std_msgs.msg import String
#from std_msgs.msg import Float32MultiArray
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import TMatrix 

from sensor_msgs.msg import Image 
from sensor_msgs.msg import CameraInfo 

bridge = CvBridge()
april_pub = rospy.Publisher('/april_detections', Image, queue_size=2)
poses_pub = rospy.Publisher('/tag_poses', AprilDetections, queue_size=2)
K_params = [] 

def process_tags(detector, K, s, results, image, visualize=True):


    """

        detector: apriltag.Detector object 
        K: intrinsic parameters (fx, fy, cx, cy)
        s: tag size in meters
        image: input rectified image

        returns: transformation dictionary indexed by tag_ids

    """

    tfs = {}
    poses_msg = AprilDetections()
    #poses_msg.data = []

    for r in results:
    	# extract the bounding box (x, y)-coordinates for the AprilTag
    	# and convert each of the (x, y)-coordinate pairs to integers
        #print(r) 
        T, _, _ = detector.detection_pose(r, K, s)
        #tfs[r.tag_id] = T

        T_list = T.flatten().tolist()
        tm = TMatrix()
        tm.matrix += T_list

        poses_msg.detections.append(tm)
        poses_msg.ids.append(r.tag_id)


        #poses_msg.data.append(r.tag_id)
        #poses_msg.data += T_list

        if visualize:
    	    (ptA, ptB, ptC, ptD) = r.corners
    	    ptB = (int(ptB[0]), int(ptB[1]))
    	    ptC = (int(ptC[0]), int(ptC[1]))
    	    ptD = (int(ptD[0]), int(ptD[1]))
    	    ptA = (int(ptA[0]), int(ptA[1]))
    	    # draw the bounding box of the AprilTag detection
    	    cv2.line(image, ptA, ptB, (0, 255, 0), 2)
    	    cv2.line(image, ptB, ptC, (0, 255, 0), 2)
    	    cv2.line(image, ptC, ptD, (0, 255, 0), 2)
    	    cv2.line(image, ptD, ptA, (0, 255, 0), 2)
    	    # draw the center (x, y)-coordinates of the AprilTag
    	    (cX, cY) = (int(r.center[0]), int(r.center[1]))
    	    cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
    	    # draw the tag family on the image
    	    tagFamily = r.tag_family.decode("utf-8")
    	    cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
    		cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        print("[INFO] tag family: {}".format(tagFamily))
        # print("K")
        # print(K)
        # print("rseults")
        # print(results)
        # print("poses msg")
        # print(poses_msg)
    return image, poses_msg

def params_callback(msg):

    # fx, fy, cx, cy
    global K_params
    #K_params = [msg.K[0], msg.K[4], msg.K[2], msg.K[5]]
    K_params = [msg.P[0], msg.P[5], msg.P[2], msg.P[6]]
    

def cam_callback(msg):
    if len(K_params) == 0:
        rospy.loginfo("Waiting for camera parameters.")
        return

    # print("printing fx fy cx and cy part 2 ::: ")
    # print(K_params)
    # convert 'Image' to 'numpy' array
    img_np = bridge.imgmsg_to_cv2(msg) 

    # convert to grayscale
    img_gray_np = cv2.cvtColor(img_np, cv2.COLOR_BGR2GRAY)

    # AprilTag detection 
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(img_gray_np)

    # [fx, fy, cx, cy]
    #K = [787.77467, 791.02202, 671.041, 319.57023] 
    #K = [593.55524, 730.58258, 687.50338, 313.33228] 
    s = 0.126
    img_dets, tfs = process_tags(detector, K_params, s, results, img_np)
    #tfs.header.stamp = msg.header.stamp
    tfs.header.stamp = rospy.Time.now() 
    img = bridge.cv2_to_imgmsg(img_dets) 
    april_pub.publish(img)
    poses_pub.publish(tfs)


if __name__ == "__main__":
    rospy.init_node('april_localizer')
    rospy.Subscriber("/jetbot_camera/image_rect_color", Image, cam_callback)
    rospy.Subscriber("/jetbot_camera/camera_info", CameraInfo, params_callback)
    rospy.spin()
