#!/usr/bin/env python

# -- IMPORT --
import numpy as np
import cv2
from tf import transformations as tft
# Ros
import rospy
import cv_bridge
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
# Ros ggcnn_humanseg
from ggcnn_humanseg_ros.msg import GraspPrediction
from helper_ggcnn.pre_processing import Preparations
from helper_ggcnn.ggcnn import process_depth_image, process_depth_mask, predict
from helper_ggcnn.gridshow import gridshow
import helper_ggcnn.tf_helpers as tfh
# Ros 3rd party package darknet_ros
from darknet_ros_msgs.msg import BoundingBoxes


class GGCNN_humanseg:

    def __init__(self):

        # Class for preparations
        self.prep = Preparations()

        # Parameter
        self.init_depth = False
        self.init_body = False
        self.init_hand = False
        self.init_box = False

        self.cam_frame = rospy.get_param('/ggcnn_humanseg/camera_info/camera_frame')
        self.base_frame = rospy.get_param('/ggcnn_humanseg/camera_info/robot_base_frame')
        self.cam_fov = rospy.get_param('/ggcnn_humanseg/camera_info/fov')

        self.image_topic = rospy.get_param('/ggcnn_humanseg/camera/image')
        self.depth_topic = rospy.get_param('/ggcnn_humanseg/camera/depth')
        self.bodyparts_topic = rospy.get_param('/ggcnn_humanseg/subscription/bodyparts')
        self.egohands_topic = rospy.get_param('/ggcnn_humanseg/subscription/egohands')
        self.darknet_topic = rospy.get_param('/ggcnn_humanseg/subscription/darknet')
        self.interface_topic = rospy.get_param('/ggcnn_humanseg/interface/topic')
        #self.interface_darknet = rospy.get_param('/ggcnn_humanseg/interface/darknet')

        self.crop_size = rospy.get_param('/ggcnn_humanseg/ggcnn/crop_size')
        self.crop_offset = rospy.get_param('/ggcnn_humanseg/ggcnn/crop_offset')
        self.out_size = rospy.get_param('/ggcnn_humanseg/ggcnn/out_size')

        self.dist_ignore = rospy.get_param('/ggcnn_humanseg/robot/dist_ignore')
        self.grip_height = rospy.get_param('/ggcnn_humanseg/robot/grip_width')

        self.visualization_topic = rospy.get_param('/ggcnn_humanseg/visualization/topic')
        self.visualization_type = rospy.get_param('/ggcnn_humanseg/visualization/activated')

        # Camera info
        cam_info = rospy.get_param('/ggcnn_humanseg/camera_info/info')
        cam_info_msg = rospy.wait_for_message(cam_info, CameraInfo)
        self.cam_K = np.array(cam_info_msg.K).reshape((3, 3))

        # Images & masks
        self.image = None
        self.depth = None
        self.depth_nan = None
        self.mask_body = None
        self.mask_hand = None
        self.best_box = None

        # Init
        self.bridge = cv_bridge.CvBridge()
        self.last_image_pose = None

        # Publisher
        self.pub_ggcnn = rospy.Publisher(self.interface_topic, GraspPrediction, queue_size=1)
        #self.pub_darknet = rospy.Publisher(self.interface_darknet, Image, queue_size=1)
        
        if self.visualization_type:
            self.pub_visualization = rospy.Publisher(self.visualization_topic, Image, queue_size=1)

        # Subscriber
        rospy.Subscriber(self.image_topic, Image, self._callback_image, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self._callback_depth, queue_size=1)
        rospy.Subscriber(self.bodyparts_topic, CompressedImage, self._callback_bodyparts, queue_size=1)
        rospy.Subscriber(self.egohands_topic, CompressedImage, self._callback_egohands, queue_size=1)
        rospy.Subscriber(self.darknet_topic, BoundingBoxes, self._callback_darknet, queue_size=1)
        
        # Feedback
        print("GGCNN human segmentation publisher up and running")


    # GGCNN
    def _ggcnn(self): 

        t_start = rospy.get_time()       

        # Camera pose
        cam_p = self.last_image_pose.position
        cam_rot = tft.quaternion_matrix(tfh.quaternion_to_list(self.last_image_pose.orientation))[0:3, 0:3]
        
        # Prepare image & mask
        depth_bg, mask_bg = self.prep.prepare_image_mask(depth=self.depth, depth_nan=self.depth_nan, 
                                                         mask_body=self.mask_body, mask_hand=self.mask_hand, 
                                                         dist_obj=self.best_box[4], dist_ignore=self.dist_ignore, 
                                                         grip_height=self.grip_height)
        
        # Prepare image & mask for ggcnn
        depth_ggcnn = process_depth_image(depth=depth_bg, crop_size=self.crop_size, 
                                          out_size=self.out_size, crop_y_offset=self.crop_offset)

        mask_ggcnn = process_depth_image(depth=mask_bg, crop_size=self.crop_size, 
                                        out_size=self.out_size, crop_y_offset=self.crop_offset)
        
        # Calculate GGCNN
        points, angle, width_img, _ = predict(depth=depth_ggcnn, mask=mask_ggcnn,
                                              crop_size=self.crop_size, out_size=self.out_size, 
                                              crop_y_offset=self.crop_offset, filters=(2.0, 2.0, 2.0))

        # Post-processing angle
        angle -= np.arcsin(cam_rot[0, 1])  # Correct for the rotation of the camera
        angle = (angle + np.pi/2) % np.pi - np.pi/2  # Wrap [-np.pi/2, np.pi/2]

        # Convert to 3D positions.
        imh, imw = self.depth.shape
        x = ((np.vstack((np.linspace((imw - self.crop_size) // 2, (imw - self.crop_size) // 2 + self.crop_size, depth_ggcnn.shape[1], np.float), )*depth_ggcnn.shape[0]) - self.cam_K[0, 2])/self.cam_K[0, 0] * depth_ggcnn).flatten()
        y = ((np.vstack((np.linspace((imh - self.crop_size) // 2 - self.crop_offset, (imh - self.crop_size) // 2 + self.crop_size - self.crop_offset, depth_ggcnn.shape[0], np.float), )*depth_ggcnn.shape[1]).T - self.cam_K[1,2])/self.cam_K[1, 1] * depth_ggcnn).flatten()
        pos = np.dot(cam_rot, np.stack((x, y, depth_ggcnn.flatten()))).T + np.array([[cam_p.x, cam_p.y, cam_p.z]])

        width_m = width_img / 300.0 * 2.0 * depth_ggcnn * np.tan(self.cam_fov * self.crop_size/self.depth.shape[0] / 2.0 / 180.0 * np.pi)

        best_g = np.argmax(points)
        best_g_unr = np.unravel_index(best_g, points.shape)

        # Create message
        pred = GraspPrediction()
        pred.success = True
        pred.pose.position.x = pos[best_g, 0]
        pred.pose.position.y = pos[best_g, 1]
        pred.pose.position.z = pos[best_g, 2]
        pred.pose.orientation = tfh.list_to_quaternion(tft.quaternion_from_euler(np.pi, 0, ((angle[best_g_unr]%np.pi) - np.pi/2)))
        pred.width = width_m[best_g_unr]
        pred.quality = points[best_g_unr]
        
        # Visualization
        if self.visualization_type:

            # Create heat map
            heatmap = points.copy()*255
            heatmap = heatmap.astype(np.uint8)
            heatmap = cv2.applyColorMap(heatmap, colormap=cv2.COLORMAP_JET)
            heatmap = cv2.warpAffine(heatmap, cv2.getRotationMatrix2D((self.out_size/2,self.out_size/2), 270, 1), (self.out_size,self.out_size))
            heatmap = cv2.flip(heatmap, 1)

            # Fit heatmap into RGB frame
            imgw, imgh, _ = self.image.shape
            x_min = (imgw - self.out_size) / 2 - self.crop_offset
            y_min = (imgh - self.out_size) / 2

            heat_rgb = self.image.copy()
            heat_rgb = cv2.cvtColor(heat_rgb, cv2.COLOR_BGR2RGB)

            heat_rgb[x_min:(x_min+self.out_size), y_min:(y_min+self.out_size), :] = heatmap

            self.pub_visualization.publish(self.bridge.cv2_to_imgmsg(heat_rgb))

            # Publish rviz pose
            tfh.publish_pose_as_transform(pred.pose, 'panda_link0', 'best_G', 0.5)

        print('GGCNN successful. Current Hz-rate:\t' + str(1/(rospy.get_time() - t_start)))

        # Publish results
        self.pub_ggcnn.publish(pred)


    # Callback functions
    
    def _callback_image(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg)
        
        '''
        # Publish adapted rgb image -> input for yolo
        if self.init_depth:

            image_prep = self.image.copy()
            map_nan = cv2.erode(self.depth_nan*255, np.ones((5,5),np.uint8), iterations = 3)
            image_prep[:,:,0][map_nan == 255] = 242
            image_prep[:,:,1][map_nan == 255] = 210
            image_prep[:,:,2][map_nan == 255] = 242

            self.pub_darknet.publish(self.bridge.cv2_to_imgmsg(image_prep, encoding='rgb8'))
        '''

    def _callback_depth(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg)
        self.depth_nan = np.isnan(self.depth).astype(np.uint8)
        self.init_depth = True

        # Get pose
        self.last_image_pose = tfh.current_robot_pose(self.base_frame, self.cam_frame)

        # GGCNN
        if not (self.last_image_pose == None):
            if self.init_body and self.init_hand and self.init_box:
                self._ggcnn()
            

    def _callback_bodyparts(self, msg):
        self.mask_body = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.init_body = True
        

    def _callback_egohands(self, msg):
        self.mask_hand = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.init_hand = True

    def _callback_darknet(self, msg):

        if (self.init_depth):

            depth_nan = self.depth.copy().astype(np.float32)
            depth_nan[self.depth == 0] = np.nan
            depth_nan[self.depth >= self.dist_ignore] = np.nan
            depth_nan[self.mask_body != 0] = np.nan
            depth_nan[self.mask_hand != 0] = np.nan

            box_list = msg.bounding_boxes
            boxes = []
            for i in range(len(box_list)):
                if(box_list[i].Class != 'person'):
                    depth_mean = np.nanmean(depth_nan[box_list[i].ymin : box_list[i].ymax, box_list[i].xmin : box_list[i].xmax])
                    if(depth_mean > self.dist_ignore) or np.isnan(depth_mean):
                        continue
                    else:
                        boxes.append([box_list[i].xmin, box_list[i].ymin, box_list[i].xmax, box_list[i].ymax, depth_mean])

            if(len(boxes) != 0):
                boxes = np.asarray(boxes)
                self.best_box = boxes[np.argmin(boxes[:,4]),:]
                self.init_box = True

            else: 
                self.init_box = False

        

if __name__ == '__main__':

    rospy.init_node('ggcnn_humanseg_publisher')
    ggcnn_obj = GGCNN_humanseg()
    rospy.spin()