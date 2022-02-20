#!/usr/bin/env python
import os
import cv2
import yaml
from scipy.spatial import KDTree
import numpy as np
import PIL
from six import BytesIO

import rospy
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import tensorflow as tf
from object_detection.utils import ops as utils_ops
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util


class TLDetector(object):

    def __init__(self):

        rospy.init_node('tl_detector')

        self.current_pose = None
        self.camera_image = None
        self.processed_image = None
        self.crop_needed = False

        self.bridge = CvBridge()

        self.labelmap_path = f'/home/peng/Downloads/carla_tflight/training/label_map.pbtxt'
        self.category_index = label_map_util.create_category_index_from_labelmap(self.labelmap_path, use_display_name=True)

        tf.keras.backend.clear_session()
        self.model = tf.saved_model.load(f'/home/peng/Downloads/carla_tflight/inference_graph/saved_model')

        self.processed_image_pub = rospy.Publisher('/processed_image', Image, queue_size=1)
        self.traffic_light_state_pub = rospy.Publisher('/traffic_light_state', Int32, queue_size=1)

        current_pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_cb)
        camera_image_sub = rospy.Subscriber('/image_color', Image, self.camera_image_cb)
        multi_lane_flag_sub = rospy.Subscriber('/multi_lane_flag', Bool, self.multi_lane_flag_cb)

        rospy.spin()

    def current_pose_cb(self, msg):
        # rospy.loginfo("current pose")
        self.current_pose = msg

    def camera_image_cb(self, msg):
        # rospy.loginfo("image cb")

        self.camera_image = msg

        # Convert your ROS Image message to OpenCV2
        image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        if self.crop_needed:
            # Cropping an image
            image = image[100:280, 350:-350]

        state = self.get_light_state(self.model, image)

        if self.processed_image_msg and len(self.processed_image_msg.data) > 0:
            self.processed_image_pub.publish(self.processed_image_msg)
            self.traffic_light_state_pub.publish(Int32(state))

    def multi_lane_flag_cb(self, msg):
        # rospy.loginfo("multi lane flag cb")
        if msg.data:
            self.crop_needed = msg.data

    def get_light_state(self, model, image):

        image = np.asarray(image)
        # The input needs to be a tensor, convert it using `tf.convert_to_tensor`.
        input_tensor = tf.convert_to_tensor(image)
        # The model expects a batch of images, so add an axis with `tf.newaxis`.
        input_tensor = input_tensor[tf.newaxis,...]

        # Run inference
        model_fn = model.signatures['serving_default']
        output_dict = model_fn(input_tensor)

        # All outputs are batches tensors.
        # Convert to numpy arrays, and take index [0] to remove the batch dimension.
        # We're only interested in the first num_detections.
        num_detections = int(output_dict.pop('num_detections'))
        output_dict = {key:value[0, :num_detections].numpy()
                       for key,value in output_dict.items()}
        output_dict['num_detections'] = num_detections

        # detection_classes should be ints.
        output_dict['detection_classes'] = output_dict['detection_classes'].astype(np.int64)

        vis_util.visualize_boxes_and_labels_on_image_array(
            image,
            output_dict['detection_boxes'],
            output_dict['detection_classes'],
            output_dict['detection_scores'],
            self.category_index,
            instance_masks=output_dict.get('detection_masks_reframed', None),
            use_normalized_coordinates=True,
            line_thickness=8)

        self.processed_image_msg = self.bridge.cv2_to_imgmsg(image, encoding="rgb8")

        state = 0

        if num_detections > 0 and output_dict['detection_scores'][0] > 0.5:
            state = output_dict['detection_classes'][0]

        return state


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
