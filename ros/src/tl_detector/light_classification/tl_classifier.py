from styx_msgs.msg import TrafficLight
import rospy
import numpy as np
import os
import sys
import tensorflow as tf
from collections import defaultdict
from io import StringIO
from light_classification.label_map_util import *
from light_classification.visualization_utils import *
import time


class TLClassifier(object):
    def __init__(self):
        self.current_light = TrafficLight.UNKNOWN
        # set path
        cwd = os.path.dirname(os.path.realpath(__file__))
        PATH_TO_LABELS = cwd + '/final/sdc_label_map.pbtxt'
        GRAPH_FILE = cwd + '/final/frozen_inference_graph.pb'
        # only 4 classes are specified
        NUM_CLASSES = 4
        label_map = load_labelmap(PATH_TO_LABELS)
        categories = convert_label_map_to_categories(label_map,
            max_num_classes=NUM_CLASSES, use_display_name=True)
        self.category_index = create_category_index(categories)
        # initialize
        self.detection_graph = tf.Graph()
        # https://github.com/tensorflow/tensorflow/issues/6698
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        with self.detection_graph.as_default():
            graph_def = tf.GraphDef()
            with tf.gfile.GFile(GRAPH_FILE, 'rb') as fid:
                serialized_graph = fid.read()
                graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(graph_def, name='')
            self.sess = tf.Session(graph=self.detection_graph, config=config)
        # Definite input and output Tensors for detection_graph
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
        print("Loaded graph")


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        rospy.logdebug("tl_classifier: Classification requested")
        image_np_expanded = np.expand_dims(image, axis=0)
        #  detection.
        with self.detection_graph.as_default():
            (boxes, scores, classes, num) = self.sess.run(
                [self.detection_boxes, self.detection_scores,
                self.detection_classes, self.num_detections],
                feed_dict={self.image_tensor: image_np_expanded})
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)
        # set a threshold
        min_score_thresh = .50
        for i in range(boxes.shape[0]):
            if scores is None or scores[i] > min_score_thresh:
                class_name = self.category_index[classes[i]]['name']
                if class_name == 'Red':
                    self.current_light = TrafficLight.RED
                    rospy.logdebug("tl_classifier: RED light detected")
                elif class_name == 'Green':
                    self.current_light = TrafficLight.GREEN
                elif class_name == 'Yellow':
                    self.current_light = TrafficLight.YELLOW
                # get distance to traffic light
                perceived_width_x = (boxes[i][3] - boxes[i][1]) * 800
                perceived_width_y = (boxes[i][2] - boxes[i][0]) * 600
                # traffic light is 3 feet long and 1 foot wide
                fx =  1345.200806
                fy =  1353.838257
                perceived_depth_x = ((1 * fx) / perceived_width_x)
                perceived_depth_y = ((3 * fy) / perceived_width_y )
                estimated_distance = round((perceived_depth_x + perceived_depth_y) / 2)
        # Visualization of the results of a detection.
        visualize_boxes_and_labels_on_image_array(
            image, boxes, classes, scores,
            self.category_index,
            use_normalized_coordinates=True,
            line_thickness=8)
        self.image_np_deep = image
        return self.current_light
