from styx_msgs.msg import TrafficLight

import os
import sys
import tensorflow as tf
import numpy as np
from functools import partial

THRESHOLD = 0.20

class TLClassifier(object):
    def __init__(self):
        self.model_path = os.path.dirname(os.path.realpath(__file__))
        self.current_light = TrafficLight.UNKNOWN

        self.readsize = 1024
        # Re-Build the model for first time
        if not os.path.exists(self.model_path+'/faster-R-CNN/checkpoints/frozen_inference_graph.pb'):
            # if not - build it back up
            #if not os.path.exists(self.model_path+'faster-R-CNN/chunks'):
            output = open(self.model_path+'/faster-R-CNN/checkpoints/frozen_inference_graph.pb', 'wb')
            chunks = os.listdir(self.model_path+'/faster-R-CNN/chunks')
            chunks.sort()
            for filename in chunks:
                filepath = os.path.join(self.model_path+'/faster-R-CNN/chunks', filename)
                #print filepath
                with open(filepath, 'rb') as fileobj:
                    for chunk in iter(partial(fileobj.read, self.readsize), ''):
                        output.write(chunk)
            output.close()



        PATH_TO_CKPT = self.model_path+'/faster-R-CNN/checkpoints/frozen_inference_graph.pb'

        #Load a (frozen) Tensorflow model into memory. 
        config = tf.ConfigProto()
        #config.gpu_options.allow_growth = True
        config.gpu_options.per_process_gpu_memory_fraction = 0.5
        
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
          od_graph_def = tf.GraphDef()
          with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')#
            self.tf_session = tf.Session(graph=self.detection_graph, config=config)
            # Definite input and output Tensors for self.tf_graph
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image_np (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        image_expanded = np.expand_dims(image, axis=0)

        # Actual detection
        (scores, classes, num) = self.tf_session.run(
            [self.detection_scores, self.detection_classes, self.num_detections],
            feed_dict={self.image_tensor: image_expanded})

        # Visualization of the results of a detection.
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        # calculate prediction
        cc = classes[0]
        confidence = scores[0]

        self.current_light = TrafficLight.UNKNOWN
        if cc > 0 and cc < 4 and confidence is not None and confidence > THRESHOLD:
            # Traffic light 
            if cc == 1:
                self.current_light = TrafficLight.RED #0
            elif cc == 2:
                self.current_light = TrafficLight.YELLOW #1
            elif cc == 3:
                self.current_light = TrafficLight.GREEN #2
            #print self.current_light

        return self.current_light
