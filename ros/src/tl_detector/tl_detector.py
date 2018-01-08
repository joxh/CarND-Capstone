#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import os
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3
# Set OUTPUT_IMG = True to generate training data.
OUTPUT_IMG = False
MAX_LIMIT = 1000000

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1, buff_size=6*1450000)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.output_img_cnt = 0

        if OUTPUT_IMG == True:
            self.output_img_loop()
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''

        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    # Yipeng: I wrote this function to use for generating debug or training images
    # We may need two data set one for simulator one for real world:
    # https://discussions.udacity.com/t/do-we-need-to-develop-traffic-light-detector-or-is-it-supplied/411551/5?u=alanxiaoyi
    # Here is real world training set:
    # https://discussions.udacity.com/t/survey-traffic-lights/342249?u=alanxiaoyi
    def output_img_loop(self):
        # Generate one image per second
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.camera_image == None:
                continue
            if not os.path.exists("./output_imgs"):
                os.makedirs("./output_imgs")
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        # Call process_traffic_lights to get state in sync with image
        light_wp, state = self.process_traffic_lights()
        cv2.imwrite('./output_imgs/' + str(self.output_img_cnt)  + '_' + str(state) + '.png', cv_image)
        rospy.loginfo('Write to image output file' + str(os.getcwd()) + str(self.output_img_cnt))
        self.output_img_cnt = self.output_img_cnt + 1
        rate.sleep()

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        # Reused some code Joshua put in waypoint_upodater to find closeast wp:
        min_dist_lateral = MAX_LIMIT
        nearest_wp = None
        for i in range(0, len(self.waypoints.waypoints)):
            wp = self.waypoints.waypoints[i]
            #Check each waypoint's
            dx = wp.pose.pose.position.x - pose.position.x
            dy = wp.pose.pose.position.y - pose.position.y
            dist_lateral = math.sqrt(dx**2 + dy**2)

            if dist_lateral < min_dist_lateral:
                nearest_wp = i
                min_dist_lateral = dist_lateral

        return nearest_wp

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    # Yipeng: Get the corresponding traffic light for a stop line
    # Note that traffic light is from the topic /traffic_lights, and stop line is from the config file.
    # I am not sure if /traffic_lights will be availabe during test
    def get_tl_for_stop_line(self, stop_line_wp_idx):
        min_dist_lateral = MAX_LIMIT
        nearest_light_idx = None
        for i in range(0, len(self.lights)):
            #Check each waypoint's
            dx = self.lights[i].pose.pose.position.x - self.waypoints.waypoints[stop_line_wp_idx].pose.pose.position.x
            dy = self.lights[i].pose.pose.position.y - self.waypoints.waypoints[stop_line_wp_idx].pose.pose.position.y
            dist_lateral = math.sqrt(dx**2 + dy**2)
            if dist_lateral < min_dist_lateral:
                nearest_light_idx = i
                min_dist_lateral = dist_lateral
        return nearest_light_idx

    def get_car_in_plane_theta(self,car_pose):
        theta = 2 * math.atan2(car_pose.orientation.z,
                            car_pose.orientation.w)
        return theta

    # Yipeng: This function is to determine if the car can see the traffic light.
    # For now, I just use a simple distance to check visibility.
    # From what I understand from discussion board, we may need more sophisticated one,
    # like map 3d to 2d, etc.
    # Reference: https://discussions.udacity.com/t/focal-length-wrong/358568/22?u=alanxiaoyi
    def if_tl_visible(self, car_pose, light_idx):
        theta = self.get_car_in_plane_theta(car_pose)
        # Unit vector in direction of car's line of sight
        v_hat_x = math.cos(theta)
        v_hat_y = math.sin(theta)

        dx = self.lights[light_idx].pose.pose.position.x - car_pose.position.x
        dy = self.lights[light_idx].pose.pose.position.y - car_pose.position.y

        dist = math.sqrt(dx**2 + dy**2)

        angle_to_tl_absolute = math.atan2(dy, dx)

        angle_to_tl_relative = ((angle_to_tl_absolute - theta) + math.pi)%(2 * math.pi) - math.pi

        light_is_close = dist > 5 and dist < 100

        light_is_in_front = abs(angle_to_tl_relative) < math.pi/2

        if light_is_close and light_is_in_front:
            return True
        else:
            return False


    def process_traffic_lights(self):

        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if self.waypoints == None or self.pose == None:
            return -1, TrafficLight.UNKNOWN



        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

        # Find the closest visible traffic light (if one exists)
        nearest_stop_line_wp_idx = MAX_LIMIT
        light = None
        light_idx = None

        # Yipeng: Iterate all the stop lines from the config file to find nearest one.
        for i in range(0, len(stop_line_positions)):
            x = stop_line_positions[i][0]
            y = stop_line_positions[i][1]
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            # this is the wp index of the stop line
            stop_line_wp_idx = self.get_closest_waypoint(pose)
            # check if this stop line is the nearest one
            if stop_line_wp_idx < nearest_stop_line_wp_idx and stop_line_wp_idx > car_position:
                nearest_stop_line_wp_idx = stop_line_wp_idx
        if nearest_stop_line_wp_idx == MAX_LIMIT:
            nearest_stop_line_wp_idx = 0

        # this is the traffic light index w.r.t to this stop line
        light_idx = self.get_tl_for_stop_line(nearest_stop_line_wp_idx)
        # Check if the tl is visible to the car
        light = self.if_tl_visible(self.pose.pose, light_idx)
        # Ground truth light state, used for training only:
        state = self.lights[light_idx].state
        # Nest light waypoint
        light_wp = nearest_stop_line_wp_idx - 3 #Buffer area before stop line.

        # Print out the information. We can use this together with images for generating training set.
        #rospy.loginfo('Car_waypoint:{}, next_stopline_waypoint:{}, next_tl_index:{},tl_visibility:{}, state{}'.format(car_position,
        #                        nearest_stop_line_wp_idx, light_idx, light, state))
        # light=1 #for testing
        if light:
        # Set OUTPUT_IMG = True to generate training data
        # if not OUTPUT_IMG:
            state = self.get_light_state(light)
            #rospy.loginfo(state)
        #     return light_wp, state
            # return the ground truth stop light for testing vehicle stopping
            return light_wp, state
        #self.waypoints = None

        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
rospy.loginfo(state)
