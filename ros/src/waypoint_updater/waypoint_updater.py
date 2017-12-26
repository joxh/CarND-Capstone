#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, TransformStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):

        # TODO: Add other member variables you need below
        self.current_pose = None
        self.base_waypoints_lane = None
        self.got_base_waypoints = False
        self.current_waypoint_ind = None

        rospy.init_node('waypoint_updater')

        self.sub_current_pose = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.sub_base_waypoints = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        
        # Add a subscriber for /traffic_waypoint
        self.sub_traffic_waypoint = rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # TODO: Add a subscriber for /obstacle_waypoint below
        #self.sub_obstacle_waypoint = rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)



        #SIM ONLY: TrafficLightArray
        self.sub_traffic_lights_gt = rospy.Subscriber('/traffic_waypoint', TrafficLightArray, self.traffic_lights_gt_cb)
        

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pose = msg

        if self.got_base_waypoints and msg.header.seq % 10 == 0:
            if msg.header.seq % 100 == 0:
                self.current_waypoint_ind = None
            self.publish_next_waypoints()
        else:
            pass
            

        rospy.loginfo('Current pose is (%s, %s, %s), and got_base_waypoints is %s', 
                      msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, self.got_base_waypoints)

        


        

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        rospy.loginfo('Got the base waypoints!')
        self.base_waypoints_lane = waypoints
        self.got_base_waypoints = True
        self.num_waypoints = len(self.base_waypoints_lane.waypoints)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        rospy.loginfo('Current /traffic_waypoint is %s', msg.data)

        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def traffic_lights_gt_cb(self, msg):
        # TODO: Callback for dealing with traffic light ground truth
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def publish_next_waypoints(self):
        
        next_waypoint_ind = None
        min_dist_lateral = None
        current_pose = self.current_pose

        theta = 2 * math.atan2(current_pose.pose.orientation.z, 
                            current_pose.pose.orientation.w)

        rospy.loginfo('Theta= %s', theta * 180 / math.pi)


        yhat = math.sin(theta)
        xhat = math.cos(theta)
        if self.current_waypoint_ind is None:
            ind_to_check = range(self.num_waypoints)
        else:
            ind_to_check = [(i + self.current_waypoint_ind)%self.num_waypoints for i in range(-20, 20)]

        for i in ind_to_check:
            wp = self.base_waypoints_lane.waypoints[i]
            #Check each waypoint's
            dx = wp.pose.pose.position.x - current_pose.pose.position.x
            dy = wp.pose.pose.position.y - current_pose.pose.position.y
            dist_lateral = math.sqrt(dx**2 + dy**2)

            in_front_dist = 3 #meters
            in_front = xhat*dx + yhat*dy > in_front_dist #Two meters ahead

            if in_front:
                if next_waypoint_ind is not None:
                    if dist_lateral < min_dist_lateral:
                        next_waypoint_ind = i
                        min_dist_lateral = dist_lateral 
                else:
                    next_waypoint_ind = i
                    min_dist_lateral = dist_lateral
        
        if next_waypoint_ind is None:
            next_waypoint_ind = 0
            self.current_waypoint_ind = None
        else:
            self.current_waypoint_ind = next_waypoint_ind
        
        #Check if need to read forward or backwards: Not implemented
        

        waypoints_ind_to_return = [(i + next_waypoint_ind)%self.num_waypoints for i in range(LOOKAHEAD_WPS)]
        waypoints_to_return = [self.base_waypoints_lane.waypoints[i] for i in waypoints_ind_to_return]
        
        lane_to_publish = Lane()
        lane_to_publish.header = self.current_pose.header
        lane_to_publish.waypoints = waypoints_to_return
        self.final_waypoints_pub.publish(lane_to_publish)

        rospy.loginfo('Next waypoint is %s', 
                      waypoints_to_return[0].pose)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
