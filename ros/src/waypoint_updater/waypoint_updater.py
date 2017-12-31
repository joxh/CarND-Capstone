#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, TransformStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray

import copy
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

        self.set_speed = 11.1

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

        self.Upcoming_red_light_idx = None
        self.stopping = False
        self.already_decreased_speed = False


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


        # rospy.loginfo('Current pose is (%s, %s, %s), and got_base_waypoints is %s',
        #               msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, self.got_base_waypoints)


    def waypoints_cb(self, waypoints):
        # TODO: Implement
        rospy.loginfo('Got the base waypoints!')
        self.base_waypoints_lane = waypoints
        self.got_base_waypoints = True
        self.num_waypoints = len(self.base_waypoints_lane.waypoints)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        if msg.data != -1:
            # rospy.loginfo('Red light coming up @ idx=%s', msg.data)
            self.Upcoming_red_light_idx = msg.data#(msg.data-5)%self.num_waypoints
        else:
            # rospy.loginfo('No red light coming up')
            self.Upcoming_red_light_idx = None



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

        # rospy.loginfo('Theta= %s', theta * 180 / math.pi)


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


        # jkpld: Smooth slow down for red lights and acceleration on green
        # lights.
        # very simple approach. when a red light is comming up, decrease the
        # waypoints' speed slowly to 0 at the stop line. when the light goes
        # green, slowly incrase the waypoints' speeds back up to the set speed
        # NOTE : I need the parameter `velocity` from waypoint_loader, but I do
        # not know how to get that in this program, so I hard coded it above in
        # with the property set_speed.
        current_speed = self.get_waypoint_velocity(waypoints_to_return[0])

        # rate of deceleration will be 0.5*v_ref^2/s_ref
        v_ref = 10;
        s_ref = 20;

        if self.Upcoming_red_light_idx:
            rospy.loginfo('Upcoming red light in : {:0.2f}m'.format(self.distance(self.base_waypoints_lane.waypoints, next_waypoint_ind, self.Upcoming_red_light_idx)))
            # move 1 waypoint farther from light to give larger buffer zone.
            self.Upcoming_red_light_idx = (self.Upcoming_red_light_idx - 1)%self.num_waypoints
            # Distance until red light
            dist = self.distance(self.base_waypoints_lane.waypoints, next_waypoint_ind, self.Upcoming_red_light_idx)

            if not self.stopping:
                if dist < 5:
                    self.stopping = True
                else:
                    self.stopping = current_speed*current_speed/dist > v_ref*v_ref/s_ref
        else:
            self.stopping = False

        if (self.Upcoming_red_light_idx and
            self.Upcoming_red_light_idx < (next_waypoint_ind+LOOKAHEAD_WPS) and
            self.stopping) :

            if not self.already_decreased_speed:
                # there is a red light coming up in range of our lookahead, so
                # start slowing down the car to have a speed of 0 at and after
                # waypoint Upcoming_red_light_idx

                final_speed = 0 # goal speed
                ds = current_speed - final_speed # speed difference

                # approach_fun : use linear for linearly decreasing speed or
                # math.sqrt for a faster slow down initially and a slow approach to
                # approach_fun = lambda x : math.sqrt(x)
                approach_fun = lambda x : x

                dx = 0
                for i in range(1,LOOKAHEAD_WPS):

                    if i < self.Upcoming_red_light_idx - next_waypoint_ind+1:
                        # Distance traveled
                        wp0 = waypoints_to_return[i-1].pose.pose.position
                        wp1 = waypoints_to_return[i].pose.pose.position
                        dxi = wp0.x - wp1.x
                        dyi = wp0.y - wp1.y
                        dist_lateral = math.sqrt(dxi**2 + dyi**2)
                        dx += dist_lateral

                        speed_i = max(round(10*(current_speed - approach_fun(dx/dist)*ds))/10, 0)
                        rospy.loginfo('dist = {}, speed_diff = {}, dx = {}, new_speed = {}'.format(dist, ds, dx, speed_i))
                        self.set_waypoint_velocity(waypoints_to_return, i, speed_i)
                    else:
                        self.set_waypoint_velocity(waypoints_to_return, i, final_speed)

                self.already_decreased_speed = True
        else:
            self.already_decreased_speed = False
            # If the current speed is more than 40% different from the set speed
            # then create a linearly chaning set speed for the waypoints.
            # If the speed is within 40% of the set speed, then let the
            # controller handle it.
            if abs(current_speed - self.set_speed)/self.set_speed > 0.4:
                dist = self.distance(waypoints_to_return,0,LOOKAHEAD_WPS-2)
                if dist > 0:
                    accel_rate = (current_speed - self.set_speed)/dist
                else:
                    accel_rate = 0

                dx = 0
                for i in range(0,LOOKAHEAD_WPS):
                    # rospy.loginfo('Setting car speed to {}'.format(11.1))
                    wp0 = waypoints_to_return[i-1].pose.pose.position
                    wp1 = waypoints_to_return[i].pose.pose.position
                    dxi = wp0.x - wp1.x
                    dyi = wp0.y - wp1.y
                    dist_lateral = math.sqrt(dxi**2 + dyi**2)
                    dx += dist_lateral
                    self.set_waypoint_velocity(waypoints_to_return, i, current_speed - dx*accel_rate)
            else:
                for i in range(0,LOOKAHEAD_WPS):
                    self.set_waypoint_velocity(waypoints_to_return, i, self.set_speed)

        # rospy.loginfo('Current waypoint set speed : {:0.2f}m'.format(self.get_waypoint_velocity(waypoints_to_return[0])))
        # rospy.loginfo('Current waypoint set speed (from base): {:0.2f}m'.format(self.get_waypoint_velocity(self.base_waypoints_lane.waypoints[next_waypoint_ind])))



        lane_to_publish = Lane()
        lane_to_publish.header = self.current_pose.header
        lane_to_publish.waypoints = waypoints_to_return
        self.final_waypoints_pub.publish(lane_to_publish)

        # rospy.loginfo('Next waypoint is %s',
        #               waypoints_to_return[0].pose)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
