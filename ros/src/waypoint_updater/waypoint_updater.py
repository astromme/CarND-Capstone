#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        # self.traffic_waypoint_sub = rospy.Subscriber('traffic_waypoint',
        #                              Waypoint, self.traffic_waypoint_callback)
        #
        # self.obstacle_waypoint_sub = rospy.Subscriber('obstacle_waypoint',
        #                              Waypoint, self.obstacle_waypoint_callback)

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.pose = None
        self.waypoints = None
        self.next_waypoint = None

        rospy.spin()

    def pose_cb(self, msg):
        # rospy.loginfo(msg)
        self.pose = msg

        # if self.waypoints:
        #     next_waypoint = self.nextWaypointId()
        #     if next_waypoint != self.next_waypoint:
        #         self.next_waypoint = next_waypoint
        #
        #         lane = Lane()
        #         lane.header.frame_id = '/world'
        #         lane.header.stamp = rospy.Time(0)
        #         wrapped_waypoints = self.waypoints[next_waypoint:] + self.waypoints[:next_waypoint]
        #         lane.waypoints = wrapped_waypoints[:LOOKAHEAD_WPS]
        #         self.final_waypoints_pub.publish(lane)

    def waypoints_cb(self, waypoints):
        rospy.loginfo('got {} base waypoints'.format(len(waypoints.waypoints)))
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def closestWaypointId(self):
        closest_waypoint = 0
        closest_distance = self.distance2(self.waypoints[closest_waypoint].pose.pose.position, self.pose.pose.position)
        for i in range(1, len(self.waypoints)):
            d = self.distance2(self.waypoints[closest_waypoint].pose.pose.position, self.pose.pose.position)
            if d < closest_distance:
                closest_distance = d
                closest_waypoint = i

        return closest_waypoint

    def distance2(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    def euler_from_quaternion(self, quaternion):
        explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        return tf.transformations.euler_from_quaternion(explicit_quat)

    def nextWaypointId(self):
        closestWaypointId = self.closestWaypointId()
        closestWaypoint = self.waypoints[closestWaypointId]
        heading = math.atan2((closestWaypoint.pose.pose.position.x - self.pose.pose.position.x),
                             (closestWaypoint.pose.pose.position.y - self.pose.pose.position.y))

        yaw = self.euler_from_quaternion(self.pose.pose.orientation)[2]

        angle = yaw - heading
        if angle > math.pi / 4.0:
            closestWaypointId += 1

        return closestWaypointId;

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
