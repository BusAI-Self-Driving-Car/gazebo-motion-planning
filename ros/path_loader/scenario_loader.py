#!/usr/bin/env python

import os
import csv
import math

from geometry_msgs.msg import Quaternion, Point32, PoseStamped, Pose
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Path

import tf
import rospy

CSV_HEADER = ['x', 'y', 's', 'dx', 'dy']
MAX_DECEL = 1.0

class ScenarioLoader(object):

    def __init__(self):

        rospy.init_node('scenario_loader', log_level=rospy.DEBUG)

        self.frame_id = rospy.get_param('~frame_id')

        self.lane_line_yellow_points_pub = rospy.Publisher('/lane_line_yellow_points', PointCloud, queue_size=1, latch=True)
        self.lane_line_white_points_pub = rospy.Publisher('/lane_line_white_points', PointCloud, queue_size=1, latch=True)

        self.path_pub = rospy.Publisher('/base_path', Path, queue_size=1, latch=True)

        file1_name = rospy.get_param('~fpath1')
        file2_name = rospy.get_param('~fpath2')
        file3_name = rospy.get_param('~fpath3')
        self.new_lane_lines_loader(file1_name, file2_name, file3_name)

        rospy.spin()

    def new_lane_lines_loader(self, fpath1, fpath2, fpath3):
        if os.path.isfile(fpath1) and os.path.isfile(fpath2):
            yellow_points = self.load_path(fpath1)
            white_points = self.load_path(fpath2)
            target_path = self.target_path(fpath3)
            self.publish(yellow_points, white_points, target_path)
            rospy.loginfo('Lane Lines and Target Path loaded')
        else:
            rospy.logerr('%s, %s or %s is not a file', fpath1, fpath2, fpath3)

    def load_path(self, fname):
        points = []
        with open(fname) as wfile:
            reader = csv.DictReader(wfile, delimiter=' ', fieldnames=CSV_HEADER)
            for index, wp in enumerate(reader):
                point = Point32()
                point.x = float(wp['x'])
                point.y = float(wp['y'])
                point.z = 0.0
                points.append(point)

        return points

    def target_path(self, fname):
        poses = []
        points = []
        with open(fname) as wfile:
            reader = csv.DictReader(wfile, delimiter=' ', fieldnames=CSV_HEADER)
            for index, wp in enumerate(reader):
                pose_stamped = PoseStamped()
                pose = Pose()

                pose.position.x = float(wp['x'])
                pose.position.y = float(wp['y'])
                pose.position.z = 0.0

                pose_stamped.header.frame_id = self.frame_id
                pose_stamped.header.seq = index
                pose_stamped.pose = pose
                poses.append(pose_stamped)

        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = rospy.Time(0)
        path.poses = poses

        return path


    def publish(self, yellow_points, white_points, target_path):
        lane_line_yellow_points = PointCloud()
        lane_line_yellow_points.header.frame_id = self.frame_id
        lane_line_yellow_points.header.stamp = rospy.Time(0)
        lane_line_yellow_points.points = yellow_points
        self.lane_line_yellow_points_pub.publish(lane_line_yellow_points)

        lane_line_white_points = PointCloud()
        lane_line_white_points.header.frame_id = self.frame_id
        lane_line_white_points.header.stamp = rospy.Time(0)
        lane_line_white_points.points = white_points
        self.lane_line_white_points_pub.publish(lane_line_white_points)

        self.path_pub.publish(target_path)

if __name__ == '__main__':
    try:
        ScenarioLoader()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start scenarioLoader loader node.')
