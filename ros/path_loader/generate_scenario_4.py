import os
import csv
from math import sin, cos, pi
import tf
import rospy
from geometry_msgs.msg import Quaternion, Point32, PoseStamped, Pose
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Path

# utility fnc to compute min delta angle, accounting for periodicity
def min_dang(dang):
    while dang > pi: dang -= 2.0 * pi
    while dang < -pi: dang += 2.0 * pi
    return dang

def quaternion_from_yaw(yaw):
    return tf.transformations.quaternion_from_euler(0., 0., yaw)

def generate_lane_lines():
    poses = []

    # upper horizontal line
    for i in range(230):
        pose_stamped = PoseStamped()
        pose = Pose()

        pose.position.x = i * 0.5
        pose.position.y = 0.0
        pose.position.z = 0.0  # let's hope so!

        yaw = 0.0
        quat = quaternion_from_yaw(yaw)
        pose.orientation = quat

        pose_stamped.header.frame_id = '/world'
        pose_stamped.header.seq = i
        pose_stamped.pose = pose

        poses.append(pose_stamped)

    # bottom line
    for i in range(230):
        pose_stamped = PoseStamped()
        pose = Pose()

        pose.position.x = i * 0.5
        pose.position.y = -10.0
        pose.position.z = 0.0  # let's hope so!

        yaw = 0.0
        quat = quaternion_from_yaw(yaw)
        pose.orientation = quat

        pose_stamped.header.frame_id = '/world'
        pose_stamped.header.seq = 230 + i
        pose_stamped.pose = pose

        poses.append(pose_stamped)

    return poses

def generate_white_lines():
    poses = []

    # middle horizontal line
    for i in range(230):
        pose_stamped = PoseStamped()
        pose = Pose()

        pose.position.x = i * 0.5
        pose.position.y = -5.0
        pose.position.z = 0.0  # let's hope so!

        yaw = 0.0
        quat = quaternion_from_yaw(yaw)
        pose.orientation = quat

        pose_stamped.header.frame_id = '/world'
        pose_stamped.header.seq = i
        pose_stamped.pose = pose

        poses.append(pose_stamped)

    # left vertical line
    for i in range(20):
        pose_stamped = PoseStamped()
        pose = Pose()

        pose.position.x = 50
        pose.position.y = - i * 0.5
        pose.position.z = 0.0  # let's hope so!

        yaw = 0.0
        quat = quaternion_from_yaw(yaw)
        pose.orientation = quat

        pose_stamped.header.frame_id = '/world'
        pose_stamped.header.seq = 230 + i
        pose_stamped.pose = pose

        poses.append(pose_stamped)

    # right vertical line
    for i in range(20):
        pose_stamped = PoseStamped()
        pose = Pose()

        pose.position.x = 55
        pose.position.y = - i * 0.5
        pose.position.z = 0.0  # let's hope so!

        yaw = 0.0
        quat = quaternion_from_yaw(yaw)
        pose.orientation = quat

        pose_stamped.header.frame_id = '/world'
        pose_stamped.header.seq = 250 + i
        pose_stamped.pose = pose

        poses.append(pose_stamped)

    for k in range(1, 10):
        # short horizontal line
        for i in range(10):
            pose_stamped = PoseStamped()
            pose = Pose()

            pose.position.x = 50 + i * 0.5
            pose.position.y = -1.0 * k
            pose.position.z = 0.0  # let's hope so!

            yaw = 0.0
            quat = quaternion_from_yaw(yaw)
            pose.orientation = quat

            pose_stamped.header.frame_id = '/world'
            pose_stamped.header.seq = 230 + i * k
            pose_stamped.pose = pose

            poses.append(pose_stamped)

    return poses
    
def write_to_csv(poses, fname):
    with open(fname, 'w') as wfile:
        writer = csv.writer(wfile, delimiter=' ')
        for pose in poses:
            writer.writerow([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 0])


def main():
    poses = generate_lane_lines()
    fname = 'scenario4_yellow.csv'
    write_to_csv(poses, fname)

    poses = generate_white_lines()
    fname = 'scenario4_white.csv'
    write_to_csv(poses, fname)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start the node.')