import os
import csv
from math import sin, cos, pi
import tf
import rospy
from geometry_msgs.msg import Quaternion, Point32, PoseStamped, Pose
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Path

ORIGIN = [0, 0]
ROAD_WIDTH = 5
ROAD_LENGTH = 115
CORNER_GAP = 2.5

# utility fnc to compute min delta angle, accounting for periodicity
def min_dang(dang):
    while dang > pi: dang -= 2.0 * pi
    while dang < -pi: dang += 2.0 * pi
    return dang

def quaternion_from_yaw(yaw):
    return tf.transformations.quaternion_from_euler(0., 0., yaw)

def generate_yellow_lines(id):
    poses = []

    # upper horizontal line 1
    for i in range(100):
        id += 1

        pose_stamped = PoseStamped()
        pose = Pose()

        pose.position.x = ORIGIN[0] + ROAD_LENGTH / 2.0 - ROAD_WIDTH * 1.5 - CORNER_GAP - i * 0.5
        pose.position.y = ORIGIN[1] + 0.0
        pose.position.z = 0.0  # let's hope so!

        yaw = 0.0
        quat = quaternion_from_yaw(yaw)
        pose.orientation = quat

        pose_stamped.header.frame_id = '/world'
        pose_stamped.header.seq = id
        pose_stamped.pose = pose

        poses.append(pose_stamped)

    # # upper horizontal line 2
    # for i in range(100):
    #     id += 1

    #     pose_stamped = PoseStamped()
    #     pose = Pose()

    #     pose.position.x = ORIGIN[0] + ROAD_LENGTH / 2 + ROAD_WIDTH * 1.5 + CORNER_GAP + i * 0.5
    #     pose.position.y = ORIGIN[1] + 0.0
    #     pose.position.z = 0.0  # let's hope so!

    #     yaw = 0.0
    #     quat = quaternion_from_yaw(yaw)
    #     pose.orientation = quat

    #     pose_stamped.header.frame_id = '/world'
    #     pose_stamped.header.seq = id
    #     pose_stamped.pose = pose

    #     poses.append(pose_stamped)

    # lower horizontal line 1
    for i in range(100):
        id += 1

        pose_stamped = PoseStamped()
        pose = Pose()

        pose.position.x = ORIGIN[0] + ROAD_LENGTH / 2.0 - ROAD_WIDTH * 1.5 - CORNER_GAP - i * 0.5
        pose.position.y = ORIGIN[1] - ROAD_WIDTH * 3.0
        pose.position.z = 0.0  # let's hope so!

        yaw = 0.0
        quat = quaternion_from_yaw(yaw)
        pose.orientation = quat

        pose_stamped.header.frame_id = '/world'
        pose_stamped.header.seq = id
        pose_stamped.pose = pose

        poses.append(pose_stamped)

    # # lower horizontal line 2
    # for i in range(100):
    #     id += 1

    #     pose_stamped = PoseStamped()
    #     pose = Pose()

    #     pose.position.x = ORIGIN[0] + ROAD_LENGTH / 2 + ROAD_WIDTH * 1.5 + CORNER_GAP + i * 0.5
    #     pose.position.y = ORIGIN[1] - ROAD_WIDTH * 3
    #     pose.position.z = 0.0  # let's hope so!

    #     yaw = 0.0
    #     quat = quaternion_from_yaw(yaw)
    #     pose.orientation = quat

    #     pose_stamped.header.frame_id = '/world'
    #     pose_stamped.header.seq = id
    #     pose_stamped.pose = pose

    #     poses.append(pose_stamped)

    # round corner 1
    phi0 = 0.0
    radius = CORNER_GAP
    n_points = int(pi / 2 * radius / 0.5)
    for i in range(1, n_points+1):
        id += 1

        pose_stamped = PoseStamped()
        pose = Pose()

        phi = min_dang(phi0 + 0.5 / radius * i)
        pose.position.x = ORIGIN[0] + ROAD_LENGTH / 2.0 - ROAD_WIDTH * 1.5 - CORNER_GAP + radius * cos(phi)
        pose.position.y = ORIGIN[1] - ROAD_WIDTH * 3.0 - CORNER_GAP + radius * sin(phi)
        pose.position.z = 0.0  # let's hope so!

        yaw = min_dang(phi)
        quat = quaternion_from_yaw(yaw)
        pose.orientation = quat

        pose_stamped.header.frame_id = '/world'
        pose_stamped.header.seq = id
        pose_stamped.pose = pose

        poses.append(pose_stamped)

    # # round corner 2
    # phi0 = pi / 2
    # radius = CORNER_GAP
    # n_points = int(pi / 2 * radius / 0.5)
    # for i in range(1, n_points+1):
    #     id += 1

    #     pose_stamped = PoseStamped()
    #     pose = Pose()

    #     phi = min_dang(phi0 + 0.5 / radius * i)
    #     pose.position.x = ORIGIN[0] + ROAD_LENGTH / 2 + ROAD_WIDTH * 1.5 + CORNER_GAP + radius * cos(phi)
    #     pose.position.y = ORIGIN[1] - ROAD_WIDTH * 3 - CORNER_GAP + radius * sin(phi)
    #     pose.position.z = 0.0  # let's hope so!

    #     yaw = min_dang(phi)
    #     quat = quaternion_from_yaw(yaw)
    #     pose.orientation = quat

    #     pose_stamped.header.frame_id = '/world'
    #     pose_stamped.header.seq = id
    #     pose_stamped.pose = pose

    #     poses.append(pose_stamped)

    # left vertical line
    for i in range(50):
        id += 1

        pose_stamped = PoseStamped()
        pose = Pose()

        pose.position.x = ORIGIN[0]  + ROAD_LENGTH / 2.0 - ROAD_WIDTH * 1.5
        pose.position.y = ORIGIN[1] - ROAD_WIDTH * 3.0 - CORNER_GAP - i * 0.5
        pose.position.z = 0.0  # let's hope so!

        yaw = 0.0
        quat = quaternion_from_yaw(yaw)
        pose.orientation = quat

        pose_stamped.header.frame_id = '/world'
        pose_stamped.header.seq = id
        pose_stamped.pose = pose

        poses.append(pose_stamped)

    # right vertical line
    for i in range(50):
        id += 1

        pose_stamped = PoseStamped()
        pose = Pose()

        pose.position.x = ORIGIN[0] + ROAD_LENGTH / 2.0 + ROAD_WIDTH * 1.5
        pose.position.y = ORIGIN[1] - ROAD_WIDTH * 3.0 - CORNER_GAP- i * 0.5
        pose.position.z = 0.0  # let's hope so!

        yaw = 0.0
        quat = quaternion_from_yaw(yaw)
        pose.orientation = quat

        pose_stamped.header.frame_id = '/world'
        pose_stamped.header.seq = id
        pose_stamped.pose = pose

        poses.append(pose_stamped)

    return poses

def generate_white_lines(id):
    poses = []

    # horizontal line 1
    for i in range(100):
        id += 1

        pose_stamped = PoseStamped()
        pose = Pose()

        pose.position.x = ORIGIN[0] + ROAD_LENGTH / 2.0 - ROAD_WIDTH * 1.5 - CORNER_GAP - i * 0.5
        pose.position.y = ORIGIN[1] - ROAD_WIDTH
        pose.position.z = 0.0  # let's hope so!

        yaw = 0.0
        quat = quaternion_from_yaw(yaw)
        pose.orientation = quat

        pose_stamped.header.frame_id = '/world'
        pose_stamped.header.seq = id
        pose_stamped.pose = pose

        poses.append(pose_stamped)

    # # horizontal line 2
    # for i in range(100):
    #     id += 1

    #     pose_stamped = PoseStamped()
    #     pose = Pose()

    #     pose.position.x = ORIGIN[0] + ROAD_LENGTH / 2.0 + ROAD_WIDTH * 1.5 + CORNER_GAP + i * 0.5
    #     pose.position.y = ORIGIN[1] - ROAD_WIDTH
    #     pose.position.z = 0.0  # let's hope so!

    #     yaw = 0.0
    #     quat = quaternion_from_yaw(yaw)
    #     pose.orientation = quat

    #     pose_stamped.header.frame_id = '/world'
    #     pose_stamped.header.seq = id
    #     pose_stamped.pose = pose

    #     poses.append(pose_stamped)

    # horizontal line 3
    for i in range(100):
        id += 1

        pose_stamped = PoseStamped()
        pose = Pose()

        pose.position.x = ORIGIN[0] + ROAD_LENGTH / 2.0 - ROAD_WIDTH * 1.5 - CORNER_GAP - i * 0.5
        pose.position.y = ORIGIN[1] - ROAD_WIDTH * 2.0
        pose.position.z = 0.0  # let's hope so!

        yaw = 0.0
        quat = quaternion_from_yaw(yaw)
        pose.orientation = quat

        pose_stamped.header.frame_id = '/world'
        pose_stamped.header.seq = id
        pose_stamped.pose = pose

        poses.append(pose_stamped)

    # # horizontal line 4
    # for i in range(100):
    #     id += 1

    #     pose_stamped = PoseStamped()
    #     pose = Pose()

    #     pose.position.x = ORIGIN[0] + ROAD_LENGTH / 2 + ROAD_WIDTH * 1.5 + CORNER_GAP + i * 0.5
    #     pose.position.y = ORIGIN[1] - ROAD_WIDTH * 2
    #     pose.position.z = 0.0  # let's hope so!

    #     yaw = 0.0
    #     quat = quaternion_from_yaw(yaw)
    #     pose.orientation = quat

    #     pose_stamped.header.frame_id = '/world'
    #     pose_stamped.header.seq = id
    #     pose_stamped.pose = pose

    #     poses.append(pose_stamped)

    # left vertical line
    for i in range(50):
        id += 1

        pose_stamped = PoseStamped()
        pose = Pose()

        pose.position.x = ORIGIN[0] + ROAD_LENGTH / 2.0 - ROAD_WIDTH * 0.5
        pose.position.y = ORIGIN[1] - ROAD_WIDTH * 3.0 - CORNER_GAP - i * 0.5
        pose.position.z = 0.0  # let's hope so!

        yaw = 0.0
        quat = quaternion_from_yaw(yaw)
        pose.orientation = quat

        pose_stamped.header.frame_id = '/world'
        pose_stamped.header.seq = id
        pose_stamped.pose = pose

        poses.append(pose_stamped)

    # right vertical line
    for i in range(50):
        id += 1

        pose_stamped = PoseStamped()
        pose = Pose()

        pose.position.x = ORIGIN[0] + ROAD_LENGTH / 2.0 + ROAD_WIDTH * 0.5
        pose.position.y = ORIGIN[1] - ROAD_WIDTH * 3.0 - CORNER_GAP - i * 0.5
        pose.position.z = 0.0  # let's hope so!

        yaw = 0.0
        quat = quaternion_from_yaw(yaw)
        pose.orientation = quat

        pose_stamped.header.frame_id = '/world'
        pose_stamped.header.seq = id
        pose_stamped.pose = pose

        poses.append(pose_stamped)

    poses += generate_crosswalk([ORIGIN[0] + ROAD_LENGTH / 2.0 - ROAD_WIDTH * 1.5 - CORNER_GAP - 5, ORIGIN[1]], id)
    poses += generate_crosswalk_hori([ORIGIN[0] + ROAD_LENGTH / 2.0 - ROAD_WIDTH * 1.5, ORIGIN[1] - ROAD_WIDTH * 3 - CORNER_GAP], id)

    return poses

def generate_crosswalk(topleft, id):
    poses = []

    # left vertical line
    for i in range(30):
        id += 1

        pose_stamped = PoseStamped()
        pose = Pose()

        pose.position.x = topleft[0]
        pose.position.y = topleft[1] - i * 0.5
        pose.position.z = 0.0  # let's hope so!

        yaw = 0.0
        quat = quaternion_from_yaw(yaw)
        pose.orientation = quat

        pose_stamped.header.frame_id = '/world'
        pose_stamped.header.seq = id
        pose_stamped.pose = pose

        poses.append(pose_stamped)

    # right vertical line
    for i in range(30):
        id += 1

        pose_stamped = PoseStamped()
        pose = Pose()

        pose.position.x = topleft[0] + 5
        pose.position.y = topleft[1] - i * 0.5
        pose.position.z = 0.0  # let's hope so!

        yaw = 0.0
        quat = quaternion_from_yaw(yaw)
        pose.orientation = quat

        pose_stamped.header.frame_id = '/world'
        pose_stamped.header.seq = id
        pose_stamped.pose = pose

        poses.append(pose_stamped)

    for k in range(1, 15):
        # short horizontal line
        for i in range(10):
            id += 1

            pose_stamped = PoseStamped()
            pose = Pose()

            pose.position.x = topleft[0] + i * 0.5
            pose.position.y = topleft[1] - 1.0 * k
            pose.position.z = 0.0  # let's hope so!

            yaw = 0.0
            quat = quaternion_from_yaw(yaw)
            pose.orientation = quat

            pose_stamped.header.frame_id = '/world'
            pose_stamped.header.seq = id
            pose_stamped.pose = pose

            poses.append(pose_stamped)

    return poses

def generate_crosswalk_hori(topleft, id):
    poses = []

    # upper horizontal line
    for i in range(30):
        id += 1

        pose_stamped = PoseStamped()
        pose = Pose()

        pose.position.x = topleft[0] + i * 0.5
        pose.position.y = topleft[1]
        pose.position.z = 0.0  # let's hope so!

        yaw = 0.0
        quat = quaternion_from_yaw(yaw)
        pose.orientation = quat

        pose_stamped.header.frame_id = '/world'
        pose_stamped.header.seq = id
        pose_stamped.pose = pose

        poses.append(pose_stamped)

    # lower horizontal line
    for i in range(30):
        id += 1

        pose_stamped = PoseStamped()
        pose = Pose()

        pose.position.x = topleft[0] + i * 0.5
        pose.position.y = topleft[1] - 5
        pose.position.z = 0.0  # let's hope so!

        yaw = 0.0
        quat = quaternion_from_yaw(yaw)
        pose.orientation = quat

        pose_stamped.header.frame_id = '/world'
        pose_stamped.header.seq = id
        pose_stamped.pose = pose

        poses.append(pose_stamped)

    for k in range(1, 15):
        # short vertical line
        for i in range(10):
            id += 1

            pose_stamped = PoseStamped()
            pose = Pose()

            pose.position.x = topleft[0] + 1.0 * k
            pose.position.y = topleft[1] -0.5 * i
            pose.position.z = 0.0  # let's hope so!

            yaw = 0.0
            quat = quaternion_from_yaw(yaw)
            pose.orientation = quat

            pose_stamped.header.frame_id = '/world'
            pose_stamped.header.seq = id
            pose_stamped.pose = pose

            poses.append(pose_stamped)

    return poses

def generate_target_path(id):
    poses = []

    # before turning
    for i in range(50):
        id += 1

        pose_stamped = PoseStamped()
        pose = Pose()

        pose.position.x = ORIGIN[0] + ROAD_LENGTH / 2.0 - ROAD_WIDTH
        pose.position.y = ORIGIN[1] - ROAD_WIDTH * 3.0 - CORNER_GAP - 25 + i * 0.5
        pose.position.z = 0.0  # let's hope so!

        yaw = 0.0
        quat = quaternion_from_yaw(yaw)
        pose.orientation = quat

        pose_stamped.header.frame_id = '/world'
        pose_stamped.header.seq = id
        pose_stamped.pose = pose

        poses.append(pose_stamped)

    # on turning
    phi0 = 0.0
    radius = CORNER_GAP + ROAD_WIDTH / 2.0
    n_points = int(pi / 2 * radius / 0.5)
    for i in range(1, n_points+1):
        id += 1

        pose_stamped = PoseStamped()
        pose = Pose()

        phi = min_dang(phi0 + 0.5 / radius * i)
        pose.position.x = ORIGIN[0] + ROAD_LENGTH / 2.0 - ROAD_WIDTH * 1.5 - CORNER_GAP + radius * cos(phi)
        pose.position.y = ORIGIN[1] - ROAD_WIDTH * 3.0 - CORNER_GAP + radius * sin(phi)
        pose.position.z = 0.0  # let's hope so!

        yaw = min_dang(phi)
        quat = quaternion_from_yaw(yaw)
        pose.orientation = quat

        pose_stamped.header.frame_id = '/world'
        pose_stamped.header.seq = id
        pose_stamped.pose = pose

        poses.append(pose_stamped)

    # after turning
    for i in range(100):
        id += 1

        pose_stamped = PoseStamped()
        pose = Pose()

        pose.position.x = ORIGIN[0] + ROAD_LENGTH / 2.0 - ROAD_WIDTH * 1.5 - CORNER_GAP - i * 0.5
        pose.position.y = ORIGIN[1] - ROAD_WIDTH * 5.0 / 2.0
        pose.position.z = 0.0  # let's hope so!

        yaw = 0.0
        quat = quaternion_from_yaw(yaw)
        pose.orientation = quat

        pose_stamped.header.frame_id = '/world'
        pose_stamped.header.seq = id
        pose_stamped.pose = pose

        poses.append(pose_stamped)

    return poses

def write_to_csv(poses, fname):
    with open(fname, 'w') as wfile:
        writer = csv.writer(wfile, delimiter=' ')
        for pose in poses:
            writer.writerow([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 0])

def main():
    id = 0

    poses = generate_yellow_lines(id)
    fname = 'scenario2_yellow.csv'
    write_to_csv(poses, fname)

    poses = generate_white_lines(id)
    fname = 'scenario2_white.csv'
    write_to_csv(poses, fname)

    poses = generate_target_path(0)
    fname = 'scenario2_target_path.csv'
    write_to_csv(poses, fname)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start the node.')
