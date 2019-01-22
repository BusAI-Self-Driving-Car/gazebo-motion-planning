import rospy

class Obstacle(Object):
  
  def __init__(self):
        rospy.init_node('Obstacle', log_level=rospy.DEBUG)

        self.frame_id = rospy.get_param('~frame_id')

        self.obstacle_pub = rospy.Publisher('/base_path', Path, queue_size=1, latch=True)
        self.path_points_pub = rospy.Publisher('/base_path_points', PointCloud, queue_size=1, latch=True)

        self.lane_line0_pub = rospy.Publisher('/lane_line0', Path, queue_size=1, latch=True)

        rospy.spin()

  def predict_motion(self):
    pass
