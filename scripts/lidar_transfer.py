import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tf

def fastlio_callback(data):
    local_pose = PoseStamped()
    local_pose.header.frame_id = 'map'  # 或 'odom'
    local_pose.header.stamp = rospy.Time.now()
    
    # 直接使用FAST-LIO的输出
    local_pose.pose.position = data.pose.pose.position
    local_pose.pose.orientation = data.pose.pose.orientation
    
    position_pub.publish(local_pose)

rospy.init_node('fastlio_to_mavros')
rospy.Subscriber("/Odometry", Odometry, fastlio_callback, queue_size=1)
position_pub = rospy.Publisher("/solo_0/mavros/vision_pose/pose", PoseStamped, queue_size=1)
rospy.spin()