import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
import sys
import tf

"""
author: xtdrone
modification and annotation: pulishen 
Obtain the true pose of UAV
"""

vehicle_type = sys.argv[1] # uav type
vehicle_num = int(sys.argv[2]) # uav num
multi_pose_pub = [None]*vehicle_num # pose publisher
multi_speed_pub = [None]*vehicle_num # velocity publisher
multi_local_pose = [PoseStamped() for i in range(vehicle_num)]
multi_speed = [Vector3Stamped() for i in range(vehicle_num)]
multi_odom_groundtruth_sub = [None]*vehicle_num # groundtruth subscriber

def odm_groundtruth_callback(msg, i):
    multi_local_pose[i].header = msg.header
    multi_local_pose[i].header.frame_id = 'map'
    multi_local_pose[i].pose = msg.pose.pose
    multi_speed[i].header = msg.header
    multi_speed[i].vector = msg.twist.twist.linear

if __name__ == '__main__':
    rospy.init_node(vehicle_type+'get_pose_groundtruth')
    for i in range(vehicle_num):
        multi_odom_groundtruth_sub[i] = rospy.Subscriber('/xtdrone/'+vehicle_type+'_'+str(i)+'/ground_truth/odom', Odometry, odm_groundtruth_callback, i)
        multi_pose_pub[i] = rospy.Publisher(vehicle_type+'_'+str(i)+'/mavros/vision_pose/pose', PoseStamped, queue_size=2)
        multi_speed_pub[i] = rospy.Publisher(vehicle_type+'_'+str(i)+'/mavros/vision_speed/speed', Vector3Stamped, queue_size=2)
    rate = rospy.Rate(100)
    while True:
        for i in range(vehicle_num):
            multi_pose_pub[i].publish(multi_local_pose[i])
            multi_speed_pub[i].publish(multi_speed[i])
        rate.sleep()


