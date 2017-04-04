#!/usr/bin/env python
import rospy

import tf
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

first_fix = True
first_lat = float(0)
first_long = float(0)
path = Path()


def handle_gps_to_tf(msg, path_pub):
    global first_fix
    global first_lat
    global first_long

    if first_fix:
        first_fix = False
        first_lat = msg.latitude
        first_long = msg.longitude

    scale_factor = 100000.0
    dlat = scale_factor*(msg.latitude - first_lat)
    dlong = scale_factor*(msg.longitude - first_long)

    print('time: ' + str(rospy.Time.now()))
    print('first_lat: ' + str(first_lat))
    print('first_long: ' + str(first_long))
    print('msg.latitude: ' + str(msg.latitude))
    print('msg.longitude: ' + str(msg.longitude))
    print('dlat: ' + str(dlat))
    print('dlong: ' + str(dlong))
    print('===')

    br = tf.TransformBroadcaster()
    q = tf.transformations.quaternion_from_euler(0, 0, 0)
    br.sendTransform((dlat, dlong, 0), q, rospy.Time.now(), '/gps', '/odom')

    pose = PoseStamped()
    pose.header.frame_id = '/odom'
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = dlat
    pose.pose.position.y = dlong
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]

    path.header = pose.header
    path.poses.append(pose)
    path_pub.publish(path)

if __name__ == '__main__':
    print('gps_to_tf')
    path_pub = rospy.Publisher('/gps_path', Path, queue_size=10)
    rospy.init_node('gps_to_tf')
    rospy.Subscriber('/fix', NavSatFix, handle_gps_to_tf, path_pub)
    rospy.spin()
