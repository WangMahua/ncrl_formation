import rospy
import pymap3d as pm
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped


self_pose = PoseStamped()
gps_l = NavSatFix()
gps_f = NavSatFix()

def leader_lla_cb(data):
	gps_l.header = data.header
	gps_l.latitude = data.latitude
	gps_l.longitude = data.longitude
	gps_l.altitude = data.altitude
def self_lla_cb(data):
	gps_f.latitude = data.latitude
	gps_f.longitude = data.longitude
	gps_f.altitude = data.altitude

rospy.init_node('lla2enu', anonymous=True)
rospy.Subscriber("/MAV1/mavros/global_position/global", NavSatFix, leader_lla_cb)
rospy.Subscriber("/MAV2/mavros/global_position/global", NavSatFix, self_lla_cb)
enu_pub = rospy.Publisher('/MAV2/mavros/lla2enu/relative', PoseStamped, queue_size=10)
rate = rospy.Rate(120)

while not rospy.is_shutdown():
	try:
		coordinate = pm.geodetic2enu(gps_f.latitude, gps_f.longitude, gps_f.altitude, gps_l.latitude, gps_l.longitude, gps_l.altitude)
		self_pose.pose.position.x = coordinate[0]
		self_pose.pose.position.y = coordinate[1]
		self_pose.pose.position.z = coordinate[2]
		enu_pub.publish(self_pose)
	except rospy.ROSInterruptException:
		pass

rospy.spin()