import rospy
from sensor_msgs.msg import NavSatFix
from mavros_test_common import MavrosTestCommon


def listener():
    rospy.init_node('listener', anonymous=True)

    ugv = MavrosTestCommon()

    ugv.log_topic_vars()

    rospy.spin()



if __name__ == '__main__':

    listener()
