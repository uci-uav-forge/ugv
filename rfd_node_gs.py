import string
from tokenize import String
import rospy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import NavSatFix
from mavros_test_common import MavrosTestCommon
import serial
import time


def rfd():
    rospy.init_node('rfd_gs', anonymous=True)

    port  = "COM5"  #will most likely change
    baud = 57600
    timeout = 5
    ser = serial.Serial(port = port, baudrate = baud, timeout = timeout)

    wp = (0,0,0)
    def position_callback(data):
        global wp
        wp = (float(data.latitude), float(data.longitude), float(data.altitude))
    rospy.Subscriber('ugv_waypoint', NavSatFix, position_callback)

    ll = False
    def landing_callback(data):
        global ll
        ll = data.data
    rospy.Subscriber('ugv_landing', Bool, queue_size=1)


    landed_pub = rospy.Publisher('ugv_landed', Bool, queue_size=10)
    z = Bool()

    pos_pub = rospy.Publisher('ugv_position', NavSatFix, queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        smsg = 'uciforgeugv0//' + str(ll) + '//' + str(wp[0]) + ':' + str(wp[1])

        time.sleep(5)
        rmsg = ser.read()
        if rmsg.startswith('uciforgeugv1//'):
            rdata = rmsg.split('//')
            if rdata[1] == 'True':
                z.data = True
            else:
                z.data = False
            landed_pub.publish(z) # this publishes so another decoupling node can reel back in drone

            pos = rdata[2].split(':')
            poss = NavSatFix(latitude=pos[0],longitude=pos[1])
            pos_pub.publish(poss)


        
        ser.write(smsg)

        ser.flushInput()
        ser.flushOutput()

        rate.sleep()


if __name__ == '__main__':

    rfd()
