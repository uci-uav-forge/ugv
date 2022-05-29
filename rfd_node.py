import queue
import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import NavSatFix
from mavros_test_common import MavrosTestCommon
import serial

def rfd():
    rospy.init_node('rfd', anonymous=True)

    port  = "/dev/ttyAMA0"  #will most likely change
    baud = 57600
    timeout = 5
    ser = serial.Serial(port = port, baudrate = baud, timeout = timeout)

    pos = (0,0,0)
    def position_callback(data):
        global pos
        pos = (float(data.latitude), float(data.longitude), float(data.altitude))
    rospy.Subscriber('mavros/global_position/global', NavSatFix, position_callback)

    ll = -1
    def landingSubcallback(data):
        global ll
        ll = data.data
    rospy.Subscriber('landing', Int8, landingSubcallback)

    landing_pub = rospy.Publisher('landing', Int8, queue_size=2)
    # wp_pub = rospy.Publisher('wp', NavSatFix, queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rmsg = ser.read()
        if rmsg.startswith('uciforgeugv0//'):
            rdata = rmsg.split('//')
            landing_pub.publish(int(rdata[1]))


            '''
            if rdata[1] == 'True':
                z.data = True
            else:
                z.data = False
            landing_pub.publish(z)
            '''
            '''
            wp = rdata[2].split(':')
            wpp = NavSatFix(latitude=wp[0],longitude=wp[1])
            wp_pub.publish(wpp)
            '''
            
            smsg = 'uciforgeugv1//' + str(ll) + '//' + str(pos[0]) + ':' + str(pos[1])

        ser.write(smsg)

        ser.flushInput()
        ser.flushOutput()

        rate.sleep()

if __name__ == '__main__':

    rfd()
