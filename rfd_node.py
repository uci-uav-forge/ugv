import queue
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
from mavros_test_common import MavrosTestCommon
import serial







def rfd():

    pos = (0,0,0)
    def position_callback(self, data):
        pos = (data.latitude, data.longitude, data.altitude)
    
    port  = "/dev/ttyAMA0"  #will most likely change
    baud = 57600
    timeout = 5
    ser = serial.Serial(port = port, baudrate = baud, timeout = timeout)

    rospy.init_node('rfd', anonymous=True)
    position_sub = rospy.Subscriber(
            'mavros/global_position/global', NavSatFix, position_callback)
    landed_pub = rospy.Publisher('landed', Bool, queue_size=1)
    z = Bool()
    wp_pub = rospy.Publisher('wp', NavSatFix, queue_size=10)


    while not rospy.is_shutdown():
        rmsg = ser.read()
        #while not (rmsg.startswith('uciforgeugv//')):
        #    rmsg = ser.read()
        if rmsg.startswith('uciforgeugv//'):
            rdata = rmsg.split('//')
            if rdata[1] == 'True':
                z.data = True
            else:
                z.data = False
            landed_pub.publish(z)

        
        coords = str(pos[0]) + ':' + str(pos[1]) # add subscriber to mavros msgs of global position

        smsg = 'uciforgeugv1//' + str(z.data) + '//' + coords
        ser.write(smsg)
        ser.flushInput()
        ser.flushOutput()



    ugv = MavrosTestCommon()

    ugv.log_topic_vars()

    rospy.spin()



if __name__ == '__main__':

    rfd()
