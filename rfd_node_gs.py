import string
import rospy
from sensor_msgs.msg import NavSatFix
from mavros_test_common import MavrosTestCommon
import serial


def rfd():

    port  = "COM5"  #will most likely change
    baud = 57600
    timeout = 5

    ser = serial.Serial(port = port, baudrate = baud, timeout = timeout)


    rospy.init_node('rfd_gs', anonymous=True)

    rmsg = ser.read()

    #coords = ''

    if rmsg.startswith('uciforgeugv1//'):
        rmsg_s = rmsg.split('//')
        landed = True if rmsg_s[1] == 'True' else False # change a topic that will be used to reel back in the decoubling line
        coords = rmsg_s[2]




    landing = True # change to whatever landing topic is from when drone starts to drop ugv

    if landing:
        smsg = 'uciforgeugv0//UGV_Landing'
        ser.write(smsg)



    ugv = MavrosTestCommon()

    ugv.log_topic_vars()

    rospy.spin()



if __name__ == '__main__':

    rfd()
