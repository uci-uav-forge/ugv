import csv
from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
import time
import socket
import exceptions
import math
import argparse
from pymavlink import mavutil
import ugv_functions as ugv

vehicle = ugv.connectMyCopter()
ugv.paramSetup(vehicle)
ugv.arm(vehicle)

with open('ringroadcoords.txt') as csv_f:
    csv_reader = csv.reader(csv_f, delimiter=',')
    for row in csv_reader:
        ugv.goto(vehicle, LocationGlobalRelative(round(float(row[0]), 5) , round(float(row[1]), 5), 10))
