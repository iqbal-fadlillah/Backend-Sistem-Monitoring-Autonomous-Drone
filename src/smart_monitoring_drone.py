from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command, Vehicle
import time
import math
from pymavlink import mavutil
import asyncio
import argparse
import RPi.GPIO as GPIO
import pyrebase
import paho.mqtt.client as paho
import datetime
import numpy as np
import serial
import random
import pandas as pd
from serial_asyncio import open_serial_connection
import tensorflow as tf

nextwaypoint = 0
windspeed = 0
battery = 12.4


volume_total = 506.755  # ml
debit_air_high = 6.755  # masih ngide harusnya 2x
debit_air_medium = 5.43  # masih ngide harusnya 2x
debit_air_low = 1.72  # masih ngide harusnya 2x

vel_sprayer = 100

ena = 18  # 12
in1 = 23  # 16
in2 = 24  # 18

interpreter = tf.lite.Interpreter(model_path="percobaan3.tflite")
interpreter.allocate_tensors()
interpreter1 = tf.lite.Interpreter(model_path="cobamodel_2.tflite")
interpreter1.allocate_tensors()

loop = asyncio.get_event_loop()

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(ena, GPIO.OUT)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
pwm = GPIO.PWM(ena, 100)
pwm.start(0)

firebaseConfig = {
    "apiKey": "AIzaSyCFtkJaUFOnhjnBoAy1MohoPvS02U6zItI",
    "authDomain": "dronev2-b91f3.firebaseapp.com",
    "databaseURL": "https://dronev2-b91f3-default-rtdb.asia-southeast1.firebasedatabase.app",
    "projectId": "dronev2-b91f3",
    "storageBucket": "dronev2-b91f3.appspot.com",
    "messagingSenderId": "940590245252",
    "appId": "1:940590245252:web:66e4424f047e203cd469d1"
}

firebase = pyrebase.initialize_app(firebaseConfig)
db = firebase.database()

parser = argparse.ArgumentParser(
    description='Demonstrates basic mission operations.')
parser.add_argument('--connect', default='127.0.0.1:14550',
                    help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=921600, wait_ready=True)

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def distance_to_current_waypoint():
    nextwaypoint = vehicle.commands.next
    if nextwaypoint == 0:
        return None
    missionitem = vehicle.commands[nextwaypoint-1]  # commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat, lon, alt)
    distancetopoint = get_distance_metres(
        vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

def download_mission():
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()  # wait until download is complete.

async def windspedd_value():
    global windspeed
    reader, writer = await open_serial_connection(url='/dev/rfcomm0', baudrate=9600)
    line = await reader.readline()
    windspeed = float(str(line, 'utf-8'))

async def data_database():
    global altitude, waktu_altitude, groundspeed, waktu_groundspeed, latitude, waktu_latitude, longitude, waktu_longitude, windspeed, waktu_windspeed, battery, waktu_Battery, waktu, nextwaypoint, error_gps, waktu_errorgps
    waktu = time.asctime(time.localtime(time.time())) + " ms:"+ str(datetime.datetime.now().microsecond/1000)
    altitude = vehicle.location.global_relative_frame.alt
    waktu_altitude = (str(altitude) + " " + time.asctime(time.localtime(time.time())) + " ms = " + str(datetime.datetime.now().microsecond/1000))
    groundspeed = np.round(vehicle.groundspeed, 4)
    waktu_groundspeed = (str(groundspeed) + " " + time.asctime(time.localtime(time.time())) + " ms = " + str(datetime.datetime.now().microsecond/1000))
    latitude = vehicle.location.global_relative_frame.lat
    waktu_latitude = (str(latitude) + " " + time.asctime(time.localtime(time.time())) + " ms = " + str(datetime.datetime.now().microsecond/1000))
    longitude = vehicle.location.global_relative_frame.lon
    waktu_longitude = (str(longitude) + " " + time.asctime(time.localtime(time.time())) + " ms = " + str(datetime.datetime.now().microsecond/1000))
    waktu_windspeed = (str(windspeed) + " " + time.asctime(time.localtime(time.time())) + " ms = " + str(datetime.datetime.now().microsecond/1000))
    battery = round((battery) - (random.uniform(0.02,0.08)),4)
    waktu_Battery = (str(battery) + " " + time.asctime(time.localtime(time.time())) + " ms = " + str(datetime.datetime.now().microsecond/1000))
    nextwaypoint = vehicle.commands.next
    error_gps = vehicle.gps_0.eph
    waktu_errorgps = (str(error_gps) + " " + time.asctime(time.localtime(time.time())) + " ms = " + str(datetime.datetime.now().microsecond/1000))
    return altitude, waktu_altitude, groundspeed, waktu_groundspeed, latitude, waktu_latitude, longitude, waktu_longitude, windspeed, waktu_windspeed, battery, waktu_Battery, waktu, error_gps, waktu_errorgps

async def dic_database():
    global push_altitude, push_groundspeed, push_latitude, push_longitude, push_windspeed, push_battery, push_errorgps
    push_altitude = {"Altitude": altitude, "Altitude (Waktu)": waktu_altitude, "Waktu":waktu}
    push_groundspeed = {"Ground_Speed": groundspeed,"Groundspeed (Waktu)": waktu_groundspeed, "Waktu":waktu}
    push_latitude = {"Latitude": latitude, "Latitude(Waktu)": waktu_latitude, "Waktu":waktu}
    push_longitude = {"Longitude": longitude, "Longitude(Waktu)": waktu_longitude, "Waktu":waktu} 
    push_windspeed = {"Windspeed": windspeed, "Windspeed(Waktu)": waktu_windspeed, "Waktu":waktu}
    push_battery = {"Battery": battery, "Battery(Waktu)": waktu_Battery, "Waktu":waktu}
    push_errorgps = {"Error GPS": error_gps, "Error GPS(Waktu)": waktu_errorgps, "Waktu":waktu}
    return push_altitude, push_groundspeed, push_latitude, push_longitude, push_windspeed, push_battery, push_errorgps

async def push_database():
    db.child("Altitude (Tampil)").set({"Altitude": altitude})
    db.child("Altitude (Waktu)").push(push_altitude)
    db.child("Ground Speed (Tampil)").set({"Ground_Speed": groundspeed})
    db.child("Groundspeed (Waktu)").push(push_groundspeed)
    db.child("Latitude (Tampil)").set({"Latitude": latitude})
    db.child("Latitude (Waktu)").push(push_latitude)
    db.child("Longitude (Tampil)").set({"Longitude": longitude})
    db.child("Longitude (Waktu)").push(push_longitude)
    db.child("Windspeed (Tampil)").set({"Windspeed": windspeed})
    db.child("Windspeed (Waktu)").push(push_windspeed)
    db.child("Battery (Tampil)").set({"Battery": battery})
    db.child("Battery (Waktu)").push(push_battery)
    db.child("Error GPS (Tampil)").set({"Error GPS": error_gps})
    db.child("Error GPS (Waktu)").push(error_gps)

async def dataPenyiraman():
    global volume_total, waktu_volume, putaran_motor, waktu_vel_sprayer, waktu
    waktu = time.asctime(time.localtime(time.time())) + " ms:"+ str(datetime.datetime.now().microsecond/1000)
    volume_total = np.round(volume_total, 4)
    waktu_volume = (str(volume_total) + " " + time.asctime(time.localtime(time.time())) + " ms = " + str(datetime.datetime.now().microsecond/1000))
    putaran_motor = vel_sprayer
    waktu_vel_sprayer = (str(putaran_motor)+"%" + " " + time.asctime(time.localtime(time.time())) + " ms = " + str(datetime.datetime.now().microsecond/1000))
    return volume_total, waktu_volume, putaran_motor, waktu_vel_sprayer, waktu

async def dicPenyiraman():
    global volume_air, kekuatan_penyiraman
    volume_air = {"Volume_Air": volume_total, "Volume (Waktu)": waktu_volume, "Waktu": waktu}
    kekuatan_penyiraman = {"Kekuatan_Penyiraman": putaran_motor, "Kekuatan_Penyiraman(Waktu)": waktu_vel_sprayer, "Waktu": waktu}
    return volume_air, kekuatan_penyiraman

async def pushPenyiraman():
    db.child("Volume Air (Tampil)").set({"Volume_Air": volume_total})
    db.child("Volume Air (Waktu)").push(volume_air)
    db.child("Kekuatan Penyiraman (Tampil)").set({"Kekuatan_Penyiraman": putaran_motor})
    db.child("Kekuatan Penyiraman (Waktu)").push(kekuatan_penyiraman)

async def smartSprayer():
    velocity = groundspeed
    global volume_total, vel_sprayer
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    data = {'Altitude': [altitude],
        'Kecepatan Angin' : [windspeed],
        'Kecepatan drone': [groundspeed]}
    data_test = pd.DataFrame(data)
    input_data = np.array(data_test, dtype=np.float32)
    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()
    output_data = interpreter.get_tensor(output_details[0]['index'])
    kelas = np.argmax(output_data)
    if nextwaypoint == 2:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        vel_sprayer = 0
    elif (kelas == 0): # state 1 (L L L) = LOW
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
        vel_sprayer = 25
        pwm.ChangeDutyCycle(vel_sprayer)
        volume_total -= debit_air_low
        # print("volume_total: ", volume_total)
        return volume_total, vel_sprayer
    elif (kelas == 1) :  # state 2 (L L M) = LOW
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
        vel_sprayer = 75
        pwm.ChangeDutyCycle(vel_sprayer)
        volume_total -= debit_air_medium
        # print("volume_total: ", volume_total)
        return volume_total, vel_sprayer
    elif (kelas == 2): # state 2 (L L M) = LOW
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
        vel_sprayer = 100
        pwm.ChangeDutyCycle(vel_sprayer)
        volume_total -= debit_air_high
        # print("volume_total: ", volume_total)
        return volume_total, vel_sprayer
    else:
        print("Kondisi Tidak ditemukan")
        return

async def mainData():
    while True:
        task1 = loop.create_task(data_database())
        task2 = loop.create_task(dic_database())
        task3 = loop.create_task(push_database())
        await asyncio.wait([task1, task2, task3])
        break

async def mainDataPenyiraman():
    while True:
        task1 = loop.create_task(windspedd_value())
        task2 = loop.create_task(data_database())
        task3 = loop.create_task(dic_database())
        task4 = loop.create_task(push_database())
        task5 = loop.create_task(dataPenyiraman())
        task6 = loop.create_task(dicPenyiraman())
        task7 = loop.create_task(pushPenyiraman())
        task8 = loop.create_task(smartSprayer())
        await asyncio.wait([task1, task2, task3, task4, task5, task6, task7, task8])
        break

def replace_with_mapping(value):
    mapping = {0: "Drone Terbang Sesuai Jalur", 1: "Drone Melaju Terlalu Cepat", 2: "Drone Terbang Terlalu Tinggi", 3: "Drone Terbang Diluar Jalur", 4: "Drone Terbang Diluar Jalur dan Terlalu Tinggi", 5: "Drone Terbang Terlalu Tinggi dan Melaju Cepat", 6: "Drone Terbang Diluar Jalur dan Cepat", 7: "Drone Abnormal"}
    return mapping.get(value, value)

def prediksiKondisi():
    # global error_gps
    input_details1 = interpreter1.get_input_details()
    output_details1 = interpreter1.get_output_details()
    data1 = {'Error GPS': [error_gps],
        'Altitude' : [altitude],
        'Ground Speed': [groundspeed]}
    data_test1 = pd.DataFrame(data1)
    input_data1 = np.array(data_test1, dtype=np.float32)
    print(type(input_data1))
    print(input_data1)
    # bismillah = input_data1
    interpreter1.set_tensor(input_details1[0]['index'],input_data1)
    interpreter1.invoke()
    output_data = interpreter.get_tensor(output_details1[0]['index'])
    kelas = np.argmax(output_data)
    kelas = np.array(kelas)
    vectorized_function = np.vectorize(replace_with_mapping)
    output_array = vectorized_function(kelas)
    output_array = np.array_str(output_array)
    print(output_array)
    db.child("Kondisi (Tampil)").set({"INFORMASI": output_array})
    return output_array
    
def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
        print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    while True:
        # push_database()
        loop.run_until_complete(mainData())
        prediksiKondisi()
        # Trigger just below target alt.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude*0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

print('Create a new mission (for current location)')
download_mission()

# From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
arm_and_takeoff(4)

print("Starting mission")
# Reset mission set to first (0) waypoint
vehicle.commands.next = 0

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")
# nextwaypoint=vehicle.commands.next
while True:
    nextwaypoint=vehicle.commands.next
   # print("nxt while true: ", nextwaypoint)
    if nextwaypoint == 1:
        # print("masuk if else")
        loop.run_until_complete(mainDataPenyiraman())
        prediksiKondisi()
        # loop.run_until_complete(windspedd_value())
        # loop.run_until_complete(mainDataPenyiraman())
    elif nextwaypoint == 2:
        vel_sprayer = 0 
        loop.run_until_complete(mainDataPenyiraman())
        prediksiKondisi()
        break

vehicle.mode == VehicleMode("RTL")
print('Return to launch')

while True:
    loop.run_until_complete(mainData())
    prediksiKondisi()
    if vehicle.location.global_relative_frame.alt <= 0:
        break

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()
