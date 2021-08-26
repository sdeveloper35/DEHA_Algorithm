from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math

import RPi.GPIO as GPIO
import time

def get_pwm(angle):
    return (angle / 18) + 2.5

def servo_go(servo , angle):
    servo.ChangeDutyCycle(get_pwm(angle))

GPIO.setwarnings(False)

servoPIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

p = GPIO.PWM(servoPIN, 50) #GPIO 17 pini 50Hz ile pwm olarak ayarlandı
p.start(2.5) # Initialization 20ms

#servo_go(p , 0)

#p.stop()
#GPIO.cleanup()

################
#-----PARAMETERS
################
#---- Vehicle
#connection_address = 'tcp:192.168.4.1:23'
#connection_address = 'tcp:127.0.0.1:5763'
connection_address = '/dev/ttyACM0'
baud_rate = 115200
take_off_altitude = 5 #in meter
ground_speed = 5  # m/s
air_speed = 5  # m/s
land_speed= 30 # cm/s
rtl_altitude = 5 #in meter

#---- Waypoints
waypoint_havuz = LocationGlobalRelative(38.3714614, 27.2007268, 5)  # longitude, lattitude, altitude
waypoint_havuz_1m = LocationGlobalRelative(38.3714614, 27.2007268, 2)

waypoint_target = LocationGlobalRelative(38.3713973, 27.2007852, 5)  # longitude, lattitude, altitude
waypoint_target_2m = LocationGlobalRelative(38.3713973, 27.2007852, 3)


waypoint_land = LocationGlobalRelative(38.3713731, 27.2006913, 5)

#---------Functions
#---Print Parameters of Connected Vehicle
def print_vehicle_parameters():
    # Get all vehicle attributes (state)
    print ("\nGet all vehicle attribute values:")
    print (" Global Location: %s" % vehicle.location.global_frame)
    print (" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    print (" Local Location: %s" % vehicle.location.local_frame)
    print (" Attitude: %s" % vehicle.attitude)
    print (" Velocity: %s" % vehicle.velocity)
    print (" GPS: %s" % vehicle.gps_0)
    print (" EKF OK?: %s" % vehicle.ekf_ok)
    print (" Last Heartbeat: %s" % vehicle.last_heartbeat)
    print (" Rangefinder: %s" % vehicle.rangefinder)
    print (" Rangefinder distance: %s" % vehicle.rangefinder.distance)
    print (" Heading: %s" % vehicle.heading)
    print (" Is Armable?: %s" % vehicle.is_armable)
    print (" System status: %s" % vehicle.system_status.state)
    print (" Groundspeed: %s" % vehicle.groundspeed)
    print (" Airspeed: %s" % vehicle.airspeed)
    print (" Mode: %s" % vehicle.mode.name)
    print (" Armed: %s" % vehicle.armed)

"""
def Check_alt(TargetAltitude):
    if vehicle.location.global_relative_frame.alt < TargetAltitude:
        print("You are Ascending...")
    
#        İf target altitude is higher than our altitude
    
        while True:
            print (" Altitude: ", vehicle.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
                print ("Reached target altitude")
                break
            time.sleep(1)
            
    else if vehicle.location.global_relative_frame.alt > TargetAltitude:
        print("You are Descending...")
    
#        İf target altitude is lower than our altitude
    
        while True:
            print (" Altitude: ", vehicle.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if vehicle.location.global_relative_frame.alt <= aTargetAltitude * 1.05:
                print ("Reached target altitude")
                break
            time.sleep(1)
"""
#----Arm and Take Off
# TODO: take off and landing speed will be adjusted via a parameter
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print ("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)
    print ("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)
    print ("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)
#---- Get Distance in Meters
def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5
#---- Distance Between The Vehicle and Given Waypoint
def waypoint_distance(waypoint):
    distance = get_distance_metres(vehicle.location.global_frame, waypoint)
    return distance
def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a Location object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location has the same `alt and `is_relative` values
    as `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))
    print ("dlat, dlon", dLat, dLon)
    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return(newlat, newlon)


print ("\nConnecting to vehicle on: " + connection_address + " with baud rate: " + str(baud_rate))
vehicle = connect(connection_address, wait_ready=True, baud=baud_rate)

#Set first arming for rtl
waypoint_home = vehicle.location.global_frame
#Print all vehicle parameters
print_vehicle_parameters()
# TODO: After printing vehicle state, to continue the program ask for user approval and serve 3 option: refresh vehicle state, arm the vehicle, terminate the program
#Set ground speed
vehicle.groundspeed = ground_speed
print (" Ground speed is set to " + str(ground_speed) + " m/s")
#Set air speed
vehicle.airspeed = air_speed
print ("Air speed is set to " + str(air_speed) + " m/s")
#Set rtl altitude
vehicle.parameters['RTL_ALT'] = rtl_altitude
#Set landing speed
vehicle.parameters['LAND_SPEED'] = land_speed
user_approval = input("Please press type 'arm' to start mission or type 'cancel' to cancel mission: ")
while not (user_approval == "arm" or user_approval == "cancel"):
    print("Invalid input, please type again...")
    user_approval = input("Please press type 'arm' to start mission or type 'cancel' to cancel mission: ")
if user_approval == "arm":
    # From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
    arm_and_takeoff(take_off_altitude)
    vehicle.simple_goto(waypoint_havuz)
    print ('Vehicle is going to waypoint_havuz')
    while waypoint_distance(waypoint_havuz)>=1:
        print ('Distance to waypoint havuz : %s' % waypoint_distance(waypoint_havuz))
        time.sleep(1)
        
    print("Water mechanism open...")
    servo_go(p , 180)
    vehicle.simple_goto(waypoint_havuz_1m)
    print("Descending to 1 meter...")
    print("20 seconds just started")
    time.sleep(20)
    print("Water mechanism closed...")
    servo_go(p , 0)
    time.sleep(5)
    vehicle.simple_goto(waypoint_havuz)
    print("Vehicle Ascending to 5 meter again...")
    time.sleep(7)
    
    vehicle.simple_goto(waypoint_target)
    print ('Vehicle is going to waypoint target')
    while waypoint_distance(waypoint_target)>=1:
        print ('Distance to waypoint target : %s' % waypoint_distance(waypoint_target))
        time.sleep(1)
    
    print("Descending for 2 meter...")
    vehicle.simple_goto(waypoint_target_2m)
    time.sleep(7)
    print("Water mechanism opening...")
    print("15 sec. to dropping water...")
    servo_go(p , 180)
    time.sleep(15)
    print("Water mechanism closing...")
    servo_go(p , 0)
    time.sleep(5)
    print("Ascending for 5 meter...")
    vehicle.simple_goto(waypoint_target)
    time.sleep(7)
    
    
    print("Inis konumuna gidiliyor...")
    vehicle.simple_goto(waypoint_land)
    print ('Vehicle is going to waypoint land')
    while waypoint_distance(waypoint_land)>=1:
        print ('Distance to waypoint land : %s' % waypoint_distance(waypoint_land))
        time.sleep(1)
    
    
    
    
    
    #Update home location to first take off point
    vehicle.home_location = waypoint_home
    print ('Vehicle landing...')
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed == True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        time.sleep(1)
    print ('Landing Completed')
    print ("Close vehicle object")
    vehicle.close()
else:
    print("Mission denied...")
    print ("Close vehicle object")
    vehicle.close()

p.stop()
GPIO.cleanup()
print("PWM pini kapatildi")
