"""
Simple script for take off and control with arrow keys
"""

import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import math


connection_address = '/dev/ttyACM0' #kontrol et
baud_rate = 115200
take_off_altitude = 5 #in meter
ground_speed = 5  # m/s
air_speed = 5  # m/s
land_speed= 60 # cm/s
rtl_altitude = 5 #in meter

Velocity_x = 0 #X ekseni hızı
Velocity_y = 0 #Y ekseni hızı
Velocity_z = 0 #Z ekseni hızı

alt_sensivity = 0.2


#Connect to the vehicle on given address
print ("\nConnecting to vehicle on: " + connection_address + " with baud rate: " + str(baud_rate))
vehicle = connect(connection_address, wait_ready=True, baud=baud_rate)


# -- Define arm and takeoff
def arm_and_takeoff(altitude):
    while not vehicle.is_armable:
        print("waiting to be armable")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed: time.sleep(1)

    print("Taking Off")
    vehicle.simple_takeoff(altitude)

    while True:
        v_alt = vehicle.location.global_relative_frame.alt
        print(">> Altitude = %.1f m" % v_alt)
        if v_alt >= altitude - 1.0:
            print("Target altitude reached")
            break
        time.sleep(1)


# -- Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html

    Bitmask to indicate which dimensions should be ignored by the vehicle 
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
    none of the setpoint dimensions should be ignored). Mapping: 
    bit 1: x,  bit 2: y,  bit 3: z, 
    bit 4: vx, bit 5: vy, bit 6: vz, 
    bit 7: ax, bit 8: ay, bit 9:


    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # -- BITMASK -> Consider only the velocities
        0, 0, 0,  # -- POSITION
        vx, vy, vz,  # -- VELOCITY
        0, 0, 0,  # -- ACCELERATIONS
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


def yukseklik_ayarla(vehicle, yukseklik, hiz, Vx, Vy):
    v_altitude = vehicle.location.global_relative_frame.alt
    
    if yukseklik > v_altitude:
        print("Yukseliniyor...")
        set_velocity_body(vehicle, Vx, Vy, -hiz)
        while True:
            v_alt = vehicle.location.global_relative_frame.alt
            print(">> Altitude = %.1f m" % v_alt)
            if v_alt >= yukseklik - alt_sensivity:
                print("Target altitude reached")
                break
    else:
        print("Alcaliniyor...")
        set_velocity_body(vehicle, Vx, Vy, hiz)
        while True:
            v_alt = vehicle.location.global_relative_frame.alt
            print(">> Altitude = %.1f m" % v_alt)
            if v_alt <= yukseklik + alt_sensivity:
                print("Target altitude reached")
                break
        
         
# ---- MAIN FUNCTION
# - Takeoff

#Set ground speed
#vehicle.groundspeed = ground_speed
#print (" Ground speed is set to " + str(ground_speed) + " m/s")

#Set air speed
#vehicle.airspeed = air_speed
#print ("Air speed is set to " + str(air_speed) + " m/s")

#Set rtl altitude
#vehicle.parameters['RTL_ALT'] = rtl_altitude

#Set landing speed
#vehicle.parameters['LAND_SPEED'] = land_speed

user_approval = input("Please press type 'arm' to start mission or type 'cancel' to cancel mission: ")
while not (user_approval == "arm" or user_approval == "cancel"):
    print("Invalid input, please type again...")
    user_approval = input("Please press type 'arm' to start mission or type 'cancel' to cancel mission: ")

if user_approval == "arm":
    # From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
    arm_and_takeoff(take_off_altitude)

    print("Command List\n"+
          "1 -> ileri\n"+
          "2 -> geri\n"+
          "3 -> sag\n"+
          "4 -> sol\n"+
          "5 -> alcal\n"+
          "6 -> yuksel\n"+
          "7 -> dur\n"+
          "8 -> durx\n"+
          "9 -> dury\n"+
          "10 -> durz\n"+
          "11 -> land")
    while True:
        command = input("Enter your command --> ")
        if command == "ileri":
            Velocity_x = 1
        elif command == "geri":
            Velocity_x = -1
        elif command == "sag":
            Velocity_y = 1
        elif command == "sol":
            Velocity_y = -1
        elif command == "alcal":
            Velocity_z = 0.2
        elif command == "yuksel":
            Velocity_z = -0.2
        elif command == "dur":
            Velocity_x = 0
            Velocity_y = 0
            Velocity_z = 0
        elif command == "durx":
            Velocity_x = 0
        elif command == "dury":
            Velocity_y = 0
        elif command == "durz":
            Velocity_z = 0
        elif command == "land":
            print('Vehicle landing...')
            vehicle.mode = VehicleMode("LAND")
            
        set_velocity_body(vehicle, Velocity_x, Velocity_y, Velocity_z)
        

#vehicle.close()
