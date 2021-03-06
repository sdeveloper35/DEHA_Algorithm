"""
Simple script for take off and control with arrow keys
"""


import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

import math

import RPi.GPIO as GPIO
#- Importing Tkinter: sudo apt-get install python-tk
import tkinter as tk

connection_address = '/dev/ttyACM0'
baud_rate = 115200
take_off_altitude = 5 #in meter
ground_speed = 5  # m/s
air_speed = 5  # m/s
land_speed= 60 # cm/s
rtl_altitude = 5 #in meter

#-- Connect to the vehicle
print ("\nConnecting to vehicle on: " + connection_address + " with baud rate: " + str(baud_rate))
vehicle = connect(connection_address, wait_ready=True, baud=baud_rate)

#-- Setup the commanded flying speed
gnd_speed = 5 # [m/s]

#-- Define arm and takeoff
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
      print(">> Altitude = %.1f m"%v_alt)
      if v_alt >= altitude - 1.0:
          print("Target altitude reached")
          break
      time.sleep(1)
      
 #-- Define the function for sending mavlink velocity command in body frame
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
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
    
    
#-- Key event function
def key(event):
    if event.char == event.keysym: #-- standard keys
        if event.keysym == 'r':
            print("r pressed >> Set the vehicle to RTL")
            vehicle.mode = VehicleMode("LAND")
            
    else: #-- non standard keys
        if event.keysym == 'Up':
            set_velocity_body(vehicle, gnd_speed, 0, 0)
        elif event.keysym == 'Down':
            set_velocity_body(vehicle,-gnd_speed, 0, 0)
        elif event.keysym == 'Left':
            set_velocity_body(vehicle, 0, -gnd_speed, 0)
        elif event.keysym == 'Right':
            set_velocity_body(vehicle, 0, gnd_speed, 0)
    
    
#---- MAIN FUNCTION
#- Takeoff
user_approval = input("Please press type 'arm' to start mission or type 'cancel' to cancel mission: ")
while not (user_approval == "arm" or user_approval == "cancel"):
    print("Invalid input, please type again...")
    user_approval = input("Please press type 'arm' to start mission or type 'cancel' to cancel mission: ")
    
if user_approval == "arm":
   arm_and_takeoff(take_off_altitude)
   print("arm oluyor")
else:
    print("Mission denied...")
    print ("Close vehicle object")
    vehicle.close()
#- Read the keyboard with tkinter
#set_velocity_body(vehicle, 2,0,0)
#time.sleep(3)
#vehicle.mode = VehicleMode("LAND")

while True:
    command = input("Please press direction")
    if command  == "ileri":
        set_velocity_body(vehicle, 1,0,0)
    elif command == "geri":
        set_velocity_body(vehicle, -1,0,0)
    elif command == "sag":
        set_velocity_body(vehicle, 0,1,0)
    elif command == "sol":
        set_velocity_body(vehicle, 0,-1,0)
    elif command == "dur":
        vehicle.mode = VehicleMode("LAND")


#root = tk.Tk()
#print(">> Control the drone with the arrow keys. Press r for RTL mode")
#root.bind_all('<Key>', key)
#root.mainloop()
#GPIO.cleanup()
print("PWM pini kapatildi")
