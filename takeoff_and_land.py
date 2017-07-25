from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import webcamPy

import argparse  
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, baud=57600, wait_ready=True)

## Esto es para mandar solo la velocidad, pero no la usamos en este script

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):

  print "Basic pre-arm checks"
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print " Waiting for vehicle to initialise..."
    time.sleep(1)
        
  print "Arming motors"
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print " Waiting for arming..."
    time.sleep(1)

  print "Taking off!"
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print " Altitude: ", vehicle.location.global_relative_frame.alt 
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print "Reached target altitude"
      break
    time.sleep(1)

# Initialize the takeoff sequence to 20m
arm_and_takeoff(20)

print("Take off complete")

# Hover for 10 seconds
time.sleep(10)

print("Objetivo identificado")

#Aqui llamamos a OpenCV

arr=[]

arr=webcamPy.findCenter()


msg = vehicle.message_factory.set_position_target_local_ned_encode(
    0,       # time_boot_ms (not used)
    0, 0,    # target_system, target_component
    mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
    0b0000111111000111, # type_mask (only speeds enabled)
    200, 2, 0, # x, y, z positions
    5, 5, 0, # x, y, z velocity in m/s
    0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
    0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
# send command to vehicle
vehicle.send_mavlink(msg)


print("Moviendonos a objetivo")
time.sleep(40)
print("Now let's land")
vehicle.mode = VehicleMode("LAND")

# Close vehicle object
vehicle.close()
