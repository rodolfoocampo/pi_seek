from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
import time
import math
import webcamPy

import argparse  
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, baud=57600, wait_ready=True)

def goto(dNorth, dEast):
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
    the target position. This allows it to be called with different position-setting commands. 
    By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

    The method reports the distance to target every two seconds.
    """
    
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    vehicle.simple_goto(targetLocation)
    
    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print "Distance to target: ", remainingDistance
        if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
            print "Reached target"
            break;
        time.sleep(2)

## Esto es para mandar solo la velocidad, pero no la usamos en este script
def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation;


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

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

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

#Set home as the actual position
#vehicle.home_location=vehicle.location.global_frame

# Initialize the takeoff sequence to 20m
arm_and_takeoff(20)

print("Take off complete")

# Hover for 2 seconds
time.sleep(2)

print("Objetivo identificado")

#Aqui llamamos a OpenCV

arr=[]
objEncontrado= False
arr.append(0)
arr.append(0)

while arr[0]==0 and arr[1]==0:
	arr=webcamPy.findCenter()
	break
print arr[0]
print arr[1]

#while !objEncontrado:
	#muevete en un cuadrado que se va ampliando
x=arr[0]
y=arr[1]

goto(y,x)
#vehicle.send_mavlink(msg)


#print("Moviendonos a objetivo")
time.sleep(10)
#coordenadas=vehicle.location.global_relative_frame

#baja a cierta altitud
baja = LocationGlobalRelative(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, 3)

vehicle.simple_goto(baja)

time.sleep(10)

#abre servo
#RTL
sube = LocationGlobalRelative(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, 20)

vehicle.simple_goto(sube)

time.sleep(10)

print("Regresando a casa")
vehicle.mode    = VehicleMode("RTL")

time.sleep(10)

print("Now let's land")
#vehicle.mode = VehicleMode("LAND")

# Close vehicle object
vehicle.close()
