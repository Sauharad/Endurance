from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math

def current_location(veh):
    dict={'lat':veh.location.global_relative_frame.lat,'lon':veh.location.global_relative_frame.lon,'alt':veh.location.global_relative_frame.alt}
    return dict

def haversine_distance(lat1, lon1, lat2, lon2):
    r = 6371000 
    dLat = math.radians(lat2 - lat1)
    dLon = math.radians(lon2 - lon1)
    a = math.sin(dLat / 2) * math.sin(dLat / 2) + \
        math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * \
        math.sin(dLon / 2) * math.sin(dLon / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = r * c
    return distance

vehicle=connect('udp:127.0.0.1:14550',wait_ready=False)
vehicle.mode=VehicleMode("GUIDED")
vehicle.armed=True

while not vehicle.armed:
    print("Waiting to Arm...")
    vehicle.armed=True
    time.sleep(2)

altitude=20
vehicle.simple_takeoff(altitude)
while current_location(vehicle)['alt'] < 0.95*altitude:
    print("Gaining Altitude. Current Altitude is: ",current_location(vehicle)['alt'])
    time.sleep(0.5)

waypoint=LocationGlobalRelative(28.75387327,77.11672037,20)
vehicle.simple_goto(waypoint)

while haversine_distance(waypoint.lat,waypoint.lon,current_location(vehicle)['lat'],current_location(vehicle)['lon'])>2:
    print("Moving towards waypoint. Current distance from point is: ",haversine_distance(waypoint.lat,waypoint.lon,current_location(vehicle)['lat'],current_location(vehicle)['lon']))
    time.sleep(1)

vehicle.mode=VehicleMode("RTL")
while current_location(vehicle)['alt'] > 0.5:
    print("Returning to Launch")
    time.sleep(1)

vehicle.armed=False
vehicle.close()