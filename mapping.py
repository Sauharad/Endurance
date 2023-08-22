from dronekit import connect, LocationGlobalRelative, VehicleMode
import time
import math

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

def calc_bearing(lat1, long1, lat2, long2):
  # Convert latitude and longitude to radians
  lat1 = math.radians(lat1)
  long1 = math.radians(long1)
  lat2 = math.radians(lat2)
  long2 = math.radians(long2)
  
  # Calculate the bearing
  bearing = math.atan2(
      math.sin(long2 - long1) * math.cos(lat2),
      math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(long2 - long1)
  )
  
  # Convert the bearing to degrees
  bearing = math.degrees(bearing)
  
  # Make sure the bearing is positive
  bearing = (bearing + 360) % 360
  return bearing

def reverse_haversine(point,distance,bearing):
    r=6371000
    la1=math.radians(point.lat)
    lo1=math.radians(point.lon)  
    ad=distance/r
    theta=math.radians(bearing)
    lat2=math.degrees(math.asin(math.sin(la1)*math.cos(ad)+math.cos(la1)*math.cos(theta)*math.sin(ad)))
    lon2=math.degrees(lo1+math.atan2(math.sin(theta)*math.sin(ad)*math.cos(la1),math.cos(ad)-math.sin(la1)*math.sin(lat2)))
    return LocationGlobalRelative(lat2,lon2,50)

def current_location(veh):
    return [veh.location.global_relative_frame.lat,veh.location.global_relative_frame.lon,veh.location.global_relative_frame.alt]

def path(p1,p2,p3,p4):
    d1=haversine_distance(p1.lat,p1.lon,p2.lat,p2.lon)
    d2=haversine_distance(p3.lat,p3.lon,p4.lat,p4.lon)
    b1=calc_bearing(p1.lat,p1.lon,p2.lat,p2.lon)
    b1=(b1+180)%360
    b2=calc_bearing(p4.lat,p4.lon,p3.lat,p3.lon)
    b2=(b2+180)%360
    n=int(d1/5)
    m=int(d2/5)
    points=[]
    for i in range (1,n+1):
        if (i%4==0 or i%4==1):
            dd=d1-5*i
            points[i-1]=reverse_haversine(p2,dd,b1)
        else:
            dd=d2-5*i
            points[i-1]=reverse_haversine(p3,dd,b2)
    return points

def goto(veh,point):
    veh.simple_goto(point)
    while haversine_distance(current_location(vehicle)[0],current_location(vehicle)[1],point.lat,point.lon) > 1:
        print("Moving towards next waypoint")
        time.sleep(1)


point1=LocationGlobalRelative(28.75314672,77.11571890,50)
point2=LocationGlobalRelative(28.75320376,77.11651421,50)
point3=LocationGlobalRelative(28.75428766,77.11642745,50)
point4=LocationGlobalRelative(28.75428133,77.11568998,50)

vehicle=connect('udp:127.0.0.1:14550',wait_ready=False)
vehicle.mode=VehicleMode("GUIDED")
vehicle.armed=True
while not vehicle.armed:
    print("Waiting to Arm")
    vehicle.armed=True
    time.sleep(1)

altitude=50
vehicle.simple_takeoff(altitude)
while not current_location(vehicle)[2]>0.95*altitude:
    print("Gaining Altitude")
    time.sleep(1)

vehicle.simple_goto(point1)
while haversine_distance(current_location(vehicle)[0],current_location(vehicle)[1],point1.lat,point1.lon)>1:
    print("Moving towards waypoint")
    time.sleep(1)

grid=path(point1,point2,point3,point4)
for i in grid:
    goto(vehicle,i)

# d1=haversine_distance(point1.lat,point1.lon,point2.lat,point2.lon)
# d2=haversine_distance(point4.lat,point3.lon,point4.lat,point4.lon)
# dd=d1
# go=reverse_haversine(point2,d1,(calc_bearing(point1.lat,point1.lon,point2.lat,point2.lon)+180)%360)
# vehicle.simple_goto(go)
# while haversine_distance(current_location(vehicle)[0],current_location(vehicle)[1],go.lat,go.lon)>1:
#     print("Moving towards waypoint")
#     time.sleep(1)

# go2=reverse_haversine(point3,d2 ,(calc_bearing(point4.lat,point4.lon,point3.lat,point3.lon)+180)%360)
# vehicle.simple_goto(go2)
# iteration=1
# while haversine_distance(current_location(vehicle)[0],current_location(vehicle)[1],point2.lat,point2.lon)>0.5:
#     if (iteration%4 == 1 or iteration%4 == 0):
#         goto(point2,d1,iteration,vehicle)
#     else:
#         goto(point3,d2,iteration,vehicle)
#     iteration+=1

#vehicle.mode=VehicleMode("LAND")


