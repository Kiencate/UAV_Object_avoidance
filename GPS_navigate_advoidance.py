import rospy
from clover import srv
import math
from std_srvs.srv import Trigger
from std_msgs.msg import Int8
import time
control = 0
rospy.init_node('flight')
                                    # TRUYEN NHAN MESSAGE
def callback(data):
    global control
    control = data.data
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def Check():
    rospy.Subscriber("control", Int8, callback)
        
        
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

def get_distance_global(lat1, lon1, lat2, lon2):
    #print "distance: %s"%(math.hypot(lat1 - lat2, lon1 - lon2) * 1.113195e5)
    return math.hypot(lat1 - lat2, lon1 - lon2) * 1.113195e5

def findAngle(src,dst):
    lat1 = math.radians(src[0])
    lon1 = math.radians(src[1])
    lat2 = math.radians(dst[0])
    lon2 = math.radians(dst[1])
    dLon = lon2 - lon1
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
    brng = math.atan2(y,x)
    if brng > -math.pi/2:
        brng = math.pi/2-brng
    else:
        brng = -math.pi*3/2-brng

    return brng

def change_yaw(dst_local):
    src= [get_telemetry().lat, get_telemetry().lon]
    yaw = findAngle(src,dst_local)
    navigate(x=0, y=0, z=0, frame_id='map', yaw=float(yaw))
    while not rospy.is_shutdown():
        yaw_src = get_telemetry(frame_id = 'map').yaw
        yaw_dst = get_telemetry(frame_id='navigate_target').yaw
        if math.abs(yaw_src-yaw_dst)<0.02:
            break
        rospy.sleep(0.1)

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), yaw_rate = 0, speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, yaw_rate= yaw_rate, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)

def navigate_global_advoidance(lat=0, lon=0, frame_id='body', auto_arm=False, tolerance=0.3):
    dst = [lat,lon]
    arm = True
    while (not rospy.is_shutdown()) and arm:

        change_yaw(dst)
        print("Xoay goc")
        while True:
            #f = open('object.txt','r')
            #control = f.read()
            #f.close()

            if(control==1):
                navigate_wait(x=0, y=0.5, z=0, frame_id='body', yaw=float('nan'), yaw_rate=0)
                print("turn left: ",round(time.time(),2))
                break
            elif(control==2):
                navigate_wait(x=0, y=-0.5, z=0, frame_id='body', yaw=float('nan'), yaw_rate=0)
                print("turn right: ",round(time.time(),2))
                break
            elif(control==0):
                navigate_global(lat=lat, lon=lon, z=0, yaw=float('nan'), yaw_rate= 0, speed=0.5, frame_id=frame_id, auto_arm=auto_arm)
                print("flying forward: ",round(time.time(),2))
                telem = get_telemetry(frame_id='navigate_target')
                if get_distance_global(telem.lat,telem.lon,lat,lon)  < tolerance:
                    arm = False
                    break
                rospy.sleep(0.5)
                continue


destination = [21.0065356, 105.8430258]
# Take off and hover 1 m above the ground
navigate_wait(x=0, y=0, z=1.2, frame_id='body', auto_arm=True)

# Wait for 3 seconds
rospy.sleep(3)

navigate_global_advoidance(lat = destination[0],lon=destination[1],auto_arm = True)

land_wait()
