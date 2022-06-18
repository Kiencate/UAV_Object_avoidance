import rospy
from clover import srv
import math
from std_srvs.srv import Trigger
import haversine

rospy.init_node('flight')
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

dst = []
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

def findAngle(src,dst):
    src[0] = math.radians(src[0])
    src[1] = math.radians(src[1])
    dst[0] = math.radians(dst[0])
    dst[1] = math.radians(dst[1])
    dLon = dst[1] - src[1]
    y = math.sin(dLon) * math.cos(dst[0])
    x = math.cos(src[0]) * math.sin(dst[0]) - math.sin(src[0]) * math.cos(dst[0]) * math.cos(dLon)
    brng = math.atan2(y,x)
    brng = math.pi/2-brng
    dist = haversine(src,dst)*1000
    return brng, dist

def change_yaw():
    src= [get_telemetry().lat, get_telemetry().lon]
    yaw,dist = findAngle(src,dst)
    set_position(x=0, y=0, z=0, frame_id='map', yaw=float(yaw))   
    if dist < 1:
        return True
    else:
        return False

# Take off and hover 1 m above the ground
navigate_wait(x=0, y=0, z=1.7, frame_id='body', auto_arm=True)

# Wait for 3 seconds
rospy.sleep(3)
check = 1
while True:
    if change_yaw():
        land_wait()
        break
    f = open('object.txt','r')
    control = f.read()
    f.close()
    #print(control)
    if(control=="right"):
        set_position(x=0, y=0.5, z=0, frame_id='body', yaw=float('nan'), yaw_rate=0)
        print("turn left")
    elif(control=="left"):
        set_position(x=0, y=-0.5, z=0, frame_id='body', yaw=float('nan'), yaw_rate=0)
        print("turn right")
    elif(control=="safe"):
        set_position(x=0.5, y=0, z=0, frame_id='body', yaw=float('nan'), yaw_rate=0)
        print("flying forward")

