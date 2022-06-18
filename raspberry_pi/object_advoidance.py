import rospy
from clover import srv
import math
from std_srvs.srv import Trigger

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)


def navigate_wait(x=0, y=0, z=0, yaw=float('nan'),yaw_rate=0, speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, yaw_rate=yaw_rate, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)
# Take off and hover 1 m above the ground
navigate_wait(x=0, y=0, z=1.7, frame_id='body', auto_arm=True)

# Wait for 3 seconds
rospy.sleep(3)
check = 1
while True:
    f = open('object.txt','r')
    control = f.read()
    #print(control)
    if(control=="right"):
        set_velocity(vy=0.5,frame_id='body', auto_arm=True)
        print("right")
        rospy.sleep(0.2)
    elif(control=="left"):
        set_velocity(vy=-0.5,frame_id='body', auto_arm=True)
        print("left")
        rospy.sleep(0.2)
    elif(control=="safe"):
        set_velocity(vx=0.5,frame_id='body', auto_arm=True)
        print("safe")
        rospy.sleep(0.2)
    #f.close()
    #rospy.sleep(1)
    check +=1
    if check == 50:
        land_wait()
        break


