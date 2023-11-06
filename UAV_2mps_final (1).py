#! /usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped, Point, Vector3
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, CommandTOL, CommandTOLRequest, SetMode, SetModeRequest, CommandLong, CommandLongRequest

current_state = State()  # First startup drone in POSITION MODE
kill_flag = False

class Services():

    def state_cb(msg):
        global current_state
        current_state = msg

    def __init__(self):   
        self.local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.current_state = State()

    def arm(self):
        rospy.wait_for_service("/mavros/cmd/arming")
        try:
            arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
            arming_client(True)
        except rospy.ServiceException:
            print("Service for arming call failed")

    def disarm(self):
        rospy.wait_for_service("/mavros/cmd/arming")
        try:
            arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
            arming_client(False)
        except rospy.ServiceException:
            print("Service for disarming call failed")

    def offboard(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode) 
            set_mode_client(custom_mode='OFFBOARD')
        except rospy.ServiceException:
            print ("Service set_mode call failed: Offboard Mode could not be set.")

    def takeoff(self, height=1):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
            takeoffService(altitude = height)
        except rospy.ServiceException:
            print ("takeoff call failed")

    def land(self):
        global kill_flag
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            landingService = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
            landingService(altitude = 0)
            kill_flag=True
        except rospy.ServiceException:
            print ("Service land call failed")

    def kill(self):
        global kill_flag
        rospy.wait_for_service('/mavros/cmd/command')
        try:
            kill_client = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
            kill_client(command = 185, param1 = 1.0, param2 = 0.0)
            kill_flag = True
        except rospy.ServiceException:
            print("Kill service call failed")


# Callback function to save the current state/mode, check connection and arm offboard etc
def state_cb(msg):
    global current_state
    current_state = msg

def create_pose(x, y, z):  #creating position points for publishing, topic: local_position
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    return pose

def create_msg(x ,y, z, Vx, Vy, Vz):    #Creating PositionTarget msg for publishing, topic: setpoint_raw
    raw_msg = PositionTarget()
    raw_msg.header.stamp = rospy.Time.now()
    raw_msg.coordinate_frame = 1
    raw_msg.type_mask  = 64 | 128 | 256 | 1024 | 2048
    raw_msg.position.x = x
    raw_msg.position.y = y
    raw_msg.position.z = z
    raw_msg.velocity.x = Vx
    raw_msg.velocity.y = Vy
    raw_msg.velocity.z = Vz
    
    return raw_msg

def track(x ,y, z, Vx, Vy):    #Creating PositionTarget msg for publishing, topic: setpoint_raw
    raw_msg = PositionTarget()
    raw_msg.header.stamp = rospy.Time.now()
    raw_msg.coordinate_frame = 1
    raw_msg.type_mask  = 32 | 64 | 128 | 256 | 1024 | 2048
    raw_msg.position.x = x
    raw_msg.position.y = y
    raw_msg.position.z = z
    raw_msg.velocity.x = Vx
    raw_msg.velocity.y = Vy
    # raw_msg.velocity.z = Vz
    
    return raw_msg

def slowdown(x ,y, z, Vx, Vy, Vz):    #Creating PositionTarget msg for publishing, topic: setpoint_raw
    raw_msg = PositionTarget()
    raw_msg.header.stamp = rospy.Time.now()
    raw_msg.coordinate_frame = 1
    raw_msg.type_mask  = 64 | 128 | 256 | 1024 | 2048
    raw_msg.position.x = x
    raw_msg.position.y = y
    raw_msg.position.z = z
    raw_msg.velocity.x = Vx
    raw_msg.velocity.y = Vy
    raw_msg.velocity.z = Vz
    
    return raw_msg

def descend(x ,y, z, Vx, Vy):    #Creating PositionTarget msg for publishing, topic: setpoint_raw
    raw_msg = PositionTarget()
    raw_msg.header.stamp = rospy.Time.now()
    raw_msg.coordinate_frame = 1
    raw_msg.type_mask  = 32 | 64 | 128 | 256 | 1024 | 2048
    raw_msg.position.x = x
    raw_msg.position.y = y
    raw_msg.position.z = z
    raw_msg.velocity.x = Vx
    raw_msg.velocity.y = Vy
    # raw_msg.velocity.z = Vz
    
    return raw_msg

def hovering(x ,y, z, Vx, Vy, Vz):    #Creating PositionTarget msg for publishing, topic: setpoint_raw
    raw_msg = PositionTarget()
    raw_msg.header.stamp = rospy.Time.now()
    raw_msg.coordinate_frame = 1
    raw_msg.type_mask  = 64 | 128 | 256 | 1024 | 2048
    raw_msg.position.x = x
    raw_msg.position.y = y
    raw_msg.position.z = z
    raw_msg.velocity.x = Vx
    raw_msg.velocity.y = Vy
    raw_msg.velocity.z = Vz
    
    return raw_msg

def create_current(fcu_data):   #current local pose from FCU
    global curr_pose
    curr_pose = fcu_data
    return curr_pose

def create_raw_current(fcu_raw_data):       #current RAW local, might not be from FCU??
    global curr_raw_pose
    curr_raw_pose = fcu_raw_data
    return curr_raw_pose

def main():
    service = Services()    #For sending commands to drone, refer to class Services()

    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)
    #Subscribing current state, which is position mode?

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    # Publisher to publish commanded local position

    local_raw_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size = 10)
    #Publisher to publish commanded setpoint_RAW position, with local position, velocity and acceleration setpoint

    local_raw_sub = rospy.Subscriber("mavros/setpoint_raw/local", PositionTarget, callback = create_raw_current)
    #Subscribe to local RAW position from FCU

    current_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=create_current)
    # Subscriber to subscribe local position from FCU

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # Setpoint publishing MUST be faster than 2Hz, PX4 has 500ms timeout btwn 2 Offboard cmd
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection, connection needed before publishing anything
    while (not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    wp = (5,5,0)  # Waypoints (x, y, z), for the sake of entering offboard. wp defined later
    speed = (0.1,0.1,0.1)

    # Send a few setpoints before starting, entering offboard mode requires streaming setpoints else mode switch is rejected
    for i in range(100):
        if (rospy.is_shutdown()):
            break

        raw = create_msg(wp[0], wp[1], wp[2], speed[0], speed[1], speed[2])
        local_raw_pub.publish(raw)
        
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    last_req = rospy.Time.now()

    timestep_counter = int(0)                   #Initialize start time, counter represent each hertz. Time = counter / rospy rate
    fly_rate = 20                               #follow rospy.rate() hertz, in this case 20hz. each sec equivalent of 20 hz

    while (not rospy.is_shutdown()):
        if (current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if (set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if (not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                service.arm()
                if (current_state.armed): 
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        wp_i = (10,10,0)                                #Start pt of AGV
        t_elapsed = timestep_counter/fly_rate
        Vg_x = 0                                    #AGV STATIONARY
        Vg_y = 0
        Vt_x = Vg_x * (t_elapsed)                   #Each Vt component for AGV in x = x_0 + (V_x * t_elapsed)
        Vt_y = Vg_y * (t_elapsed)                   #mental note: wp[0],wp[1],wp[2] is start point of AGV
        Vt_z = 0.0                                  # No height change for AGV
        
        #Base Velocity of UAV
        Vd_x = 1
        Vd_y = 1
        Vd_z = 1

        wp = [wp_i[0]+Vt_x, wp_i[1]+Vt_y, wp_i[2]]        #Position of AGV aft time t, also known as displacement

        show_pos = False                             #Change to False to not show position info

        if show_pos == True:
            print("AGV location: %f", wp)
            print("UAV's X: %f", curr_pose.pose.position.x)
            print("UAV's Y: %f", curr_pose.pose.position.y)
            print("UAV's Z: %f", curr_pose.pose.position.z)
            print("X difference: %f", wp[0] - curr_pose.pose.position.x)
            print("Y difference: %f", wp[1] - curr_pose.pose.position.y)

        if (t_elapsed <= 20):
            raw = create_msg(0,0,2,0,0,1)                       #takeoff

        elif (abs(wp[0] - curr_pose.pose.position.x) >= (wp_i[0] *0.7)) and (abs(wp[1] - curr_pose.pose.position.y) >= (wp_i[1] *0.7)):
            raw = track(wp[0], wp[1], 2, Vd_x*2, Vd_y*2)        #Drone target height z = 2
            print(1)

        elif (abs(wp[0] - curr_pose.pose.position.x) <= 0.1) and (abs(wp[1] - curr_pose.pose.position.y) <= 0.1) and (abs(wp[2] - curr_pose.pose.position.z) <= 0.2):
            raw = hovering(wp[0], wp[1], 0.08, 0, 0, -0.1)       #Drone Vz = 0
            if kill_flag== False:
                local_raw_pub.publish(raw)

            print(2)
            if abs(wp[2] - curr_pose.pose.position.z) <= 0.09:
                service.kill()
                final_pose = False
                service.land()
                while (curr_pose.pose.position.z != 0.0):
                    if (curr_pose.pose.position.z <= 0.0) and final_pose == False:
                        rospy.loginfo("Vehicle landed")
                        print("Time elasped:",'%.2f', t_elapsed)
                        print("X difference: %f", wp[0] - curr_pose.pose.position.x)
                        print("Y difference: %f", wp[1] - curr_pose.pose.position.y)
                        final_pose = True
                    service.disarm()

        elif (abs(wp[0] - curr_pose.pose.position.x) <= 1.25) and (abs(wp[1] - curr_pose.pose.position.y) <= 1.25):
            raw = descend(wp[0], wp[1], 0.1, Vg_x*1.1, Vg_y*1.1)        
            print(4)

        elif (abs(wp[0]- curr_pose.pose.position.x) < 3) and (abs(wp[1] - curr_pose.pose.position.y) < 3):
            raw = slowdown(wp[0], wp[1], 2, Vg_x*1.2, Vg_y*1.2, 0.0)          #Drone Vz = 0.0
            print(5)

        elif (abs(wp[0] - curr_pose.pose.position.x) < (0.4*wp_i[0])) and (abs(wp[1] - curr_pose.pose.position.y) < (0.4*wp_i[0])):
            raw = slowdown(wp[0], wp[1], 2, Vg_x*1.25, Vg_y*1.25, 0.0)          #Drone Vz = 0.0
            print(6)

        elif (abs(wp[0] - curr_pose.pose.position.x) < (0.65*wp_i[0])) and (abs(wp[1] - curr_pose.pose.position.y) < (0.65*wp_i[0])):
            raw = slowdown(wp[0], wp[1], 2, Vg_x*1.5, Vg_y*1.5, 0.0)          #Drone Vz = 0.0
            print(7)

        if kill_flag == False:
            local_raw_pub.publish(raw)
            timestep_counter +=1 
                   
        rate.sleep()

if __name__ == "__main__":

    main()
    




        