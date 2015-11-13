#!/usr/bin/env python

from mission_ctrl_msgs.srv import *
from studs_defines import *
import rospy
import time
import carlos_vision as crlv
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from carlos_controller.msg import StudsPoses
#import geometry_msgs.msg
#import std_msgs.msg

# incomming handlers
def set_mode_hnd(req):
    print 'Set mode requested: '+str(req.mode)
    global working_mode
    working_mode = req.mode
#    if working_mode != WMODE_STOPPED:
#      for i in range(12):
#        puntos, stiff1, stiff2, tipo= crlv.calcula_dist(working_mode)
#        time.sleep(0.3)
    return SetModeResponse(req.mode)

def get_mode_hnd(req):
    print 'Mode requested'
    global working_mode
    return GetModeResponse(working_mode)
    

# publisher
def publica(dato, stiff1, stiff2, tipo):
    global working_mode
    global studs_pub
    global wall_pub
    #pub = rospy.Publisher(CRL_STUDS_POS_MSG, StudsPoses, queue_size=1)
    studs = PoseArray()
    mes = StudsPoses()
    studs.header.stamp = rospy.Time.now()

    if tipo==WMODE_TRACK:
        #studs.header.frame_id = "orientation"
        #studi = Pose()
	if dato[0][0][0]!=0 or dato[0][0][1]!=0 or dato[0][0][2]!=0:
          wall_or = PointStamped()

          wall_or.point.x = dato[0][0][0]+0.0
          wall_or.point.y = dato[0][0][1]-0.035
          wall_or.point.z = dato[0][0][2]-0.015
 
          wall_pub.publish(wall_or)


    elif tipo==WMODE_STOPPED:
        studs.header.frame_id = "idle"
    	studi = Pose()
        studi.position.x = 0
        studi.position.y = 0
        studi.position.z = 0

        studi.orientation.x = 0
        studi.orientation.y = 0
        studi.orientation.z = 0
        studi.orientation.w = 0

        studs.poses.append(studi)

    else:
        studs.header.frame_id = "/base_link"


        for i in range(len(dato)):
            studi = Pose()
            studi.position.y = (dato[i][0][0])/1000.0
            studi.position.x = -(dato[i][0][1]-110)/1000.0
            studi.position.z = (dato[i][0][2]-175)/1000.0

            studi.orientation.x = dato[i][1][0]
            studi.orientation.y = dato[i][1][1]
            studi.orientation.z = dato[i][1][2]
            studi.orientation.w = dato[i][1][3]

            studs.poses.append(studi)

        mes.pose_array=studs
        mes.stiff1=stiff1/1000.0
        mes.stiff2=stiff2/1000.0
	if dato[0][0][0]!=0 or dato[0][0][1]!=0 or dato[0][0][2]!=0:
          studs_pub.publish(mes)



# timer callback for state machine update
def timer_callback(event):
    global working_mode
    global pattern
    global init_time
    global laser_threshold



    if working_mode==WMODE_DETECT:
	pattern = rospy.get_param(STUDS_PATTERN)
        laser_threshold=rospy.get_param(STUDS_PATTERN_LASER_THRESHOLD)
        crlv.cambia_patron(pattern, stud_margin, stud_prox)
        puntos, stiff1, stiff2, tipo= crlv.calcula_dist(working_mode, laser_threshold)

        print 'Detecting...'
        print 'Studs detected: '+ str(len(puntos))
        for i in range(len(puntos)):
            print 'Stud '+str(i)+': '+  str(puntos[i][0])
        print 'Stiff. 1: ' +str(stiff1) + ' Stiff. 2: ' +str(stiff2) + ' SD:'  + str(stiff2-stiff1)
        if puntos[0][1][0]!=2 and tipo==working_mode:
            publica(puntos, stiff1, stiff2, tipo)
        else:
            publica(puntos, stiff1, stiff2, WMODE_STOPPED)

    elif working_mode==WMODE_TRACK:
	pattern = rospy.get_param(STUDS_PATTERN)
        laser_threshold=rospy.get_param(STUDS_PATTERN_LASER_THRESHOLD)
        crlv.cambia_patron(pattern, stud_margin, stud_prox)
        puntos, stiff1, stiff2, tipo= crlv.calcula_dist(working_mode, laser_threshold)

        print 'Tracking...'
        print 'Orientation = ' + str(puntos[0][0])
        print 'Stiff. 1: ' +str(stiff1) + ' Stiff. 2: ' +str(stiff2) + ' SD:'  + str(stiff2-stiff1)
        if puntos[0][1][0]!=2 and tipo==working_mode:
            publica(puntos, stiff1, stiff2, tipo)
        else:
            publica(puntos, stiff1, stiff2, WMODE_STOPPED)
    else:
       print 'Idle ' + str(rospy.Time.to_sec(rospy.Time.now())-rospy.Time.to_sec(init_time))
	
def install_params():
    global pattern
    pattern = rospy.get_param(STUDS_PATTERN)
    global stud_margin
    stud_margin = rospy.get_param(STUDS_PATTERN_DIST)
    global stud_prox
    stud_prox = rospy.get_param(STUDS_PATTERN_PROX)
    global laser_threshold
    laser_threshold = rospy.get_param(STUDS_PATTERN_LASER_THRESHOLD)

def init_server():
    global init_time
    global thres_laser

    thres_laser=220
    rospy.init_node(MODULE_NAME)
    crlv.arranca()
    install_params()
    global working_mode
    working_mode = WMODE_STOPPED
    #working_mode = WMODE_DETECT
    #working_mode = WMODE_TRACK
    global studs_pub
    global wall_pub
    wall_pub = rospy.Publisher(CRL_WALL_ORIENT_MSG, PointStamped, queue_size=1)
    studs_pub = rospy.Publisher(CRL_STUDS_POS_MSG, StudsPoses, queue_size=1)
    s = rospy.Service(SET_MODE_SRV, SetMode, set_mode_hnd)
    init_time=rospy.Time.now()
    rospy.Timer(rospy.Duration(0.25), timer_callback)
    rospy.spin()


if __name__ == "__main__":
    init_server()
