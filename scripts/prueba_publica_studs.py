#!/usr/bin/env python

from amn_msgs.srv import *
from studs_defines import *
import rospy
import carlos_vision2 as crlv
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
#import geometry_msgs.msg
#import std_msgs.msg

# incomming handlers
def set_mode_hnd(req):
    print 'Set mode requested: '+str(req.mode)
    global working_mode
    working_mode = req.mode
    return SetModeResponse(req.mode)

def get_mode_hnd(req):
    print 'Mode requested'
    global working_mode
    return GetModeResponse(working_mode)
    

# publisher
def publica(dato):
    global working_mode

    pub = rospy.Publisher(CRL_STUDS_POS_MSG, PoseArray, queue_size=10)
    studs = PoseArray()
    studs.header.stamp = rospy.Time.now()
    studs.header.frame_id = "/base_link"

    for i in range(len(dato)):
        studi = Pose()
        studi.position.x = dato[i][0][0]
        studi.position.y = dato[i][0][1]
        studi.position.z = dato[i][0][2]

        studi.orientation.x = dato[i][1][0]
        studi.orientation.y = dato[i][1][1]
        studi.orientation.z = dato[i][1][2]
        studi.orientation.w = dato[i][1][3]

        studs.poses.append(studi)

    pub.publish(studs)



def euler_a_cuaternio(roll, pitch, yaw):
    
    import tf
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw, 'rxyz')

    return quaternion 


# timer callback for state machine update
def timer_callback(event):
    # [+arriba -abajo, -izq +der, +adelante -atras]
    puntos = [[0 for x in xrange(5)] for x in xrange(5)] 
    #ph=[-1.5708 1.4 2.2708 0 0.7 0]

    puntos[0]=[[  0.0, -0.15,  0.55 ],   [0.5,-0.5,0.5,0.5]]
    puntos[1]=[[  0.0,  0.15,  0.55 ],   [0.5,-0.5,0.5,0.5]]
    puntos[2]=[[ -0.2,  0.00,  0.55 ],   [0.5,-0.5,0.5,0.5]]
    puntos[3]=[[ -0.4, -0.15,  0.55 ],   [0.5,-0.5,0.5,0.5]]
    puntos[4]=[[ -0.4,  0.15,  0.55 ],   [0.5,-0.5,0.5,0.5]]
 
    
    publica(puntos)
    print 'Puntos = '
    for i in range(len(puntos)):
        print '   P'+str(i)+': '+str(puntos[i][0])


def install_params():

#    global pattern
#    pattern = rospy.get_param(STUDS_PATTERN)
#    global stud_dist
#    stud_dist = rospy.get_param(STUDS_PATTERN_DIST)
#    global stud_prox
#    stud_prox = rospy.get_param(STUDS_PATTERN_PROX)
    return

def init_server():
    rospy.init_node(MODULE_NAME)
    install_params()
    global working_mode
    working_mode = WMODE_STOPPED
    s = rospy.Service(SET_MODE_SRV, SetMode, set_mode_hnd)
    rospy.Timer(rospy.Duration(1), timer_callback)
    rospy.spin()

if __name__ == "__main__":
    init_server()
