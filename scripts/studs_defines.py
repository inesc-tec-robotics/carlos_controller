#!/usr/bin/env python

MODULE_NAME  = "studs_dist"
SET_MODE_SRV = "/carlos/vision/set_mode"
GET_MODE_SRV = "/carlos/vision/get_mode"

CRL_STUDS_POS_MSG = "/carlos/vision/studs_pose"
CRL_WALL_ORIENT_MSG = "/carlos/vision/wall_orientation"

''' Params '''
STUDS_PATTERN_DIST = "/studs_pattern/distance"
STUDS_PATTERN_PROX = "/studs_pattern/proximity"
STUDS_PATTERN      = "/studs_pattern/studs"
STUDS_PATTERN_LASER_THRESHOLD = "/studs_pattern/laser_threshold"

WMODE_STOPPED = 0
WMODE_DETECT  = 1
WMODE_TRACK   = 2
