#!/usr/bin/env python

# - get map data from share
# - get path planner(call service)
# logic for clean
#
# save cleaned path in map
#    - save myMap
#    - get cleaned path
#    - update myMap


import sys
sys.path.append("/home/slam/sweeper/src/sweeper-bringup")
sys.path.append("/home/slam/sweeper/src/sweeper-move-mynt")
import math
import numpy as np
import copy

import rospy
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Vector3, PointStamped
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import LaserScan
from nav_msgs.srv import GetPlan

from scale_map.srv import ScaleMapData
from scale_map.srv import GetCoveragePath  

from sweeper_bringup.scripts.lib.bringupRequestPos import createPoseWithRPY
import sweeper_bringup.scripts.common.bringupShare as share
import sweeper_bringup.scripts.common.bringupConf as conf
from sweeper_move_mynt.scripts.action.gotoPoint import gotoPoint
from sweeper_move_mynt.scripts.action.gotoPoint import gotoPointWithQuaternion
from sweeper_move_mynt.scripts.action.gotoPoint import gotoPointAndStartWithRotate
from sweeper_bringup.scripts.lib.logwrite import logWrite
from sweeper_bringup.scripts.lib.loopmonitor import loopStart, loopEnd
from sweeper_bringup.scripts.lib.bringupRequestPos import createPointStamped
from sweeper_bringup.scripts.lib.set_param import reload_smoother_param

moudleName = share.moudleName = "clean_strategy"


class pathPlanner:
  # Erosion radius,(float64)
  # unit: meter
  erosionRadius = 0.01 #0.31  # 0.02 0.01 0.1 0.02 0.1 0.15 0.08 0.2 0.4 0.25 0.22 0.21 0.05 0.02

  # Robot radius, (float64 )
  # unit: meter
  robotRadius = 0.04  # 0.2 0.1 0.04 0.1 0.115 0.15 0.12 0.06

  # Occupancy threshold, (int8)
  # range: 0 ~ 100, if > threshold, then flag it Occupancied
  occupancyThreshold = 10

  # The start pose for the plan, (geometry_msgs/PoseStamped)
  start = None

  # The final pose of the goal position,(geometry_msgs/PoseStamped)
  goal = None


class mapModifierThreshold:
  holeThreshold = 100
  noiseThreshold = 10
  debug = 0


def getCleanPathPlanner():
  if not share.receivedNewMap:  # for mapData
    logMsg = "share.receivedNewMap = False in getCleanPathPlanner()"
    logWrite('warning',moudleName,logMsg)
    return []

  if not share.receivedInitPos:  # for start param
    logMsg = "share.receivedInitPos = False in getCleanPathPlanner()"
    logWrite('warning',moudleName,logMsg)
    return []


  mapInput = mergeMapData(share.globalMapData,share.mapData)

  #####################
  ## (nav_msgs/OccupancyGrid)
  ## mapDataPre = share.mapData
  ##mapInput = share.mapData
  #mapInput = share.globalMapData
  ##mapDataPre = mapInput

  #####################
  ### service MapModifire
  ##try:
  ##  mapModifierResponse = share.mapModifier(mapModifierThreshold.holeThreshold, mapModifierThreshold.noiseThreshold, mapModifierThreshold.debug, mapDataPre)
  ##  share.MapDataModified  = mapData =  mapModifierResponse.map

  ##except rospy.ServiceException, e:
  ##  #mapData = share.mapData
  ##  mapData = mapInput
  ##  logMsg = "call mapModifier service fail. %s"%e
  ##  logWrite('warning',moudleName,logMsg)
  #####################



  ##############################
  if share.topLeft == None:
        share.topLeft = createPointStamped(Point(-2,2,0),"map");
  if share.bottomLeft == None:
        share.bottomLeft = createPointStamped(Point(-2,-2,0),"map");
  if share.bottomRight == None:
        share.bottomRight = createPointStamped(Point(2,-2,0),"map");
  if share.topRight == None:
        share.topRight = createPointStamped(Point(2,2,0),"map");

  logMsg = "====3"
  logWrite('debug',share.moudleName,logMsg)
  mapResolution = share.mapData.info.resolution
  mapOrigin = (share.mapData.info.origin.position.x, share.mapData.info.origin.position.y)
  mapWidth = mapInput.info.width


  logMsg = "====6 mapResolution:%s mapOriginX:%s mapOriginY:%s mapWidth:%s"%(mapResolution,mapOrigin[0],mapOrigin[1],mapWidth)
  logWrite('debug',share.moudleName,logMsg)

  mapData = updateMapWithborder(share.topLeft,share.bottomLeft,share.bottomRight,share.topRight,mapInput,mapResolution,mapOrigin,mapWidth)
  logMsg = "====10 len(mapData.data):%s"%(len(mapData.data))
  logWrite('debug',share.moudleName,logMsg)
  share.MapDataModified = mapData
  #############################
  ####################
  ##mapInput = share.mapData



  #mapInput = share.globalMapData
  #mapData = mapInput  # to invalid mapModifier
  #share.MapDataModified = mapData

  # service cleanPlanner
  pathPlanner.start = share.currentPose
  pathPlanner.start.header.frame_id = mapData.header.frame_id
  pathPlanner.goal = goal = pathPlanner.start

  logMsg = "====4"
  logWrite('debug',share.moudleName,logMsg)
  plan = []
  try:
    cleanPlannerResponse = share.cleanPlanner(
        pathPlanner.start,
        pathPlanner.goal,
        pathPlanner.erosionRadius,
        pathPlanner.robotRadius,
        pathPlanner.occupancyThreshold,
        mapData)

    # (nav_msgs/Path)
    plan = cleanPlannerResponse.plan
  except rospy.ServiceException, e:
    #mapData = share.mapData
    mapData = mapInput
    logMsg = "call cleanPlanner service fail.%s"%e
    logWrite('warning',moudleName,logMsg)

  return plan  # (nav_msgs/Path)



def getCleanPathPlannerWithMapData(mapData,startPose,endPose):

  mapInput = mapData

  # service cleanPlanner
  pathPlanner.start = startPose
  pathPlanner.start.header.frame_id = mapData.header.frame_id
  pathPlanner.goal = goal = endPose

  plan = []
  try:
    cleanPlannerResponse = share.cleanPlanner(
        pathPlanner.start,
        pathPlanner.goal,
        pathPlanner.erosionRadius,
        pathPlanner.robotRadius,
        pathPlanner.occupancyThreshold,
        mapData)

    # (nav_msgs/Path)
    plan = cleanPlannerResponse.plan
  except rospy.ServiceException, e:
    #mapData = share.mapData
    mapData = mapInput
    #logMsg = "call cleanPlanner service fail.%s"%e
    #logWrite('warning',moudleName,logMsg)

  return plan  # (nav_msgs/Path)

def getCleanPathPlannerWithMapDataScaleMap(mapData,startPose,endPose):
    #service scale map
    rospy.wait_for_service('/sweeper/scale_map_srv')

    plan = []
    try:
        scale_erosion_radius = 1
        scale_robot_radius = 0.3

        scale_map_data = rospy.ServiceProxy('/sweeper/scale_map_srv',ScaleMapData)
        res_scale_map = scale_map_data(scale_erosion_radius,scale_robot_radius,mapData)
    except rospy.ServiceException, e:
        rospy.loginfo("scale_map_srv error. e:%s",e)
        pass

    rospy.wait_for_service('/sweeper/make_coverage_plan')
    try:
        
        darp_map_id = 0
        darp_start_pose = startPose
        darp_erosion_radius = 0.01
        darp_robot_radius = 0.03 
        darp_occupancy_threshold = 95
        
        make_coverage_path_plan = rospy.ServiceProxy('/sweeper/make_coverage_plan',GetCoveragePath)
        res_make_coverage_plan = make_coverage_path_plan(darp_map_id,
                                                                                                            darp_start_pose,
                                                                                                            darp_erosion_radius,
                                                                                                            darp_robot_radius,
                                                                                                            darp_occupancy_threshold,
                                                                                                            res_scale_map.map)
        plan = res_make_coverage_plan.plan
    except rospy.ServiceException, e:
        rospy.loginfo("make_coverage_plan error. e:%s",e)
        pass

    return plan

        

def getCleanPoseListWithMapData(mapData,startPose,endPose):

    plan = []
    try:
        plan = getCleanPathPlannerWithMapData(mapData,startPose,endPose)

        return plan.poses
    except AttributeError, e:
        #logMsg = "get plan failed. plan:%s startPose:(%s,%s) e:%s"%(plan, startPose.pose.position.x, startPose.pose.position.y, e)
        #logWrite('warning',moudleName,logMsg)
        return [startPose]


def getCleanPoseList():

    plan = []
    try:
        plan = getCleanPathPlanner()
        return plan.poses
    except AttributeError, e:
        logMsg = "get plan failed. plan:%s"%(plan)
        logWrite('warning',moudleName,logMsg)
        return []


def createMarker(
        markerName,
        markerType,
        pose,
        scale,
        color,
        markerId,
        frameId=""):

  if len(frameId) == 0:
    frameId = conf.FRAME_ID_MAP

  marker = Marker()
  marker.header.frame_id = frameId
  marker.header.stamp = rospy.Time.now()
  marker.ns = "marker_" + markerName
  marker.id = markerId
  marker.type = markerType
  marker.action = Marker.ADD
  marker.pose.position.x = pose.position.x
  marker.pose.position.y = pose.position.y
  marker.pose.position.z = pose.position.z
  marker.pose.orientation.x = pose.orientation.x
  marker.pose.orientation.y = pose.orientation.y
  marker.pose.orientation.z = pose.orientation.z
  marker.pose.orientation.w = pose.orientation.w
  marker.scale.x = scale.x
  marker.scale.y = scale.y
  marker.scale.z = scale.z
  marker.color.a = color.a
  marker.color.r = color.r
  marker.color.g = color.g
  marker.color.b = color.b

  return marker





def pubEmptyCleanPlanner(poseList):


  markerArray = MarkerArray()

  ## marker goal pose
  ###########################
  #pose = Pose()
  #pose.position.x = pathPlanner.goal.pose.position.x
  #pose.position.y = pathPlanner.goal.pose.position.y
  #pose.orientation.w = 1.0

  #scale = Vector3(0.2, 0.2, 0.2)

  #color = ColorRGBA(0, 0, 1, 1)  # r,g,b,a

  #markerId = 0

  #markerStart = createMarker(
  #    "plannerStart",
  #    Marker.SPHERE,
  #    pose,
  #    scale,
  #    color,
  #    markerId)
  #markerArray.markers.append(markerStart)
  ###########################

  share.cleanPlannerPub.publish(markerArray)


def pubCleanPlanner(poseList):
    pubCleanPlannerOnce(poseList, share.cleanPlannerPub)

    if len(poseList) ==0:
        logMsg = "poseList empty."
        logWrite('warning',moudleName,logMsg)
        return False

    logMsg = "pubCleanPlannerOnce. len(poseList):%s"%(len(poseList))
    logWrite('warning',moudleName,logMsg)


    pubCleanPlannerOnce(poseList, share.cleanPlannerPub)

    share.cleanPostList = share.cleanPostList + poseList
    pubCleanPlannerOnce(share.cleanPostList, share.cleanPlannerAllPub)


def pubCleanPlannerOnce(poseList, cleanPlannerPub):

  logMsg = "pubCleanPlannerOnce. len(poseList):%s"%(len(poseList))
  logWrite('warning',moudleName,logMsg)

  markerArray = MarkerArray()

  # marker start pose
  ##########################
  pose = Pose()
  pose.position.x = pathPlanner.start.pose.position.x
  pose.position.y = pathPlanner.start.pose.position.y
  pose.orientation.w = 1.0

  scale = Vector3(0.2, 0.2, 0.2)

  color = ColorRGBA(0, 1, 0, 1)  # r,g,b,a

  markerId = 0

  markerStart = createMarker(
      "plannerStart",
      Marker.SPHERE,
      pose,
      scale,
      color,
      markerId)
  markerArray.markers.append(markerStart)
  ##########################

  #cleanPlanner = Marker()
  #cleanPlanner.header.frame_id = "s_map"
  #cleanPlanner.header.stamp = rospy.Time.now()
  #cleanPlanner.ns = 'cleanPlanner'
  #cleanPlanner.action = Marker.ADD
  #cleanPlanner.pose.orientation.w = 1.0;
  #cleanPlanner.id = 0
  #cleanPlanner.type = Marker.LINE_STRIP
  #cleanPlanner.scale.x = 0.02
  #cleanPlanner.scale.y = 0.1
  #cleanPlanner.scale.z = 0.1
  # cleanPlanner.color.r = 1.0 #red
  # cleanPlanner.color.g = 0.0 #green
  # cleanPlanner.color.b = 0.0 #blue
  #cleanPlanner.color.a = 1.0

  lastPose = poseList[0]
  count = 0
  lastPoint = Point(lastPose.pose.position.x, lastPose.pose.position.y, 0)

  loopStart("for_pubCleanPlanner")
  for pose in poseList:
    count = count + 1

    # marker planner pose
    ##########################
    scale = Vector3(0.05, 0.1, 0.1)

    color = ColorRGBA(1, 0, 0, 1)  # r,g,b,a

    poseTmp = Pose()
    poseTmp.position.x = lastPoint.x
    poseTmp.position.y = lastPoint.y
    # z must 0, if do not this, the line will can not see in map, because it
    # may be above the map or below the map.
    poseTmp.position.z = 0

    poseTmp.orientation.w = 1.0
    markerId = count

    markerArrow = createMarker(
        "plannerPath",
        Marker.ARROW,
        poseTmp,
        scale,
        color,
        markerId)

    #arrowHead, arrowEnd
    ##########
    arrowHeadPoint = Point(0, 0, 0)

    p = Point(pose.pose.position.x, pose.pose.position.y, 0)

    arrowEndPoint = Point()
    arrowEndPoint.x = p.x - lastPoint.x
    arrowEndPoint.y = p.y - lastPoint.y
    arrowEndPoint.z = 0


    markerArrow.points.append(arrowHeadPoint)
    markerArrow.points.append(arrowEndPoint)
    ##########
    ##########################

    markerArray.markers.append(markerArrow)

    lastPoint = p

  loopEnd("for_pubCleanPlanner")

  # marker goal pose
  ##########################
  pose = Pose()
  pose.position.x = pathPlanner.goal.pose.position.x
  pose.position.y = pathPlanner.goal.pose.position.y
  pose.orientation.w = 1.0

  scale = Vector3(0.2, 0.2, 0.2)

  color = ColorRGBA(0, 0, 1, 1)  # r,g,b,a

  markerId = 0

  markerStart = createMarker(
      "plannerStart",
      Marker.SPHERE,
      pose,
      scale,
      color,
      markerId)
  markerArray.markers.append(markerStart)
  ##########################

  cleanPlannerPub.publish(markerArray)

  #logMsg = "==== cleanPlanner:%s"%(markerArray)
  # logWrite('debug',moudleName,logMsg)


def cleanAction(control):
  poseList = getCleanPoseList()
  pubCleanPlanner(poseList)

  share.reachedEndOfPlan = False
  counter = 0

  loopStart("for_cleanAction")
  for point in poseList:
    if count == 0:
      continue

    if share.stoped:
      break

    share.localGoalX = point.pose.position.x
    share.localGoalY = point.pose.position.y
    share.wasLocalGoalDefined = True
    #control.goToPositionInAStraightLine(conf.LINEAR_VELOCITY, share.localGoalX, share.localGoalY)
    cleanMoveTo(
        control,
        share.currentX,
        share.currentY,
        share.localGoalX,
        share.localGoalY)

    if share.bumperEncountered:
      logMsg = "share.bumperEncountered encountered in cleanAction."
      logWrite('warning', moudleName, logMsg)
      # return True
      continue

    if share.obstacleEncountered:
      logMsg = "share.obstacleEncountered encountered in cleanAction."
      logWrite('warning', moudleName, logMsg)
      # return True
      continue

    if len(poseList) >= 1 and counter >= (len(poseList) - 1):
      logMsg = "reach the end of poseList in cleanAction."
      logWrite('warning', moudleName, logMsg)
      share.reachedEndOfPlan = True

    counter = counter + 1

  loopEnd("for_cleanAction")

  if len(poseList) == 0:
    share.reachedEndOfPlan = True

  share.wasLocalGoalDefined = False
  return True

def cleanMoveToWithMoveBase(control, fromX, fromY, goalX, goalY, linearVelocity = -1):

  needRetryWaitingMessage = 0
  try:
    logMsg = "Waiting for %s topic..."%conf.TOPIC_OBSTACLE_AVOID_SCAN
    logWrite('debug', moudleName, logMsg)

    rospy.wait_for_message(conf.TOPIC_OBSTACLE_AVOID_SCAN, LaserScan,1.0)
  except rospy.exceptions.ROSException,e:
    logMsg = "Waiting for %s topic exceed time.  so rotate 90 degrees want to change env , then may be can got topic messages."
    logWrite('debug', moudleName, logMsg)

    control.rotateAround90Right()
    needRetryWaitingMessage = 1


  if needRetryWaitingMessage == 1:
      try:
          rospy.loginfo("retry Waiting for %s topic...",conf.TOPIC_OBSTACLE_AVOID_SCAN)
          rospy.wait_for_message(conf.TOPIC_OBSTACLE_AVOID_SCAN, LaserScan,1.0)
      except rospy.exceptions.ROSException,e:
    	  logMsg = "bad env without obstacle_avoid_scan. so do not move to new goal,and want to next clean goal."
    	  logWrite('debug', moudleName, logMsg)
          return False

  if share.nextCleanPosePub:
    nextPoint = createPointStamped(Point(goalX, goalY, 0), conf.FRAME_ID_MAP)
    share.nextCleanPosePub.publish(nextPoint)

  distance = math.sqrt((goalX - fromX)**2 + (goalY - fromY)**2)
  thresholdDistance = pathPlanner.robotRadius * 0.6


  if distance < thresholdDistance:
    logMsg = "distance is too near, so not move to there. distance:%s threshold:%s  from:(%s,%s) goal:(%s,%s) " % (distance, thresholdDistance, fromX, fromY, goalX, goalY)
    logWrite('debug', moudleName, logMsg)
    return True
  else:
    pass


  reload_smoother_param("faster")
  angle = control.rotateToAngleWithDetination(goalX, goalY)

  if linearVelocity  < 0:
      linearVelocity = conf.LINEAR_VELOCITY

  if isShortDistance(distance):
      logMsg = "isShortDistance. distance:%s"%(distance)
      logWrite('debug', moudleName, logMsg)
      if control.goToPositionInAStraightLine(conf.LINEAR_VELOCITY, goalX, goalY):
          return True
      else:
          return False
  else:
      #reload_smoother_param("slower")
      reload_smoother_param("faster")
      quaternion = quaternion_from_euler(0, 0, angle)

      goalState = gotoPointWithQuaternion(goalX, goalY, quaternion)
      if 3 ==  goalState:
          return True
      else:
          return False


def cleanMoveTo(control, fromX, fromY, goalX, goalY, linearVelocity = -1):

  ##for obstacle_avoid check
  # obstacle_avoid is very importent while moving


  needRetryWaitingMessage = 0
  try:
    logMsg = "Waiting for %s topic..."%conf.TOPIC_OBSTACLE_AVOID_SCAN
    logWrite('debug', moudleName, logMsg)

    rospy.wait_for_message(conf.TOPIC_OBSTACLE_AVOID_SCAN, LaserScan,1.0)
  except rospy.exceptions.ROSException,e:
    logMsg = "Waiting for %s topic exceed time.  so rotate 90 degrees want to change env , then may be can got topic messages."
    logWrite('debug', moudleName, logMsg)

    control.rotateAround90Right()
    needRetryWaitingMessage = 1

  if needRetryWaitingMessage == 1:
      try:
        logMsg = "retry Waiting for %s topic..."%conf.TOPIC_OBSTACLE_AVOID_SCAN
        logWrite('debug', moudleName, logMsg)

        rospy.wait_for_message(conf.TOPIC_OBSTACLE_AVOID_SCAN, LaserScan,1.0)
      except rospy.exceptions.ROSException,e:
    	  logMsg = "bad env without obstacle_avoid_scan. so do not move to new goal,and want to next clean goal."
    	  logWrite('debug', moudleName, logMsg)
          return False



  if share.nextCleanPosePub:
    nextPoint = createPointStamped(Point(goalX, goalY, 0), conf.FRAME_ID_MAP)
    share.nextCleanPosePub.publish(nextPoint)

  distance = math.sqrt((goalX - fromX)**2 + (goalY - fromY)**2)
  thresholdDistance = pathPlanner.robotRadius * 0.6
  if distance < thresholdDistance:
    logMsg = "distance is too near, so not move to there. distance:%s threshold:%s  from:(%s,%s) goal:(%s,%s) " % (distance, thresholdDistance, fromX, fromY, goalX, goalY)
    logWrite('debug', moudleName, logMsg)
    return True
  else:
    #logMsg = "distance:%s  from:(%s,%s) goal:(%s,%s) " % (
    #    distance, fromX, fromY, goalX, goalY)
    #logWrite('debug', moudleName, logMsg)
    pass

  ######
  #logMsg = "control.gotoPointAndStartWithRotate(%s, %s) gotoPoint(%s, %s) currentX:%s currentY:%s"%(goalX, goalY, goalX, goalY, share.currentX, share.currentY)
  # logWrite('warning',moudleName,logMsg)
  #gotoPointAndStartWithRotate(goalX, goalY)
  ######

  ########
  ##logMsg = "control.rotateToAngleWithDetination(%s, %s) gotoPoint(%s, %s) currentX:%s currentY:%s" % (
  ##    goalX, goalY, goalX, goalY, share.currentX, share.currentY)
  ##logWrite('debug', moudleName, logMsg)
  #angle = control.rotateToAngleWithDetination(goalX, goalY)
  ## the rotation of the transformation as a tuple (x,y,z,w)
  #quaternion = quaternion_from_euler(0, 0, angle)
  #gotoPointWithQuaternion(goalX, goalY, quaternion)
  ########

  reload_smoother_param("faster")
  angle = control.rotateToAngleWithDetination(goalX, goalY)

  if linearVelocity  < 0:
	linearVelocity = conf.LINEAR_VELOCITY

  if isShortDistance(distance):
      logMsg = "isShortDistance. distance:%s"%(distance)
      logWrite('debug', moudleName, logMsg)
      #control.goToPositionInAStraightLineOnlyBumper(conf.LINEAR_VELOCITY*0.5, goalX, goalY)
      control.goToPositionInAStraightLine(conf.LINEAR_VELOCITY, goalX, goalY)
  else:
      control.goToPositionInAStraightLine(conf.LINEAR_VELOCITY, goalX, goalY)
      #reload_smoother_param("faster")
      #angle = control.rotateToAngleWithDetination(goalX, goalY)
      #reload_smoother_param("slower")
      #quaternion = quaternion_from_euler(0, 0, angle)
      #gotoPointWithQuaternion(goalX, goalY, quaternion)


  #if isShortDistance(distance):
  #    logMsg = "isShortDistance. distance:%s"%(distance)
  #    logWrite('debug', moudleName, logMsg)
  #    #control.goToPositionInAStraightLineOnlyBumper(conf.LINEAR_VELOCITY/2, goalX, goalY)
  #    control.goToPositionInAStraightLine(conf.LINEAR_VELOCITY/1.5, goalX, goalY)
  #else:
  #    control.goToPositionInAStraightLine(conf.LINEAR_VELOCITY, goalX, goalY)

  #logMsg = "cleanMoveTo. distance:%s"%(distance)
  #logWrite('debug', moudleName, logMsg)
  #if isShortDistance(distance):
  #    logMsg = "isShortDistance. distance:%s"%(distance)
  #    logWrite('debug', moudleName, logMsg)
  #    reload_smoother_param("faster")
  #    control.rotateToAngleWithDetination(goalX, goalY)
  #    control.goToPositionInAStraightLineOnlyBumper(conf.LINEAR_VELOCITY/2, goalX, goalY)
  #else:
  #    reload_smoother_param("faster")
  #    angle = control.rotateToAngleWithDetination(goalX, goalY)
  #    reload_smoother_param("slower")
  #    quaternion = quaternion_from_euler(0, 0, angle)
  #    gotoPointWithQuaternion(goalX, goalY, quaternion)


  #reload_smoother_param("faster")
  #angle = control.rotateToAngleWithDetination(goalX, goalY)
  #reload_smoother_param("slower")
  #quaternion = quaternion_from_euler(0, 0, angle)
  #gotoPointWithQuaternion(goalX, goalY, quaternion)

  ##if isStraightGoal(fromX,fromY,goalX,goalY):

  ##    if isShortDistance(distance):
  ##        logMsg = "isShortDistance. distance:%s"%(distance)
  ##        logWrite('debug', moudleName, logMsg)
  ##        #control.goToPositionInAStraightLineOnlyBumper(conf.SLOW_LINEAR_VELOCITY, goalX, goalY)
  ##        control.goToPositionInAStraightLine(conf.SLOW_LINEAR_VELOCITY, goalX, goalY)

  ##    else:
  ##       logMsg = "control.goToPositionInAStraightLine(%s, %s, %s) currentX:%s currentY:%s"%(conf.LINEAR_VELOCITY, goalX, goalY, share.currentX, share.currentY)
  ##       logWrite('warning',moudleName,logMsg)
  ##       control.goToPositionInAStraightLine(conf.LINEAR_VELOCITY, goalX, goalY)

  ##else:
  ##   logMsg = "gotoPointWithQuaternion(%s, %s) currentX:%s currentY:%s"%(goalX, goalY, share.currentX, share.currentY)
  ##   logWrite('warning',moudleName,logMsg)
  ##   gotoPointWithQuaternion(goalX,goalY,quaternion)


  #if isShortDistance(distance):
  #    logMsg = "isShortDistance. distance:%s"%(distance)
  #    logWrite('debug', moudleName, logMsg)
  #    control.rotateToAngleWithDetination(goalX, goalY)
  #    control.goToPositionInAStraightLineOnlyBumper(conf.LINEAR_VELOCITY, goalX, goalY)
  #else:
  #    angle = control.rotateToAngleWithDetination(goalX, goalY)
  #    quaternion = quaternion_from_euler(0, 0, angle)
  #    gotoPointWithQuaternion(goalX, goalY, quaternion)


  #####
  # if isStraightGoal(fromX,fromY,goalX,goalY):
  #    #logMsg = "control.goToPositionInAStraightLine(%s, %s, %s) currentX:%s currentY:%s"%(conf.LINEAR_VELOCITY, goalX, goalY, share.currentX, share.currentY)
  #    #logWrite('warning',moudleName,logMsg)
  #    #control.goToPositionInAStraightLine(conf.LINEAR_VELOCITY, goalX, goalY)

  #    logMsg = "control.rotateToAngleWithDetination(%s, %s) gotoPoint(%s, %s) currentX:%s currentY:%s"%(goalX, goalY, goalX, goalY, share.currentX, share.currentY)
  #    logWrite('warning',moudleName,logMsg)
  #    angle = control.rotateToAngleWithDetination(goalX, goalY)
  #    quaternion = quaternion_from_euler(0,0,angle)   #the rotation of the transformation as a tuple (x,y,z,w)
  #    gotoPointWithQuaternion(goalX,goalY,quaternion)

  # else:
  #    logMsg = "gotoPoint(%s, %s) currentX:%s currentY:%s"%(goalX, goalY, share.currentX, share.currentY)
  #    logWrite('warning',moudleName,logMsg)
  #    gotoPoint(goalX,goalY)


def isShortDistance(distance):
    if distance < 0.5:
        return True

    return False

def isStraightGoal(fromX, fromY, goalX, goalY):
  threshold = 0.15

  #logMsg = "if (%s - %s) = %s < %s . (fromX - goalX) < threshold"%(fromX , math.copysign(goalX,fromX),(fromX - math.copysign(goalX,fromX)) , threshold)
  # logWrite('debug',share.moudleName,logMsg)

  #logMsg = "if (%s - %s) = %s < %s . (fromY - goalY) < threshold"%(fromY , math.copysign(goalY,fromY),(fromY - math.copysign(goalY,fromY)),  threshold)
  # logWrite('debug',share.moudleName,logMsg)

  if (fromX - math.copysign(goalX, fromX)) < threshold:
    return True

  if (fromY - math.copysign(goalY, fromY)) < threshold:
    return True
  return False


# Callback function that processes the initial position received. 
def convertPointToCell(point, mapOrigin, mapResolution):
    (mapOriginX, mapOriginY) = mapOrigin
    goalPosCellX = int((point.x - mapOriginX) // mapResolution)
    goalPosCellY = int((point.y - mapOriginY) // mapResolution)
    tempCell = (goalPosCellX, goalPosCellY)
    #print "The position selected was outside of the grid.  postion:",point     
    #if not grid.isWithinGrid(tempCell):     
    #    raise Exception("Error: The position selected was outside of the grid! Please, try again.")     
    return tempCell 

# Converts cells to points 
def convertCellToPoint(cell, cellOrigin, mapResolution):
     (x, y) = cell
     (cellOriginX, cellOriginY) = cellOrigin
     point = Point()
     point.x = cellOriginX + mapResolution*x
     point.y = cellOriginY + mapResolution*y
     return point

#mapWidth is mapData.info.width
#map data:  100 occupied      -1 unknown       0 free
def getGridValue(mapData,cell):
    mapResolution = mapData.info.resolution
    mapOrigin = (mapData.info.origin.position.x,mapData.info.origin.position.y)
    mapWidth = mapData.info.width

    indx = getCellIndex(cell,mapResolution,mapOrigin,mapWidth)
    if indx < 0:
        out = -1
        return out

    out=mapData.data[indx];
    return out;


#TODO  bad result, need improve
def getMapPointValue(mapData,x,y):
    mapResolution = mapData.info.resolution
    mapOrigin = (mapData.info.origin.position.x,mapData.info.origin.position.y)
    mapWidth = mapData.info.width

    indx = getCellIndex((x,y),mapResolution,mapOrigin,mapWidth)
    if indx < 0:
        out = -1
        return out
    

    try:
        out=mapData.data[indx];
    except IndexError, e:
        out = -1

    #logMsg = "==== x:%s y:%s  indx:%s out:%s len(data):%s"%(x,y,indx,out,len(mapData.data))
    #logWrite('debug',share.moudleName,logMsg)
    
    return out;



def mergeMapData(oldMap,newMap):

    data = []
    for i in range(len(oldMap.data)):
        try:
            if newMap.data[i] == -1:
                data.append(100)
            else:
                data.append(oldMap.data[i])
        except IndexError as e:
            data.append(100)

    newMap.data = data
    return newMap


#mapOrigin = (99,98)
#mapWidth is mapData.info.width
#perPointA = (11,12)
#perPointB = (11,12)
#valueList = {}
#valueList[perPointA] = 100
#valueList[perPointB] = 100
def setGridValue(mapData,valueList):

    mapResolution = mapData.info.resolution
    mapOrigin = (mapData.info.origin.position.x,mapData.info.origin.position.y)
    mapWidth = mapData.info.width

    for pointX,pointY in valueList:
        cellTmp = (pointX,pointY)
        valueTmp = valueList[cellTmp]
        indx = getCellIndex(cellTmp,mapResolution,mapOrigin,mapWidth)
        if indx < 0:
            logMsg = "setGridValue error with point(%s,%s). because is out of map."%(pointX, pointY)
            logWrite('warning',share.moudleName,logMsg)
            continue

        mapData.data[indx] = valueTmp;
    return mapData


def setGridValueBatchWithSameValue(mapData,cellList,value,mapResolution,mapOrigin,mapWidth):

    logMsg = "==== 4  mapResolution:%s mapOriginX:%s mapOriginY:%s mapWidth:%s len(cellList):%s"%(mapResolution,mapOrigin[0], mapOrigin[1], mapWidth,len(cellList))
    logWrite('debug',share.moudleName,logMsg)

    appendData = {}
    for i in range(len(cellList)):
        cellTmp = cellList[i]
        #logMsg = "==== 14 cellList:%s,%s "%(cellList[i])
        indx = getCellIndex(cellTmp,mapResolution,mapOrigin,mapWidth)
        if indx < 0:
            logMsg = "setGridValue error with point(%s,%s). because is out of map."%(cellTmp[0], cellTmp[1])
            logWrite('warning',share.moudleName,logMsg)
            continue
        #logMsg = "==== 14 cellTmp(%s,%s) indx:%s"%(cellTmp[0],cellTmp[1],indx)
        #logWrite('debug',share.moudleName,logMsg)
        appendData[indx] = True

    #appendData[0] = True
    #appendData[1] = True
    #appendData[2] = True
    #appendData[3] = True
    #appendData[4] = True


    logMsg = "==== 5 len(mapData.data):%s"%(len(mapData.data))
    logWrite('debug',share.moudleName,logMsg)

    logMsg = "==== y appendData:%s"%(len(appendData))
    logWrite('debug',share.moudleName,logMsg)



    data = []
    for i in range(len(mapData.data)):
        if i in appendData:
            data.append(value)
        #elif mapData.data[i] == -1:
        #    data.append(100)    #unknown -> obstacle
        else:
            data.append(mapData.data[i])

    logMsg = "==== 7  mapWidth:%s"%(mapWidth)
    logWrite('debug',share.moudleName,logMsg)
    mapData.data = data

    logMsg = "==== 6  mapWidth:%s"%(mapWidth)
    logWrite('debug',share.moudleName,logMsg)

    return mapData

def getCellIndex(cell,mapResolution,mapOrigin,mapWidth):
    (mapOriginX, mapOriginY) = mapOrigin
    indx = int(( ((cell[1]-mapOriginY)//mapResolution)*mapWidth ) + ( ((cell[0]-mapOriginX)//mapResolution) ));
    #if indx < 0:   #has this situation, because area has unknown in map
    #    logMsg = "==== 6  indx:%s  cell[1]:%s  cell[0]:%s mapOriginY:%s mapOriginX:%s mapResolution:%s "%(( ((cell[1]-mapOriginY)//mapResolution)*mapWidth ) + ( ((cell[0]-mapOriginX)//mapResolution) ), cell[1], cell[0], mapOriginY, mapOriginX,mapResolution)
    #    logWrite('debug',share.moudleName,logMsg)
    return indx



def getBorderCellList(topLeft,bottomLeft,bottomRight,topRight,mapResolution,mapOrigin):
    (mapOriginX, mapOriginY) = mapOrigin

    ###########################
    # (-2,2) --------------- (2,2)
    #        |             |
    #        |             |
    #        |             |
    #        |             |
    #        |             |
    # (-2,-2) --------------- (2,-2)
    ###########################
    borderList = []
    

    ###########################
    # (-2,2)   --------------- 
    # (-2,1.5) |             |
    # (-2,1)   |             |
    # (-2,0.5) |             |
    #   ...    |             |
    # (-2,-2)  --------------
    ###########################
    #left border
    startX = topLeft.point.x
    startY = topLeft.point.y
    endX = bottomLeft.point.x
    endY = bottomLeft.point.y
    
    for y in np.arange(endY,startY,mapResolution):
        borderList.append((startX,y))

    ###########################
    #  --------------- 
    #  |             |
    #  |             |
    #  |             |
    #  |             |
    #  --------------
    #  (-2,-2) , (-1.5,-2), (-1,-2), (-0.5 -2) (0,-2) ... (2,-2)
    ###########################
    #bottorm border
    startX = bottomLeft.point.x
    startY = bottomLeft.point.y
    endX = bottomRight.point.x
    endY = bottomRight.point.y
    
    for x in np.arange(startX,endX,mapResolution):
        borderList.append((x,startY))


    ###########################
    # --------------  (2,2)   
    # |             | (2,1.5)
    # |             | (2,1)  
    # |             | (2,0.5)
    # |             |   ...   
    # --------------  (2,-2) 
    ###########################
    #right border
    startX = bottomRight.point.x
    startY = bottomRight.point.y
    endX = topRight.point.x
    endY = topRight.point.y
    
    for y in np.arange(startY,endY,mapResolution):
        logMsg = "==== 11 startX:%s y:%s    new:(%s,%s)"%(startX,y,startX-mapOriginX, y)
        logWrite('debug',share.moudleName,logMsg)
        borderList.append((startX,y))


    ###########################
    #  (-2,2) , (-1.5,2), (-1,2), (-0.5,2) (0,2) ... (2,2)
    #  --------------- 
    #  |             |
    #  |             |
    #  |             |
    #  |             |
    #  --------------
    ###########################
    #top border
    startX = topLeft.point.x
    startY = topLeft.point.y
    endX = topRight.point.x
    endY = topRight.point.y
    
    for x in np.arange(startX,endX,mapResolution):
        borderList.append((x,startY))

    logMsg = "==== 6.  len(borderList):%s"%(len(borderList))
    logWrite('debug',share.moudleName,logMsg)
    return borderList


def updateMapWithborder(topLeft,bottomLeft,bottomRight,topRight,mapData,mapResolution,mapOrigin,mapWidth):
    cellList = getBorderCellList(topLeft,bottomLeft,bottomRight,topRight,mapResolution,mapOrigin)
    value = 100
    newMapData = setGridValueBatchWithSameValue(mapData,cellList,value,mapResolution,mapOrigin,mapWidth)
    return newMapData



def gotCurrentAreaIndexList(bottomLeft, topLeft, topRight, bottomRight, mapData, mapResolution, mapOrigin, mapWidth, mapHeight):
    currentAreaIndexList = getCurrentMapIndex(bottomLeft, topLeft, topRight, bottomRight, mapData, mapResolution, mapOrigin, mapWidth, mapHeight)

    appendData = {}
    for i in range(len(currentAreaIndexList)):
        indx = currentAreaIndexList[i]
        appendData[indx] = True

    return appendData

        #(bottomLeft, topLeft, topRight, bottomRight)
def gotCurrentAreaMap(bottomLeft, topLeft, topRight, bottomRight, mapData, mapResolution, mapOrigin, mapWidth, mapHeight):
    currentAreaIndexList = gotCurrentAreaIndexList(bottomLeft, topLeft, topRight, bottomRight, mapData, mapResolution, mapOrigin, mapWidth, mapHeight)


    valueNotInCurrentArea = 100
    data = []
    for i in range(len(mapData.data)):
        if i in currentAreaIndexList:
            data.append(mapData.data[i])
        else:
            data.append(valueNotInCurrentArea)
    mapData.data = data

    return mapData


def getPointValue(x,y,mapData):
    mapResolution = mapData.info.resolution
    mapOrigin = (mapData.info.origin.position.x, mapData.info.origin.position.y)
    mapWidth = mapData.info.width
    mapHeight = mapData.info.height
    indx = getCellIndex((x, y), mapResolution, mapOrigin, mapWidth)
    if indx < 0:
        out = -1
        return out
    valueTmp = mapData.data[indx]

    return valueTmp


def testPointOnMap(x,y,mapData,currentAreaMapPub):

    cleanBeginPoint = createPointStamped(Point(x, y,0),"map")
    share.currnetAreaCleanBeginPointpub.publish(cleanBeginPoint)
    
    
    mapInput = copy.deepcopy(share.mapData)
    mapResolution = mapInput.info.resolution
    mapOrigin = (mapInput.info.origin.position.x, mapInput.info.origin.position.y)
    mapWidth = mapInput.info.width
    mapHeight = mapInput.info.height

    currentAreaIndexList = {}

    indx = getCellIndex((x, y), mapResolution, mapOrigin, mapWidth)
    if indx > 0:
        currentAreaIndexList[indx] = True
        valueTmp = mapInput.data[indx]
    else:
        valueTmp = -1
    
    valueNotInCurrentArea = 100
    data = []
    for i in range(len(mapInput.data)):
        if i in currentAreaIndexList:
            data.append(valueNotInCurrentArea)
            logWrite('debug',share.moudleName,"i:%s = 100."%(i) )
        else:
            data.append(mapInput.data[i])
    
    mapInput.data = data
    
    
    rate = rospy.Rate(conf.POS_REQUEST_RATE)
    for i in range(0, 3):
        if not (mapInput is None):
            try:
                currentAreaMapPub.publish(mapInput)
            except rospy.ROSException as e:
                logWrite('debug',share.moudleName,"currentAreaMapPub.publish.publish fail. %s"%(e) )
    
        rate.sleep()
        continue
    
    logMsg = "testPoint:(%s,%s) index;%s valueTmp:%s "%(1.85, -0.9, indx, valueTmp)
    logWrite('debug',share.moudleName,logMsg)
    #####################







def updateMapFillCurrentArea(bottomLeft, topLeft, topRight, bottomRight, mapData, mapResolution, mapOrigin, mapWidth, mapHeight):
    currentAreaIndexList = getCurrentMapIndex(bottomLeft, topLeft, topRight, bottomRight, mapData, mapResolution, mapOrigin, mapWidth, mapHeight)
    value = 100

    #logMsg = "====   currentAreaIndexList:%s"%(currentAreaIndexList)
    #logWrite('debug',share.moudleName,logMsg)


    appendData = {}
    for i in range(len(currentAreaIndexList)):
        indx = currentAreaIndexList[i]
        appendData[indx] = True


    data = []
    for i in range(len(mapData.data)):
        if i in appendData:
            data.append(value)
        else:
            data.append(mapData.data[i])


    mapData.data = data

    return mapData


def getCurrentMapIndex(bottomLeft, topLeft, topRight, bottomRight, mapData, mapResolution, mapOrigin, mapWidth, mapHeight):


    ###########################
    # (-2,2) --------------- (2,2)
    #        |             |
    #        |             |
    #        |             |
    #        |             |
    #        |             |
    # (-2,-2) --------------- (2,-2)
    ###########################


    boundMaxX = mapOrigin[0] + mapWidth*mapResolution
    boundMaxY = mapOrigin[1] + mapHeight*mapResolution

    startX = topLeft.point.x
    startY = topLeft.point.y



    endX = bottomRight.point.x
    endY = bottomRight.point.y

    startX = max(mapOrigin[0], startX)
    startY = min(boundMaxY, startY)


    #logMsg = "====   mapOrigin_0:%s  startX:%s"%(mapOrigin[0],startX)
    #logWrite('debug',share.moudleName,logMsg)
    #logMsg = "====   boundMaxY:%s  startY:%s"%(boundMaxY,startY)
    #logWrite('debug',share.moudleName,logMsg)


    endX = min(boundMaxX,endX)
    endX = max(mapOrigin[0],endX)

    endY = min(boundMaxY,endY)
    endY = max(mapOrigin[1],endY)

    #logMsg = "====   endX:%s  endY:%s"%(endX,endY)
    #logWrite('debug',share.moudleName,logMsg)

    startPoint = createPointStamped(Point(startX,startY,0),"map");
    endPoint = createPointStamped(Point(endX,endY,0),"map")

    for i in range(0, 3):
        share.currnetAreaStartPointpub.publish(startPoint)
        share.currnetAreaEndPointpub.publish(endPoint)


    ###########################
    # (-2,2)   (-1.5,2)   (-1,2) (-0.5,2)   (0,2)   (0.5,2)   (1,2)   (1.5,2)  (2,2)
    # |-------|--------|--------|--------|--------|--------|--------|--------|--------|
    # |                                                                               |
    # |                                                                               |
    # |                                                                               |
    # |                                                                               |
    # |                                                                               |
    # |                                                                               |
    # |-------|--------|--------|--------|--------|--------|--------|--------|--------|
    ###########################
    #top line


    #TODO if max > mapWidth

    indxList = []


    

    #startY to endY
    #logMsg = "====   startY:%s  endY:%s"%(startY,endY)
    #logWrite('debug',share.moudleName,logMsg)
    for y in np.arange(min(startY,endY),max(startY,endY),mapResolution):
        indx = getCellIndex( (startX,y) , mapResolution, mapOrigin, mapWidth)
        if indx < 0:
            continue
        offSet = int((endX - startX)/mapResolution)



        #logMsg = "====   startX:%s endX:%s boundX:%s boundY:%s offSet:%s, mapResolution:%s mapOrigin:%s mapWidth:%s mapHeight:%s"%(startX,endX,boundX,boundY,offSet,mapResolution, mapOrigin, mapWidth, mapHeight)
        #logWrite('debug',share.moudleName,logMsg)



        #logMsg = "----   offSet:%s endX:%s startX:%s"%(offSet,endX,startX)
        #logWrite('debug',share.moudleName,logMsg)
        indxList.append(indx)
        #startX to endX
        for i in range(offSet):
            indxList.append(indx+i)
            #logMsg = "----   y:%s indx_i:%s"%(y,indx+i)
            #logWrite('debug',share.moudleName,logMsg)

    return indxList

#get plan
#found the N goal ,it's the new goal
def getNewGoal(plan, theNum=30):
    tuplePoses = plan.poses

    logMsg = "getNewGoal. len(tuplePoses):%s"%(len(tuplePoses))
    logWrite('debug',share.moudleName,logMsg)

    if len(tuplePoses) > theNum:
        return tuplePoses[theNum]
    else:
        return tuplePoses[len(tuplePoses)-1]


def gotoGoalSplit(control, finalGoalX, finalGoalY):

    #Wait for the availability of this service
    rospy.wait_for_service(conf.SERVICE_MAKE_PLAN)
    #Get a proxy to execute the service
    make_plan = rospy.ServiceProxy(conf.SERVICE_MAKE_PLAN, GetPlan)

    distance = math.sqrt((finalGoalX - share.currentX)**2 + (finalGoalY - share.currentY)**2)
    thresholdDistance = 0.5

    logMsg = "gotoGoalSplit. start."
    logWrite('debug',share.moudleName,logMsg)
    while distance > thresholdDistance:

        start = createPoseWithRPY(share.currentX, share.currentY, 0.0, 0, 0, 0, conf.FRAME_ID_MAP)
        goal  = createPoseWithRPY(finalGoalX, finalGoalY, 0.0, 0, 0, 0, conf.FRAME_ID_MAP)
        tolerance = 0.5 #meter

        planResponse = make_plan(start = start, goal = goal, tolerance = tolerance)
        if len(planResponse.plan.poses) <= 0:
            return False

        newTmpGoal = getNewGoal(planResponse.plan)

        logMsg = "gotoGoalSplit. newTmpGoal(%s,%s)"%(newTmpGoal.pose.position.x, newTmpGoal.pose.position.y)
        logWrite('debug',share.moudleName,logMsg)
        share.localGoalX = newTmpGoal.pose.position.x
        share.localGoalY = newTmpGoal.pose.position.y
        if cleanMoveToWithMoveBase(control,share.currentX,share.currentY,share.localGoalX, share.localGoalY):
            distance = math.sqrt((finalGoalX - share.currentX)**2 + (finalGoalY - share.currentY)**2)
            logMsg = "gotoGoalSplit. reached tmp goal."
            logWrite('debug',share.moudleName,logMsg)
        else:
            logMsg = "gotoGoalSplit. can not reached Goal."
            logWrite('debug',share.moudleName,logMsg)
            return False
            break  #can not reached Goal

    distance = math.sqrt((finalGoalX - share.currentX)**2 + (finalGoalY - share.currentY)**2)
    if distance <=  thresholdDistance:
        return True
    else:
        return False

    #call make plan service
    #got split goal
    #went to goal
    #if near the finial goal
    #finished


