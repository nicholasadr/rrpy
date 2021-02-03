#!/usr/bin/env python
import itertools

import rospy
import numpy as np
import openravepy as orpy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import rr_task as rrtask

class Planner(object):
  def __init__(self, robot, config, config_format="yaml"):
    self.robot = robot
    self.env = robot.GetEnv()
    self.PS_HasSolution = 1
    # Update class attributes based on provided config file
    cfg = rrtask.config.Config(config, config_format=config_format)
    cfg.update_config_in_list(self, self.get_modifiable_attributes())

  def get_modifiable_attributes(self):
    return ["planner_name", "max_iters", "max_ppiters"]

  def plan_to_joint_configuration(self, qgoal, try_swap=False):
    qstart = self.robot.GetActiveDOFValues()
    planner = orpy.RaveCreatePlanner(self.env, self.planner_name)
    params = orpy.Planner.PlannerParameters()
    params.SetMaxIterations(self.max_iters)
    if self.max_ppiters > 0:
      params.SetPostProcessing('ParabolicSmoother',
              '<_nmaxiterations>{0}</_nmaxiterations>'.format(self.max_ppiters))
    else:
      params.SetPostProcessing('', '')
    # Plan trajectory
    best_traj = None
    min_duration = float('inf')
    reversed_is_better = False
    count = 0
    for qa, qb in itertools.permutations([qstart, qgoal], 2):
      count += 1
      with self.robot:
        self.robot.SetActiveDOFValues(qa)
        params.SetGoalConfig(qb)
        params.SetRobotActiveJoints(self.robot)
        initsuccess = planner.InitPlan(self.robot, params)
        if initsuccess:
          traj = orpy.RaveCreateTrajectory(self.env, '')
          status = planner.PlanPath(traj)             # Plan the trajectory
          if status.statusCode == self.PS_HasSolution:
            duration = traj.GetDuration()
            if duration < min_duration:
              min_duration = duration
              best_traj = orpy.RaveCreateTrajectory(self.env, traj.GetXMLId())
              best_traj.Clone(traj, 0)
              if count == 2:
                reversed_is_better = True
      if not try_swap:
        break
    # Check if we need to reverse the trajectory
    if reversed_is_better:
      best_traj = orpy.planningutils.ReverseTrajectory(best_traj)
    return best_traj

  def ros_trajectory_from_openrave(self, traj):
    """
    Converts an OpenRAVE trajectory into a ROS JointTrajectory message.
    @type  robot_name: str
    @param robot_name: The robot name
    @type  traj: orpy.Trajectory
    @param traj: The input OpenRAVE trajectory
    @rtype: trajectory_msgs/JointTrajectory
    @return: The equivalent ROS JointTrajectory message
    """
    robot_name = self.robot.GetName()
    ros_traj = JointTrajectory()
    # Specification groups
    spec = traj.GetConfigurationSpecification()
    try:
      values_group = spec.GetGroupFromName('joint_values {0}'.format(robot_name))
    except orpy.openrave_exception:
      orpy.RaveLogError('Corrupted trajectory. Failed to find group: joint_values')
      return None
    try:
      velocities_group = spec.GetGroupFromName('joint_velocities {0}'.format(robot_name))
    except orpy.openrave_exception:
      orpy.RaveLogError('Corrupted trajectory. Failed to find group: joint_velocities')
      return None
    try:
      deltatime_group = spec.GetGroupFromName('deltatime')
    except orpy.openrave_exception:
      orpy.RaveLogError('Corrupted trajectory. Failed to find group: deltatime')
      return None
    # Copy waypoints
    time_from_start = 0
    for i in range(traj.GetNumWaypoints()):
      waypoint = traj.GetWaypoint(i).tolist()
      deltatime = waypoint[deltatime_group.offset]
      # OpenRAVE trajectory sometimes comes with repeated waypoints. DO NOT append them
      if np.isclose(deltatime, 0) and i > 0:
        continue
      # Append waypoint
      ros_point = JointTrajectoryPoint()
      ros_point.positions = waypoint[values_group.offset:values_group.offset+values_group.dof]
      ros_point.velocities = waypoint[velocities_group.offset:velocities_group.offset+velocities_group.dof]
      time_from_start += deltatime
      ros_point.time_from_start = rospy.Duration(time_from_start)
      ros_traj.points.append(ros_point)
    return ros_traj
