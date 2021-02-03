#!/usr/bin/env python
import time
import yaml

import rospy

from . import config as _config
import rr_orpy as rrorpy


class InterfaceManager(object):
  def __init__(self, ros_interface, openrave_interface, config, config_format):
    self.has_ros_interface = False
    self.has_openrave_interface = False
    self.register_ros_interface(ros_interface)
    self.register_openrave_interface(openrave_interface)
    assert self.has_both_interfaces()
    # Update class attributes based on provided config file
    cfg = _config.Config(config, config_format=config_format)
    cfg.update_config_in_list(self, self.get_modifiable_attributes())
    # Initialize
    self.mimic_ros_joint_in_openrave()

  def get_modifiable_attributes(self):
    raise NotImplementedError

  def has_both_interfaces(self):
    return self.has_ros_interface and self.has_openrave_interface

  def register_openrave_interface(self, orpy_interface):
    assert orpy_interface.has_orpy_robot
    self._orpy = orpy_interface
    self.has_openrave_interface = True

  def register_ros_interface(self, ros_interface):
    assert ros_interface.has_ros_robot
    self._ros = ros_interface
    self.has_ros_interface = True

  @classmethod
  def build_interfaces(cls, orpy_class, ros_class, env,
      orpy_config, ros_config, task_config,
      orpy_cfg_format="yaml", ros_cfg_format="yaml", task_cfg_format="yaml",
      simulation="True", namespace='', logger=rospy):
    orpy_intf = orpy_class(env, orpy_config, config_format=orpy_cfg_format)
    ros_intf = ros_class(ros_config, config_format=ros_cfg_format,
	simulation=simulation, namespace=namespace, logger=logger)
    return cls(ros_intf, orpy_intf, task_config, config_format=task_cfg_format)

  ### OpenRAVE interface functionalities  ###
  
  def set_viewer(self):
    self._orpy.set_viewer()

  def set_camera_pose(self, T):
    try:
      self._orpy.env.GetViewer()
    except NameError:
      time.sleep(0.001)
    self._orpy.env.GetViewer().SetCamera(T)

  def init_planner_interface(self, config, config_format="yaml",
      planner_cls=None):
    self._orpy.init_planner_interface(
	config, config_format=config_format, planner_cls=planner_cls
	)
    self.planner = self._orpy.planner

  ### ROS interface functionalities ###

  def in_simulation(self):
    return self._ros.simulation

  def get_ros_joint_positions(self):
    return self._ros.get_joint_positions()

  ### Combined functionalities ###

  def mimic_ros_joint_in_openrave(self):
    self._orpy.robot.SetActiveDOFValues(self._ros.get_joint_positions())

  def execute_trajectories(self, openrave_traj, wait=True):
    ros_traj = self.planner.ros_trajectory_from_openrave(openrave_traj)
    self._orpy.execute_trajectory(openrave_traj)
    self._ros.execute_trajectory(ros_traj, wait=wait)
