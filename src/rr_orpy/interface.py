#!/usr/bin/env python
import openravepy as orpy

import rr_task as rrtask

class OpenraveRobotInterface(object):
  def __init__(self, env, config, config_format="yaml"):
    self.env = env
    self.has_orpy_robot = False
    # Update class attributes based on provided config file
    cfg = rrtask.config.Config(config, config_format=config_format)
    cfg.update_config_in_list(self, self.get_modifiable_attributes())
    # Initiate
    if self.load_world:
      if not self.env.Load(self.load_world):
        raise Exception("World file does not exist: {}".format(self.load_world))
    if self.load_urdf and self.load_srdf:
      urdf_module = orpy.RaveCreateModule(env, "urdf")
      self.robot_name = urdf_module.SendCommand("LoadURI {} {}".format(
        self.load_urdf,self.load_srdf))
    if hasattr(self, "robot_name"):
      with self.env:
        self.robot = self.env.GetRobot(self.robot_name)
        self.has_orpy_robot = True
        if self.manipulator_name is not None:
          self.set_active_manipulator(self.manipulator_name)
    self.max_velocity = self.robot.GetDOFVelocityLimits()
    self.max_acceleration = self.robot.GetDOFAccelerationLimits()
    self.enabled_bodies = self.get_enabled_bodies(self.env)
    self.checker = orpy.RaveCreateCollisionChecker(self.env, self.collision_checker)
    self.enable_collision_check(self.check_collision)

  def get_modifiable_attributes(self):
    # TODO: logger
    return ["viewer", "load_world", "load_urdf", "load_srdf", "manipulator_name",
            "collision_checker", "check_collision"]

  def get_openrave_robot_handle(self):
    if not hasattr(self, "robot"):
      raise Exception("Openrave robot handle is not available")
    else:
      return self.robot

  def set_viewer(self):
    self.env.SetViewer(self.viewer)

  def set_active_manipulator(self, manipulator):
    """
    Set the robot's active manipulator
    @type  manipulator: string
    @param manipulator: name of the manipulator to set active
    """
    try:
      self.manipulator = self.robot.SetActiveManipulator(manipulator)
    except:
      return Exception("Failed to set active manipulator: {}".format(manipulator))
    self.dofindices = self.manipulator.GetArmIndices()
    self.robot.SetActiveDOFs(self.dofindices)

  def scale_velocity_limit(self, scale):
    """Scales and sets the velocity limits by the given scale from the absolute max velocity.
    @type  scale: float
    @param scale: scale value in the range [0.01, 1.0]
    """
    factor = max(0.01, min(scale, 1.0))
    self.robot.SetDOFVelocityLimits(factor * self.max_velocity)

  def scale_acceleration_limit(self, scale):
    """Scales and sets the acceleration limits by the given scale from the absolute max acceleration.
    @type  scale: float
    @param scale: scale value in the range [0.01, 1.0]
    """
    factor = max(0.01, min(scale, 1.0))
    self.robot.SetDOFAccelerationLimits(factor * self.max_acceleration)

  def get_enabled_bodies(self, env):
    """Returns a set with the names of the bodies enabled in the given environment.
    @type  env: orpy.Environment
    @param env: OpenRAVE environment
    @rtype: set
    @return: names of enabled bodies
    """
    enabled_bodies = []
    with env:
      for body in env.GetBodies():
        if body.IsEnabled():
          enabled_bodies.append(body.GetName())
    return set(enabled_bodies)

  def enable_collision_check(self, enable=True):
    """Enables the collision checking in OpenRAVE.
    @type  enable: bool
    @param enable: if true, the collision check is switched on
    """
    # Include additional enabled bodies
    current_enabled = self.get_enabled_bodies(self.env)
    if self.enabled_bodies != current_enabled:
      self.enabled_bodies = self.enabled_bodies.union(current_enabled)
    for name in self.enabled_bodies:
      self.env.GetKinBody(name).Enable(enable)
    with self.env:
      if enable:
        self.collision_checking = self.env.SetCollisionChecker(self.checker)
      else:
        self.env.SetCollisionChecker(None)
        self.collision_checking = False

  def init_planner_interface(self, config, config_format="yaml",
                             planner_cls=None):
    """Initialize planner module.
    @type  config: string
    @param config: path to config file
    @type  config_format: string
    @param config_format: config file format
    """
    if planner_cls is None:
      from . import planner
      planner_cls = planner.Planner
    self.planner = planner_cls(self.robot, config, config_format=config_format)

  def execute_trajectory(self, traj):
    self.robot.GetController().SetPath(traj)
