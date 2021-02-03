#!/usr/bin/env python
import copy
import threading

import rospy
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from ur_msgs.srv import SetSpeedSliderFraction
from controller_manager_msgs.srv import ListControllers, SwitchController, SwitchControllerRequest

import rr_ros as rrros
import rr_task as rrtask

class ROSHWInterface(object):
  def __init__(self, config, config_format, namespace='', logger=rospy):
    # Update class attributes based on provided config file
    cfg = rrtask.config.Config(config, config_format=config_format)
    cfg.update_config_in_list(self, self.get_modifiable_attributes())

  def get_modifiable_attributes(self):
    raise NotImplementedError()


class ROSRobotInterface(ROSHWInterface):
  def __init__(
      self, config, config_format="yaml", namespace='', logger=rospy,
      simulation=True):
    super(ROSRobotInterface, self).__init__(
        config, config_format, namespace=namespace, logger=logger
        )
    if not hasattr(self, "joint_names"):
      raise NotImplementedError(), "joint_names attribute is not available"
    if not hasattr(self, "controllers"):
      raise NotImplementedError(), "controllers attribute is not available"
    self.running_controller = None
    self.logger = logger
    self.ns = self.solve_namespace(namespace=namespace)
    self.simulation = simulation
    if self.simulation:
      self.logger.logdebug("Running simulated robot")
    else:
      self.logger.logdebug("Running real robot")
    # Initialize
    self.has_ros_robot = self.connect_to_robot()

  def __del__(self):
    if hasattr(self, "_js_sub"):
      self.disconnect()

  def get_modifiable_attributes(self):
    return ["connect_timeout", "controller_timeout", "traj_delay"]

  def connect_to_robot(self):
    # TODO: Any parameters to read?

    # Subscribe to joint_states
    self.joint_states_msg = None
    self.lock_js = threading.Lock()
    self._js_sub = rospy.Subscriber("{}joint_states".format(self.ns), JointState,
                                    self.cb_joint_states, queue_size=1)
    self.logger.logdebug("Waiting for [{}joint_states] topic".format(self.ns))
    if not rrros.utils.wait_for(lambda: self.joint_states_msg is not None,
        timeout=self.connect_timeout):
      self.logger.logerr("Timed out waiting for joint_states topic")
      return False
    self.logger.logdebug("Subscribed to topic [{}joint_states]".format(self.ns))
    return True

  def disconnect(self):
    """Disconnects from the joint_states topic. Useful to lighten system resources.
    """
    self._js_sub.unregister()

  def cb_joint_states(self, msg):
    """Callback executed very time a message is published in the C{joint_states} topic.
    @type  msg: sensor_msgs/JointState
    @param msg: The JointState message published by the RT hardware interface.
    """
    with self.lock_js:
      sorted_msg = rrros.utils.sorted_joint_state_msg(msg, self.joint_names)
      if len(sorted_msg.name) == len(self.joint_names):
        self.joint_states_msg = copy.deepcopy(sorted_msg)
      else:
        self.joint_states_msg = None

  def load_controllers(self, controller_type):
    if self.running_controller == controller_type:
      return True
    if controller_type not in self.controllers.keys():
      self.logger.logdebug('Invalid controller_type: {0}'.format(controller_type))
      return False
    # Are the controllers already running?
    list_controllers = rospy.ServiceProxy('{}controller_manager/list_controllers'.format(self.ns), ListControllers)
    list_controllers.wait_for_service(timeout=self.controller_timeout)
    res = list_controllers()
    required_resources = []
    start_controllers = self.controllers[controller_type]
    def get_controller_resources(controller):
      """
      Returns a list of tuples (hardware_interface, resource)
      """
      resources = []
      for claimed in controller.claimed_resources:
        resources += [resource for resource in claimed.resources]
      return resources
    # List controllers already running.
    # Additionally get the list of resources we need
    running = []
    for controller in res.controller:
      for name in start_controllers:
        if controller.name == name:
          required_resources += get_controller_resources(controller)
      if controller.state == 'running':
        running.append(controller.name)
    if set(running).issuperset(self.controllers[controller_type]):
      # Nothing to do. Controllers already running.
      self.running_controller = controller_type
      return True
    # Stop controllers using resources that start_controllers need
    stop_controllers = []
    for controller in res.controller:
      is_running = controller.name in running
      is_claiming = set(required_resources).issuperset(get_controller_resources(controller))
      if is_running and is_claiming:
        stop_controllers.append(controller.name)
    # Switch the controllers
    switch_controller = rospy.ServiceProxy('{}controller_manager/switch_controller'.format(self.ns), SwitchController)
    switch_controller.wait_for_service(timeout=self.controller_timeout)
    res = switch_controller(start_controllers=start_controllers, stop_controllers=stop_controllers,
                            strictness=SwitchControllerRequest.BEST_EFFORT)
    self.logger.logdebug('Attempted to start controllers: {0} and stop controllers: {1}'.format(start_controllers, stop_controllers))
    self.running_controller = controller_type
    return res.ok

  def get_joint_positions(self):
    """Returns joint positions sorted based on self.joint_names in radian.
    """
    with self.lock_js:
      jnt_positions = np.array(self.joint_states_msg.position)
    return jnt_positions

  def execute_trajectory(self, ros_traj, wait=True, execute_timeout=10., preempt_timeout=5.):
    if not self.has_ros_robot:
      self.logger.logwarn('execute_trajectory: Connection with robot is missing')
      return False
    if "TRAJ" not in self.running_controller:
      self.logger.logwarn('execute_trajectory: TrajectoryController is not running')
      return False
    self.controller.init_new_goal()
    self.controller.set_trajectory(ros_traj)
    # Send trajectory
    if wait:
      assert(execute_timeout > 0.)
      self.controller.start_and_wait(delay=self.traj_delay, execute_timeout=execute_timeout,
	                             preempt_timeout=preempt_timeout)
    else:
      self.controller.start(delay=delay)
    return True

  def solve_namespace(self, namespace=''):
    """Appends neccessary slashes required for a proper ROS namespace.
    @type namespace: string
    @param namespace: namespace to be fixed
    @rtype: string
    @return: proper ROS namespace
    """
    if len(namespace) == 0:
      namespace = rospy.get_namespace()
    elif len(namespace) == 1:
      if namespace != '/':
        namespace = '/' + namespace + '/'
    else:
      if namespace[0] != '/':
        namespace = '/' + namespace
      if namespace[-1] != '/':
        namespace += '/'
    return namespace
