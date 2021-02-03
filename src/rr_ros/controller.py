#!/usr/bin/env python
import copy
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from trajectory_msgs.msg import JointTrajectoryPoint

class ControllerBase(object):
  def __init__(self, joint_names, namespace="/", logger=rospy):
    self.joint_names = joint_names
    self.ns = namespace
    self.logger = logger

class TrajControllerBase(ControllerBase):
  def __init__(self, joint_names, namespace="/", logger=rospy):
    super(TrajControllerBase, self).__init__(
        joint_names, namespace=namespace, logger=logger)

  def init_new_goal(self):
    self._goal = FollowJointTrajectoryGoal()
    self._goal.trajectory.joint_names = copy.deepcopy(self.joint_names)
    return self._goal

  def add_point(self):
    raise NotImplementedError

  def clear_points(self):
    """
    Clear all points in the trajectory.
    """
    self._goal.trajectory.points[:]=[]

  def get_num_points(self):
    """
    Returns the number of points currently added to the trajectory
    @rtype: int
    @return: Number of points currently added to the trajectory
    """
    return len(self._goal.trajectory.points)

  def get_result(self):
    """
    Returns the result B{after} the execution of the trajectory. Possible values:
      - FollowJointTrajectoryResult.SUCCESSFUL = 0
      - FollowJointTrajectoryResult.INVALID_GOAL = -1
      - FollowJointTrajectoryResult.INVALID_JOINTS = -2
      - FollowJointTrajectoryResult.OLD_HEADER_TIMESTAMP = -3
      - FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED = -4
      - FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED = -5
    @rtype: int
    @return: result B{after} the execution of the trajectory
    """
    return self._client.get_result()

  def get_state(self):
    """
    Returns the status B{during} the execution of the trajectory. Possible values:
      - GoalStatus.PENDING=0
      - GoalStatus.ACTIVE=1
      - GoalStatus.PREEMPTED=2
      - GoalStatus.SUCCEEDED=3
      - GoalStatus.ABORTED=4
      - GoalStatus.REJECTED=5
      - GoalStatus.PREEMPTING=6
      - GoalStatus.RECALLING=7
      - GoalStatus.RECALLED=8
      - GoalStatus.LOST=9
    @rtype: int
    @return: result B{after} the execution of the trajectory
    """
    return self._client.get_state()

  def set_trajectory(self, trajectory):
    """
    Sets the goal trajectory directly. B{It only copies} the C{trajectory.points} field. 
    @type  trajectory: trajectory_msgs/JointTrajectory
    @param trajectory: The goal trajectory
    """
    self._goal.trajectory.points = copy.deepcopy(trajectory.points)

  def start(self, delay=0.1):
    """
    Starts the trajectory. It sends the C{FollowJointTrajectoryGoal} to the action server.
    @type  delay: float
    @param delay: Delay (in seconds) before executing the trajectory
    """
    num_points = len(self._goal.trajectory.points)
    self.logger.logdebug('Executing Joint Trajectory with {0} points'.format(num_points))
    self._goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(delay)
    self._client.send_goal(self._goal)

  def start_and_wait(self, delay=0.1, execute_timeout=10.0, preempt_timeout=5.0):
    """
    Starts the trajectory. It sends the C{FollowJointTrajectoryGoal} to the action server\
    and waits for the goal to complete, and preempts goal if necessary.
    @type  delay: float
    @param delay: Delay (in seconds) before executing the trajectory
    @param execute_timeout: The time to wait for the goal to complete (in seconds).
    @param preempt_timeout: The time to wait for preemption to complete (in seconds).
    """
    num_points = len(self._goal.trajectory.points)
    self.logger.logdebug('Executing Joint Trajectory with {0} points'.format(num_points))
    self._goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(delay)
    self._client.send_goal_and_wait(self._goal, execute_timeout=rospy.Duration(execute_timeout),
                                    preempt_timeout=rospy.Duration(preempt_timeout))

  def stop(self):
    """
    Stops an active trajectory. If there is not active trajectory an error will be shown in the console.
    """
    self._client.cancel_goal()

  def wait(self, timeout=15.0):
    """
    Waits synchronously (with a timeout) until the trajectory action server gives a result.
    @type  timeout: float
    @param timeout: The amount of time we will wait
    @rtype: bool
    @return: True if the server connected in the allocated time. False on timeout
    """
    return self._client.wait_for_result(timeout=rospy.Duration(timeout))

class PositionTrajController(TrajControllerBase):
  def __init__(self, joint_names, namespace="/", logger=rospy):
    super(PositionTrajController, self).__init__(
        joint_names, namespace=namespace, logger=logger)
    self.init_new_goal()

  def add_point(self, positions, time, velocities=None, accelerations=None):
    """
    Adds a point to the trajectory. Each point must be specified by the goal position and 
    the goal time. The velocity and acceleration are optional.
    @type  positions: list
    @param positions: The goal position in the joint space
    @type  time: float
    @param time: The time B{from start} when the robot should arrive at the goal position.
    @type  velocities: list
    @param velocities: The velocity of arrival at the goal position. If not given zero 
    velocity is assumed.
    @type  accelerations: list
    @param accelerations: The acceleration of arrival at the goal position. If not given 
    zero acceleration is assumed.
    """
    point = JointTrajectoryPoint()
    point.positions = copy.deepcopy(positions)
    if type(velocities) == type(None):
      point.velocities = [0] * self._num_joints
    else:
      point.velocities = copy.deepcopy(velocities)
    if type(accelerations) == type(None):
      point.accelerations = [0] * self._num_joints
    else:
      point.accelerations = copy.deepcopy(accelerations)
    point.time_from_start = rospy.Duration(time)
    self._goal.trajectory.points.append(point)


class JointControllerBase(ControllerBase):
  def __init__(self, joint_names, namespace="/", logger=rospy):
    super(JointControllerBase, self).__init__(
        joint_names, namespace=namespace, logger=logger)

  def disconnect(self):
    self._pub.unregister()

  def init_new_goal(self, label="joint"):
    self._goal = Float64MultiArray()
    self._goal.layout.dim.append(MultiArrayDimension())
    self._goal.layout.dim[0].label = label
    self._goal.layout.dim[0].size = len(self.joint_names)
    return self._goal

  def set_goal_command(self, cmd):
    if self.valid_joint_command(cmd):
      self._goal.data = cmd
    else:
      raise ValueError("Command should be of length: {} and of type: float".format(len(self.joint_names)))

  def start(self): 
    self._pub.publish(self._goal)

  def stop(self):
    self.set_goal_command([0.]*len(self.joint_names))
    self._pub.publish(self._goal)

  def valid_joint_command(self, cmd):
    length_ok = (len(cmd) == len(self.joint_names))
    type_ok = all(isinstance(x, float) for x in cmd)
    return (length_ok and type_ok)
