#!/usr/bin/env python
import time
import copy
from sensor_msgs.msg import JointState

def sorted_joint_state_msg(msg, joint_names):
  """
  Returns a sorted C{sensor_msgs/JointState} for the given joint names
  @type  msg: sensor_msgs/JointState
  @param msg: The input message
  @type  joint_names: list
  @param joint_names: The sorted joint names
  @rtype: sensor_msgs/JointState
  @return: The C{JointState} message with the fields in the order given by joint names
  """
  valid_names = set(joint_names).intersection(set(msg.name))
  valid_position = len(msg.name) == len(msg.position)
  valid_velocity = len(msg.name) == len(msg.velocity)
  valid_effort = len(msg.name) == len(msg.effort)
  num_joints = len(valid_names)
  retmsg = JointState()
  retmsg.header = copy.deepcopy(msg.header)
  for name in joint_names:
    if name not in valid_names:
      continue
    idx = msg.name.index(name)
    retmsg.name.append(name)
    if valid_position:
      retmsg.position.append(msg.position[idx])
    if valid_velocity:
      retmsg.velocity.append(msg.velocity[idx])
    if valid_effort:
      retmsg.effort.append(msg.effort[idx])
  return retmsg

def wait_for(predicate, timeout=5.0):
  start_time = time.time()
  while not predicate():
    now = time.time()
    if (now - start_time) > timeout:
      return False
    time.sleep(0.001)
  return True
