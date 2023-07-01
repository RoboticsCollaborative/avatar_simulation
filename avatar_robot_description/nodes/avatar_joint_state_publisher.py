#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import copy

initial_position_list=[0.0, 0.61204416, 0.2010726199999997, -1.6332415999999998, 1.36810506, 1.4878609999999999, -0.92539762, -0.3332, -0.28559999999999997, 1.1438125, -0.4784389312, -0.4765328, -0.4784389312, -0.4765328, -1.19, -1.1749999999999998, -1.1913319999999998, 0.0, 0.3426883199999997, 0.03998274000000013, -2.3525207999999997, -1.70998646, 1.566277, 0.44270743999999995, -0.33226760000000005, -0.2848008, 1.1438125, -0.477904, -0.476, -0.477904, -0.476, -1.1866700000000001, -1.1749999999999998, -1.19]

class JointStatePublisher(object):
  def __init__(self):
    # subscribe from topic published by joint_state_publisher_gui
    self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
    # publish our own joint state topic 
    self.joint_state_pub = rospy.Publisher('/avatar_joint_states', JointState, queue_size=10)

    self.initialized = False
    self.motor_joint_idx = {}

  def joint_state_init(self):
    initial_full_joint_state = JointState()
    # initial_full_joint_state.header = initial_act_joint_state.header
    # initial_full_joint_state.name = initial_act_joint_state.name
    initial_full_joint_state.position = list(initial_position_list)
    self.joint_state_pub.publish(initial_full_joint_state)
    # print('ok')
    # print(initial_full_joint_state.position)
    
  def joint_state_callback(self, msg):
    if (not self.initialized):
      # find the index of the actively actuated joint
      try:
        if 'right_thumb_flex_motor_joint' in msg.name:
          self.motor_joint_idx['right_thumb_flex_motor_joint'] = msg.name.index('right_thumb_flex_motor_joint')
        if 'right_thumb_swivel_motor_joint' in msg.name:
          self.motor_joint_idx['right_thumb_swivel_motor_joint'] = msg.name.index('right_thumb_swivel_motor_joint')
        if 'right_index_flex_motor_joint' in msg.name:
          self.motor_joint_idx['right_index_flex_motor_joint'] = msg.name.index('right_index_flex_motor_joint')
        if 'left_thumb_flex_motor_joint' in msg.name:
          self.motor_joint_idx['left_thumb_flex_motor_joint'] = msg.name.index('left_thumb_flex_motor_joint')
        if 'left_thumb_swivel_motor_joint' in msg.name:
          self.motor_joint_idx['left_thumb_swivel_motor_joint'] = msg.name.index('left_thumb_swivel_motor_joint')
        if 'left_index_flex_motor_joint' in msg.name:
          self.motor_joint_idx['left_index_flex_motor_joint'] = msg.name.index('left_index_flex_motor_joint')
        self.initialized = True
      except ValueError as e:
        print(e)

    # actively actuated joint 
    act_joint_state = msg
    # full joint state with active and passive joints
    full_joint_state = JointState()
    full_joint_state.header = act_joint_state.header
    full_joint_state.name = act_joint_state.name
    full_joint_state.position = list(act_joint_state.position)
    self.process_passive_joint(full_joint_state)
    self.joint_state_pub.publish(full_joint_state)
  
  def linear_mapping_joint(self, joint_state, multiplier, offset, motor_joint):
    return multiplier * joint_state.position[self.motor_joint_idx[motor_joint]] + offset
  
  def process_passive_joint(self, joint_state):
    for i, name in enumerate(joint_state.name):
      if 'left_thumb_knuckle_joint' in name:
        joint_state.position[i] = self.linear_mapping_joint(
          joint_state,
          multiplier=0.28,
          offset=0.0,
          motor_joint='left_thumb_flex_motor_joint'
        )
      elif 'left_thumb_finger_joint' in name:
        joint_state.position[i] = self.linear_mapping_joint(
          joint_state,
          multiplier=0.24,
          offset=0.0,
          motor_joint='left_thumb_flex_motor_joint'
        )
      elif 'left_thumb_swivel_joint' in name:
        joint_state.position[i] = self.linear_mapping_joint(
          joint_state,
          multiplier=-0.6075,
          offset=0.43,
          motor_joint='left_thumb_swivel_motor_joint'
        )
      elif 'left_index_knuckle_joint' in name:
        joint_state.position[i] = self.linear_mapping_joint(
          joint_state,
          multiplier=0.4016,
          offset=0.0,
          motor_joint='left_index_flex_motor_joint'
        )
      # write the same for other joints: left_index_fingertip_joint, left_middle_knuckle_joint, left_middle_fingertip_joint
      elif 'left_index_fingertip_joint' in name:
        joint_state.position[i] = self.linear_mapping_joint(
          joint_state,
          multiplier=0.4,
          offset=0.0,
          motor_joint='left_index_flex_motor_joint'
        )
      elif 'left_middle_knuckle_joint' in name:
        joint_state.position[i] = self.linear_mapping_joint(
          joint_state,
          multiplier=0.4016,
          offset=0.0,
          motor_joint='left_index_flex_motor_joint'
        )
      elif 'left_middle_fingertip_joint' in name:
        joint_state.position[i] = self.linear_mapping_joint(
          joint_state,
          multiplier=0.4,
          offset=0.0,
          motor_joint='left_index_flex_motor_joint'
        )
      elif 'right_thumb_knuckle_joint' in name:
        joint_state.position[i] = self.linear_mapping_joint(
          joint_state,
          multiplier=0.28,
          offset=0.0,
          motor_joint='right_thumb_flex_motor_joint'
        )
      elif 'right_thumb_finger_joint' in name:
        joint_state.position[i] = self.linear_mapping_joint(
          joint_state,
          multiplier=0.24,
          offset=0.0,
          motor_joint='right_thumb_flex_motor_joint'
        )
      elif 'right_thumb_swivel_joint' in name:
        joint_state.position[i] = self.linear_mapping_joint(
          joint_state,
          multiplier=-0.6075,
          offset=0.43,
          motor_joint='right_thumb_swivel_motor_joint'
        )
      elif 'right_index_knuckle_joint' in name:
        joint_state.position[i] = self.linear_mapping_joint(
          joint_state,
          multiplier=0.4016,
          offset=0.0,
          motor_joint='right_index_flex_motor_joint'
        )
      # write the same for other joints: right_index_fingertip_joint, right_middle_knuckle_joint, right_middle_fingertip_joint
      elif 'right_index_fingertip_joint' in name:
        joint_state.position[i] = self.linear_mapping_joint(
          joint_state,
          multiplier=0.4,
          offset=0.0,
          motor_joint='right_index_flex_motor_joint'
        )
      elif 'right_middle_knuckle_joint' in name:
        joint_state.position[i] = self.linear_mapping_joint(
          joint_state,
          multiplier=0.4016,
          offset=0.0,
          motor_joint='right_index_flex_motor_joint'
        )
      elif 'right_middle_fingertip_joint' in name:
        joint_state.position[i] = self.linear_mapping_joint(
          joint_state,
          multiplier=0.4,
          offset=0.0,
          motor_joint='right_index_flex_motor_joint'
        )


if __name__ == "__main__":
  rospy.init_node('avatar_joint_state_publisher')
  joint_state_publisher = JointStatePublisher()
  joint_state_publisher.joint_state_init()
  rospy.spin()



