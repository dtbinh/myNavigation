#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from demo_flexbe_states.go_forward_state import DemoFState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Aug 03 2018
@author: Alberto Ezquerro
'''
class DemoBehaviorSM(Behavior):
	'''
	A behavior for the demo of the FlexBe course
	'''


	def __init__(self):
		super(DemoBehaviorSM, self).__init__()
		self.name = 'Demo Behavior'

		# parameters of this behavior
		self.add_parameter('my_speed', 0.4)
		self.add_parameter('my_travel_dist', 1)
		self.add_parameter('my_obs_dist', 1.5)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:512 y:35, x:516 y:173
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:144 y:104
			OperatableStateMachine.add('Drive Forward',
										DemoFState(speed=self.my_speed, travel_dist=self.my_travel_dist, obstacle_dist=self.my_obs_dist),
										transitions={'failed': 'failed', 'done': 'finished'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
