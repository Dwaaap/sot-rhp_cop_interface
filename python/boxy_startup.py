# 0. TRICK: import Dynamic as the first command to avoid the crash at the exit
from dynamic_graph.sot.dynamics import Dynamic

# 1. Instanciate a Boxy
# The URDF description of the robot must have 
# been loaded in robot_description parameter
# on the Ros Parameter Server
# 1. Init robot, ros binding, solver
from dynamic_graph.sot.boxy.boxy_tasks import *
from dynamic_graph.sot.boxy.robot import *
from dynamic_graph.sot.core import RobotSimu
from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.core.meta_task_6d import toFlags
from dynamic_graph.sot.core.meta_task_visual_point import MetaTaskVisualPoint
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.dyninv import TaskInequality, TaskJointLimits
from dynamic_graph.sot.robohow.tools import *
from dynamic_graph.sot.robohow.gripper import Gripper
from dynamic_graph.sot.robohow.cylinder_pouring import CylinderPouring
import time

#robot = Boxy('Boxy', device=RobotSimu('Boxy'))
plug(robot.device.state, robot.dynamic.position)

# 2. Ros binding
# roscore must be running
from dynamic_graph.ros import *
ros = Ros(robot)

# Use kine solver (with inequalities)
from dynamic_graph.sot.dyninv import SolverKine
solver = initialize(robot, SolverKine)

#Define the main tasks
from dynamic_graph.sot.core.meta_tasks_kine import gotoNd, goto6d, MetaTaskKine6d
taskBase = BoxyBaseTask(robot)
currentWaistPos = robot.features['waist'].position.value
gotoNd(taskBase, currentWaistPos, '111111')
solver.push(taskBase.task)


#Add a task to immobilize the arms base
robot.features['featurePositionBase'] = FeaturePosture('featurePositionBase')
plug(robot.device.state, robot.features['featurePositionBase'].state)
robot.features['featurePositionBase'].posture.value = robot.halfSitting
postureTaskDofs = [False]*(len(robot.halfSitting))
postureTaskDofs[8] = True
for dof,isEnabled in enumerate(postureTaskDofs):
  if dof > 5:
    robot.features['featurePositionBase'].selectDof(dof,isEnabled)
robot.tasks['task_base_imo']=Task('task_base_imo')
robot.tasks['task_base_imo'].add('featurePositionBase')
gainPosition = GainAdaptive('gainPositionBase')
gainPosition.set(0.1,0.1,125e3)
gainPosition.gain.value = 5
plug(robot.tasks['task_base_imo'].error,gainPosition.error)
plug(gainPosition.gain,robot.tasks['task_base_imo'].controlGain)
robot.tasks['task_base_imo'].error.recompute(0)
solver.push(robot.tasks['task_base_imo'])

import numpy as np
from dynamic_graph.sot.expression_graph.types import *
from dynamic_graph.sot.expression_graph.expression_graph import *
from dynamic_graph.sot.expression_graph.functions import *
from dynamic_graph.sot.core.operator import Norm_of_vector#, EntityToMatrix
from dynamic_graph.sot.rh_cop_interface import *
robot.entities={}
robot.expressions={}
robot.taskGoals={}
robot.monitors={}
robot.OpPointModifiers={}
robot.currentTasks=[]
robot.removeTasks=[]
robot.plug={}

expg = FeaturePointToLineDistance('expg')
taskExpg = Task('taskExpg')
taskExpg.controlGain.value = 1
opPoint1 = 'right-wrist'
opPoint2 = 'right-wrist'
plug(robot.dynamic.signal(opPoint1),expg.signal('w_T_o1'))
plug(robot.dynamic.signal('J'+opPoint1),expg.signal('w_J_o1'))
plug(robot.dynamic.signal(opPoint2),expg.signal('w_T_o2'))
plug(robot.dynamic.signal('J'+opPoint2),expg.signal('w_J_o2'))
taskExpg.add(expg.name)
expg.reference.value = (0,)
solver.push(taskExpg)
#solver.sot.remove('taskExpg')


