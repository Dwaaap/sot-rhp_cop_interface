#!/usr/bin/env python
import roslib; roslib.load_manifest('rh_cop')
import rospy
import smach
import smach_ros
from rh_cop_msgs.msg import *
from simple_sm_action import *
from numpy import pi, cos, sin, sqrt
from copy import deepcopy

######### Define TASK
taskdb = {}

### Settings here!!!!
tframe="right-wrist"
simulation=True
radius=0.2;   # [m]
angle= pi/2;  # [rad]
spoon_lenght = 0.23  # [m] from gripper frame to spoon tip
spoon_offset = 0.00  # [m] if spoon not exactly centered in the gripper grasp
top_height = 0.25    # [m] on top of pizza, before to go down 
#### End_settings


##### Some common defs 

point_origin = EntitySpec()
point_origin.type = EntitySpec.POINT;
point_origin.point.x = 0
point_origin.point.y = 0
point_origin.point.z = 0

point_xtrasl = EntitySpec()
point_xtrasl.type = EntitySpec.POINT;
point_xtrasl.point.x =0.0 
point_xtrasl.point.y = spoon_offset #Our spoon dimension
point_xtrasl.point.z = spoon_lenght #Our spoon dimension

point_ztrasl = EntitySpec()
point_ztrasl.type = EntitySpec.POINT;
point_ztrasl.point.x = 0
point_ztrasl.point.y = 0
point_ztrasl.point.z = top_height

xline = EntitySpec()
xline.type = EntitySpec.LINE;
xline.line.p.x = 0
xline.line.p.y = 0
xline.line.p.z = 0
xline.line.dir.x = 1 
xline.line.dir.y = 0
xline.line.dir.z = 0

yline = EntitySpec()
yline.type = EntitySpec.LINE;
yline.line.p.x = 0
yline.line.p.y = 0
yline.line.p.z = 0
yline.line.dir.x = 0 
yline.line.dir.y = 1
yline.line.dir.z = 0

zline = EntitySpec()
zline.type = EntitySpec.LINE;
zline.line.p.x = 0
zline.line.p.y = 0
zline.line.p.z = 0
zline.line.dir.x = 0 
zline.line.dir.y = 0
zline.line.dir.z = 1

znversor = EntitySpec()
znversor.type = EntitySpec.VERSOR;
znversor.versor.x = 0 
znversor.versor.y = 0 
znversor.versor.z = -1 

xversor = EntitySpec()
xversor.type = EntitySpec.VERSOR;
xversor.versor.x = 1
xversor.versor.y = 0 
xversor.versor.z = 0 

yversor = EntitySpec()
yversor.type = EntitySpec.VERSOR;
yversor.versor.x = 0
yversor.versor.y = 1 
yversor.versor.z = 0 

zversor = EntitySpec()
zversor.type = EntitySpec.VERSOR;
zversor.versor.x = 0
zversor.versor.y = 0 
zversor.versor.z = 1 


plane_origin = EntitySpec()
plane_origin.type = EntitySpec.PLANE
plane_origin.plane.p.x = 0
plane_origin.plane.p.y = 0
plane_origin.plane.p.z = 0
plane_origin.plane.normal.x = 0
plane_origin.plane.normal.y = 0
plane_origin.plane.normal.z = 1

plane_spoon = EntitySpec()
plane_spoon.type = EntitySpec.PLANE
plane_spoon.plane.p.x = 0
plane_spoon.plane.p.y = spoon_offset
plane_spoon.plane.p.z = spoon_lenght
plane_spoon.plane.normal.x = 0
plane_spoon.plane.normal.y = 0
plane_spoon.plane.normal.z = 1


spoon_versor = EntitySpec()
spoon_versor.type = EntitySpec.VERSOR;
spoon_versor.versor.x = 0   # angle spoon
spoon_versor.versor.y = -sqrt(2)/2  # angle spoon
spoon_versor.versor.z = sqrt(2)/2

approaching_traj = TrajectorySpec()
approaching_traj.type = TrajectorySpec.TRAPEZOIDAL
approaching_traj.target   = 0.0
approaching_traj.vel_max  = 0.1
approaching_traj.acc_max  = 0.1
approaching_traj.duration = 5.0


distance_traj = TrajectorySpec()
distance_traj.type = TrajectorySpec.TRAPEZOIDAL
distance_traj.target   = 0.15
distance_traj.vel_max  = 0.1
distance_traj.acc_max  = 0.1
distance_traj.duration = 5.0

angle_traj = TrajectorySpec()
angle_traj.type = TrajectorySpec.TRAPEZOIDAL
angle_traj.target   = pi/2
angle_traj.vel_max  = 0.1
angle_traj.acc_max  = 0.1
angle_traj.duration = 5.0


push_force_traj = TrajectorySpec()
push_force_traj.type = TrajectorySpec.CONSTANT
push_force_traj.target   = -2.0

to_radius_traj = deepcopy(approaching_traj)
to_radius_traj.target   = radius

######## Begin Define TASK

#Position task

g_ptp = TaskGoal()
g_ptp.id = "ptp_dist"
task_ptp = ConstraintSpec()
task_ptp.constraint_id = "ptp_dist"
task_ptp.behaviour.type = BehaviourSpec.POSITIONING
task_ptp.behaviour.specification = .5
task_ptp.output_expression.expression = GeometricExpressionSpec.POINT_TO_POINT_DISTANCE
task_ptp.output_expression.id = "ptp_dist_expr"
task_ptp.output_expression.p1.object_frame = tframe
task_ptp.output_expression.p1.entity = point_xtrasl
task_ptp.output_expression.p2.object_frame = "table"
task_ptp.output_expression.p2.entity = point_ztrasl
task_ptp.tr_gen = distance_traj
g_ptp.primary.append(task_ptp)
m_ptp = MonitorSpec()
m_ptp.monitor_id = "M_ptp_dist";
m_ptp.monitored_expr = task_ptp.output_expression;
m_ptp.event_risen = "e_done";
m_ptp.comparison_type = MonitorSpec.IN_INTERVAL;
m_ptp.monitored_variable_type = MonitorSpec.POS;
m_ptp.lower_bound = -0.015 + 0.15;
m_ptp.upper_bound = +0.015 + 0.15;
g_ptp.monitors.append(m_ptp)

taskdb[g_ptp.id] = g_ptp

g_lp = TaskGoal()
g_lp.id = "lp_dist"
task_lp = ConstraintSpec()
task_lp.constraint_id = "lp_dist"
task_lp.behaviour.type = BehaviourSpec.POSITIONING
task_lp.behaviour.specification = 1.0
task_lp.output_expression.expression = GeometricExpressionSpec.LINE_POINT_DISTANCE
task_lp.output_expression.id = "lp_dist_expr"
task_lp.output_expression.p1.object_frame = tframe
task_lp.output_expression.p1.entity = point_xtrasl
task_lp.output_expression.p2.object_frame = "table"
task_lp.output_expression.p2.entity = yline
task_lp.tr_gen = distance_traj 
g_lp.primary.append(task_lp)
m_lp = MonitorSpec()
m_lp.monitor_id = "M_lp_dist";
m_lp.monitored_expr = task_lp.output_expression;
m_lp.event_risen = "e_done";
m_lp.comparison_type = MonitorSpec.IN_INTERVAL;
m_lp.monitored_variable_type = MonitorSpec.POS;
m_lp.lower_bound = -0.015 + 0.15
m_lp.upper_bound = +0.015 + 0.15
g_lp.monitors.append(m_lp)
taskdb[g_lp.id] = g_lp

g_sp = TaskGoal()
g_sp.id = "sp_dist"
task_sp = ConstraintSpec()
task_sp.constraint_id = "sp_dist"
task_sp.behaviour.type = BehaviourSpec.POSITIONING
task_sp.behaviour.specification = 1.0
task_sp.output_expression.expression = GeometricExpressionSpec.SURFACE_POINT_DISTANCE
task_sp.output_expression.id = "sp_dist_expr"
task_sp.output_expression.p1.object_frame = "table"#tframe
task_sp.output_expression.p1.entity = point_xtrasl 
task_sp.output_expression.p2.object_frame = tframe#"table"
task_sp.output_expression.p2.entity = plane_origin
task_sp.tr_gen = approaching_traj 
g_sp.primary.append(task_sp)
m_sp = MonitorSpec()
m_sp.monitor_id = "M_sp_dist";
m_sp.monitored_expr = task_sp.output_expression;
m_sp.event_risen = "e_done";
m_sp.comparison_type = MonitorSpec.IN_INTERVAL;
m_sp.monitored_variable_type = MonitorSpec.POS;
m_sp.lower_bound = -0.015
m_sp.upper_bound = +0.015
g_sp.monitors.append(m_sp)
taskdb[g_sp.id] = g_sp

g_ll = TaskGoal()
g_ll.id = "ll_dist"
task_ll = ConstraintSpec()
task_ll.constraint_id = "ll_dist"
task_ll.behaviour.type = BehaviourSpec.POSITIONING
task_ll.behaviour.specification = 1.0
task_ll.output_expression.expression = GeometricExpressionSpec.LINE_LINE_DISTANCE
task_ll.output_expression.id = "ll_dist_expr"
task_ll.output_expression.p1.object_frame = tframe
task_ll.output_expression.p1.entity = yline
task_ll.output_expression.p2.object_frame = "table"
task_ll.output_expression.p2.entity = zline
task_ll.tr_gen = distance_traj 
g_ll.primary.append(task_ll)
m_ll = MonitorSpec()
m_ll.monitor_id = "M_ll_dist";
m_ll.monitored_expr = task_ll.output_expression;
m_ll.event_risen = "e_done";
m_ll.comparison_type = MonitorSpec.IN_INTERVAL;
m_ll.monitored_variable_type = MonitorSpec.POS;
m_ll.lower_bound = -0.015 + 0.15
m_ll.upper_bound = +0.015 + 0.15
g_ll.monitors.append(m_ll)
taskdb[g_ll.id] = g_ll


g_ppa = TaskGoal()
g_ppa.id = "pp_angle"
task_ppa = ConstraintSpec()
task_ppa.constraint_id = "pp_angle"
task_ppa.behaviour.type = BehaviourSpec.POSITIONING
task_ppa.behaviour.specification = 1.0
task_ppa.output_expression.expression = GeometricExpressionSpec.ANGLE_BTW_PLANES
task_ppa.output_expression.id = "pp_angle_expr"
task_ppa.output_expression.p1.object_frame = tframe
task_ppa.output_expression.p1.entity = plane_spoon
task_ppa.output_expression.p2.object_frame = "table"
task_ppa.output_expression.p2.entity = plane_origin
task_ppa.tr_gen = angle_traj 
g_ppa.primary.append(task_ppa)
m_ppa = MonitorSpec()
m_ppa.monitor_id = "M_pp_angle";
m_ppa.monitored_expr = task_ppa.output_expression;
m_ppa.event_risen = "e_done";
m_ppa.comparison_type = MonitorSpec.IN_INTERVAL;
m_ppa.monitored_variable_type = MonitorSpec.POS;
m_ppa.lower_bound = -0.015 + pi/2
m_ppa.upper_bound = +0.015 + pi/2
g_ppa.monitors.append(m_ppa)
taskdb[g_ppa.id] = g_ppa


g_vv = TaskGoal()
g_vv.id = "vv_angle"
task_vv = ConstraintSpec()
task_vv.constraint_id = "vv_angle"
task_vv.behaviour.type = BehaviourSpec.POSITIONING
task_vv.behaviour.specification = 1.0
task_vv.output_expression.expression = GeometricExpressionSpec.ANGLE_BTW_VERSORS
task_vv.output_expression.id = "vv_angle_expr"
task_vv.output_expression.p1.object_frame = tframe
task_vv.output_expression.p1.entity = xversor
task_vv.output_expression.p2.object_frame = "table"
task_vv.output_expression.p2.entity = yversor
task_vv.tr_gen = angle_traj 
g_vv.primary.append(task_vv)
m_vv = MonitorSpec()
m_vv.monitor_id = "M_vv_angle";
m_vv.monitored_expr = task_vv.output_expression;
m_vv.event_risen = "e_done";
m_vv.comparison_type = MonitorSpec.IN_INTERVAL;
m_vv.monitored_variable_type = MonitorSpec.POS;
m_vv.lower_bound = -0.015 + pi/2
m_vv.upper_bound = +0.015 + pi/2
g_vv.monitors.append(m_vv)
taskdb[g_vv.id] = g_vv


g_pv = TaskGoal()
g_pv.id = "pv_angle"
task_pv = ConstraintSpec()
task_pv.constraint_id = "pv_angle"
task_pv.behaviour.type = BehaviourSpec.POSITIONING
task_pv.behaviour.specification = 1.0
task_pv.output_expression.expression = GeometricExpressionSpec.ANGLE_BTW_PLANE_AND_VERSOR
task_pv.output_expression.id = "pv_angle_expr"
task_pv.output_expression.p1.object_frame = tframe
task_pv.output_expression.p1.entity = plane_spoon
task_pv.output_expression.p2.object_frame = "table"
task_pv.output_expression.p2.entity = zversor
task_pv.tr_gen = angle_traj 
g_pv.primary.append(task_pv)
m_pv = MonitorSpec()
m_pv.monitor_id = "M_pv_angle";
m_pv.monitored_expr = task_pv.output_expression;
m_pv.event_risen = "e_done";
m_pv.comparison_type = MonitorSpec.IN_INTERVAL;
m_pv.monitored_variable_type = MonitorSpec.POS;
m_pv.lower_bound = -0.015 + pi/2
m_pv.upper_bound = +0.015 + pi/2
g_pv.monitors.append(m_pv)
taskdb[g_pv.id] = g_pv


g_lp = TaskGoal()
g_lp.id = "lp_angle"
task_lp = ConstraintSpec()
task_lp.constraint_id = "lp_angle"
task_lp.behaviour.type = BehaviourSpec.POSITIONING
task_lp.behaviour.specification = 1.0
task_lp.output_expression.expression = GeometricExpressionSpec.ANGLE_BTW_LINE_AND_POINT
task_lp.output_expression.id = "lp_angle_expr"
task_lp.output_expression.p1.object_frame = tframe
task_lp.output_expression.p1.entity = point_xtrasl
task_lp.output_expression.p2.object_frame = "table"
task_lp.output_expression.p2.entity = zline
task_lp.tr_gen = angle_traj 
g_lp.primary.append(task_lp)
m_lp = MonitorSpec()
m_lp.monitor_id = "M_lp_angle";
m_lp.monitored_expr = task_lp.output_expression;
m_lp.event_risen = "e_done";
m_lp.comparison_type = MonitorSpec.IN_INTERVAL;
m_lp.monitored_variable_type = MonitorSpec.POS;
m_lp.lower_bound = -0.015 + pi/2
m_lp.upper_bound = +0.015 + pi/2
g_lp.monitors.append(m_lp)
taskdb[g_lp.id] = g_lp


#Position Limit task

g_ptp_l = TaskGoal()
g_ptp_l.id = "ptp_l_dist"
task_ptp_l = ConstraintSpec()
task_ptp_l.constraint_id = "ptp_l_dist"
task_ptp_l.behaviour.type = BehaviourSpec.POSITION_LIMIT
task_ptp_l.behaviour.specification = .5
task_ptp_l.output_expression.expression = GeometricExpressionSpec.POINT_TO_POINT_DISTANCE
task_ptp_l.output_expression.id = "ptp_l_dist_expr"
task_ptp_l.output_expression.p1.object_frame = tframe
task_ptp_l.output_expression.p1.entity = point_xtrasl
task_ptp_l.output_expression.p2.object_frame = "table"
task_ptp_l.output_expression.p2.entity = point_ztrasl
task_ptp_l.tr_gen = distance_traj
task_ptp_l.tr_gen_upper = 0.5
task_ptp_l.tr_gen_lower = 0.2
g_ptp_l.primary.append(task_ptp_l)
m_ptp_l = MonitorSpec()
m_ptp_l.monitor_id = "M_ptp_l_dist";
m_ptp_l.monitored_expr = task_ptp_l.output_expression;
m_ptp_l.event_risen = "e_done";
m_ptp_l.comparison_type = MonitorSpec.IN_INTERVAL;
m_ptp_l.monitored_variable_type = MonitorSpec.POS;
m_ptp_l.lower_bound = -0.03 + 0.2;
m_ptp_l.upper_bound = +0.03 + 0.5;
g_ptp_l.monitors.append(m_ptp_l)
taskdb[g_ptp_l.id] = g_ptp_l

g_lp_l = TaskGoal()
g_lp_l.id = "lp_l_dist"
task_lp_l = ConstraintSpec()
task_lp_l.constraint_id = "lp_l_dist"
task_lp_l.behaviour.type = BehaviourSpec.POSITION_LIMIT
task_lp_l.behaviour.specification = 1.0
task_lp_l.output_expression.expression = GeometricExpressionSpec.LINE_POINT_DISTANCE
task_lp_l.output_expression.id = "lp_l_dist_expr"
task_lp_l.output_expression.p1.object_frame = tframe
task_lp_l.output_expression.p1.entity = point_xtrasl
task_lp_l.output_expression.p2.object_frame = "table"
task_lp_l.output_expression.p2.entity = yline
task_lp_l.tr_gen = distance_traj 
task_lp_l.tr_gen_upper = 0.5
task_lp_l.tr_gen_lower = 0.2
g_lp_l.primary.append(task_lp_l)
m_lp_l = MonitorSpec()
m_lp_l.monitor_id = "M_lp_l_dist";
m_lp_l.monitored_expr = task_lp_l.output_expression;
m_lp_l.event_risen = "e_done";
m_lp_l.comparison_type = MonitorSpec.IN_INTERVAL;
m_lp_l.monitored_variable_type = MonitorSpec.POS;
m_lp_l.lower_bound = -0.015 + 0.2
m_lp_l.upper_bound = +0.015 + 0.5
g_lp_l.monitors.append(m_lp_l)
taskdb[g_lp_l.id] = g_lp_l

g_sp_l = TaskGoal()
g_sp_l.id = "sp_l_dist"
task_sp_l = ConstraintSpec()
task_sp_l.constraint_id = "sp_l_dist"
task_sp_l.behaviour.type = BehaviourSpec.POSITION_LIMIT
task_sp_l.behaviour.specification = 1.0
task_sp_l.output_expression.expression = GeometricExpressionSpec.SURFACE_POINT_DISTANCE
task_sp_l.output_expression.id = "sp_l_dist_expr"
task_sp_l.output_expression.p1.object_frame = "table"#tframe
task_sp_l.output_expression.p1.entity = point_xtrasl 
task_sp_l.output_expression.p2.object_frame = tframe#"table"
task_sp_l.output_expression.p2.entity = plane_origin
task_sp_l.tr_gen = approaching_traj 
task_sp_l.tr_gen_upper = 0.1
task_sp_l.tr_gen_lower = 0.0
g_sp_l.primary.append(task_sp_l)
m_sp_l = MonitorSpec()
m_sp_l.monitor_id = "M_sp_l_dist";
m_sp_l.monitored_expr = task_sp_l.output_expression;
m_sp_l.event_risen = "e_done";
m_sp_l.comparison_type = MonitorSpec.IN_INTERVAL;
m_sp_l.monitored_variable_type = MonitorSpec.POS;
m_sp_l.lower_bound = -0.015 + 0.0
m_sp_l.upper_bound = +0.015 + 0.1
g_sp_l.monitors.append(m_sp_l)
taskdb[g_sp_l.id] = g_sp_l

g_ll_l = TaskGoal()
g_ll_l.id = "ll_l_dist"
task_ll_l = ConstraintSpec()
task_ll_l.constraint_id = "ll_l_dist"
task_ll_l.behaviour.type = BehaviourSpec.POSITION_LIMIT
task_ll_l.behaviour.specification = 1.0
task_ll_l.output_expression.expression = GeometricExpressionSpec.LINE_LINE_DISTANCE
task_ll_l.output_expression.id = "ll_l_dist_expr"
task_ll_l.output_expression.p1.object_frame = tframe
task_ll_l.output_expression.p1.entity = yline
task_ll_l.output_expression.p2.object_frame = "table"
task_ll_l.output_expression.p2.entity = zline
task_ll_l.tr_gen = distance_traj 
task_ll_l.tr_gen_upper = 0.4
task_ll_l.tr_gen_lower = 0.2
g_ll_l.primary.append(task_ll_l)
m_ll_l = MonitorSpec()
m_ll_l.monitor_id = "M_ll_l_dist";
m_ll_l.monitored_expr = task_ll_l.output_expression;
m_ll_l.event_risen = "e_done";
m_ll_l.comparison_type = MonitorSpec.IN_INTERVAL;
m_ll_l.monitored_variable_type = MonitorSpec.POS;
m_ll_l.lower_bound = -0.015 + 0.2
m_ll_l.upper_bound = +0.015 + 0.4
g_ll_l.monitors.append(m_ll_l)
taskdb[g_ll_l.id] = g_ll_l


g_ppa_l = TaskGoal()
g_ppa_l.id = "pp_l_angle"
task_ppa_l = ConstraintSpec()
task_ppa_l.constraint_id = "pp_l_angle"
task_ppa_l.behaviour.type = BehaviourSpec.POSITION_LIMIT
task_ppa_l.behaviour.specification = 1.0
task_ppa_l.output_expression.expression = GeometricExpressionSpec.ANGLE_BTW_PLANES
task_ppa_l.output_expression.id = "pp_l_angle_expr"
task_ppa_l.output_expression.p1.object_frame = tframe
task_ppa_l.output_expression.p1.entity = plane_spoon
task_ppa_l.output_expression.p2.object_frame = "table"
task_ppa_l.output_expression.p2.entity = plane_origin
task_ppa_l.tr_gen = angle_traj 
task_ppa_l.tr_gen_upper = pi
task_ppa_l.tr_gen_lower = pi/2
g_ppa_l.primary.append(task_ppa_l)
m_ppa_l = MonitorSpec()
m_ppa_l.monitor_id = "M_pp_l_angle";
m_ppa_l.monitored_expr = task_ppa_l.output_expression;
m_ppa_l.event_risen = "e_done";
m_ppa_l.comparison_type = MonitorSpec.IN_INTERVAL;
m_ppa_l.monitored_variable_type = MonitorSpec.POS;
m_ppa_l.lower_bound = -0.015 + pi/2
m_ppa_l.upper_bound = +0.015 + pi
g_ppa_l.monitors.append(m_ppa_l)
taskdb[g_ppa_l.id] = g_ppa_l


g_vv_l = TaskGoal()
g_vv_l.id = "vv_l_angle"
task_vv_l = ConstraintSpec()
task_vv_l.constraint_id = "vv_l_angle"
task_vv_l.behaviour.type = BehaviourSpec.POSITION_LIMIT
task_vv_l.behaviour.specification = 1.0
task_vv_l.output_expression.expression = GeometricExpressionSpec.ANGLE_BTW_VERSORS
task_vv_l.output_expression.id = "vv_l_angle_expr"
task_vv_l.output_expression.p1.object_frame = tframe
task_vv_l.output_expression.p1.entity = xversor
task_vv_l.output_expression.p2.object_frame = "table"
task_vv_l.output_expression.p2.entity = yversor
task_vv_l.tr_gen = angle_traj 
task_vv_l.tr_gen_upper = pi/2
task_vv_l.tr_gen_lower = 0
g_vv_l.primary.append(task_vv_l)
m_vv_l = MonitorSpec()
m_vv_l.monitor_id = "M_vv_l_angle";
m_vv_l.monitored_expr = task_vv_l.output_expression;
m_vv_l.event_risen = "e_done";
m_vv_l.comparison_type = MonitorSpec.IN_INTERVAL;
m_vv_l.monitored_variable_type = MonitorSpec.POS;
m_vv_l.lower_bound = -0.015 + 0
m_vv_l.upper_bound = +0.015 + pi/2
g_vv_l.monitors.append(m_vv_l)
taskdb[g_vv_l.id] = g_vv_l


g_pv_l = TaskGoal()
g_pv_l.id = "pv_l_angle"
task_pv_l = ConstraintSpec()
task_pv_l.constraint_id = "pv_l_angle"
task_pv_l.behaviour.type = BehaviourSpec.POSITION_LIMIT
task_pv_l.behaviour.specification = 1.0
task_pv_l.output_expression.expression = GeometricExpressionSpec.ANGLE_BTW_PLANE_AND_VERSOR
task_pv_l.output_expression.id = "pv_l_angle_expr"
task_pv_l.output_expression.p1.object_frame = tframe
task_pv_l.output_expression.p1.entity = plane_spoon
task_pv_l.output_expression.p2.object_frame = "table"
task_pv_l.output_expression.p2.entity = zversor
task_pv_l.tr_gen = angle_traj 
task_pv_l.tr_gen_upper = pi/2
task_pv_l.tr_gen_lower = pi/8
g_pv_l.primary.append(task_pv_l)
m_pv_l = MonitorSpec()
m_pv_l.monitor_id = "M_pv_l_angle";
m_pv_l.monitored_expr = task_pv_l.output_expression;
m_pv_l.event_risen = "e_done";
m_pv_l.comparison_type = MonitorSpec.IN_INTERVAL;
m_pv_l.monitored_variable_type = MonitorSpec.POS;
m_pv_l.lower_bound = -0.015 + pi/8
m_pv_l.upper_bound = +0.015 + pi/2
g_pv_l.monitors.append(m_pv_l)
taskdb[g_pv_l.id] = g_pv_l


g_lp_l = TaskGoal()
g_lp_l.id = "lp_l_angle"
task_lp_l = ConstraintSpec()
task_lp_l.constraint_id = "lp_l_angle"
task_lp_l.behaviour.type = BehaviourSpec.POSITION_LIMIT
task_lp_l.behaviour.specification = 1.0
task_lp_l.output_expression.expression = GeometricExpressionSpec.ANGLE_BTW_LINE_AND_POINT
task_lp_l.output_expression.id = "lp_l_angle_expr"
task_lp_l.output_expression.p1.object_frame = tframe
task_lp_l.output_expression.p1.entity = point_xtrasl
task_lp_l.output_expression.p2.object_frame = "table"
task_lp_l.output_expression.p2.entity = zline
task_lp_l.tr_gen = angle_traj 
task_lp_l.tr_gen_upper = pi/2
task_lp_l.tr_gen_lower = 0
g_lp_l.primary.append(task_lp_l)
m_lp_l = MonitorSpec()
m_lp_l.monitor_id = "M_lp_l_angle";
m_lp_l.monitored_expr = task_lp_l.output_expression;
m_lp_l.event_risen = "e_done";
m_lp_l.comparison_type = MonitorSpec.IN_INTERVAL;
m_lp_l.monitored_variable_type = MonitorSpec.POS;
m_lp_l.lower_bound = -0.015 + 0
m_lp_l.upper_bound = +0.015 + pi/2
g_lp_l.monitors.append(m_lp_l)
taskdb[g_lp_l.id] = g_lp_l

# Choose any joint
g_ptp = TaskGoal()
g_ptp.id = "ptp_dist_any"
task_ptp = ConstraintSpec()
task_ptp.constraint_id = "ptp_dist_any"
task_ptp.behaviour.type = BehaviourSpec.POSITIONING
task_ptp.behaviour.specification = .5
task_ptp.output_expression.expression = GeometricExpressionSpec.POINT_TO_POINT_DISTANCE
task_ptp.output_expression.id = "ptp_dist_any_expr"
task_ptp.output_expression.p1.object_frame = "right_arm_4_link"
task_ptp.output_expression.p1.entity = point_origin
task_ptp.output_expression.p2.object_frame = "table"
task_ptp.output_expression.p2.entity = point_ztrasl
task_ptp.tr_gen = distance_traj
g_ptp.primary.append(task_ptp)
m_ptp = MonitorSpec()
m_ptp.monitor_id = "M_ptp_dist_any";
m_ptp.monitored_expr = task_ptp.output_expression;
m_ptp.event_risen = "e_done";
m_ptp.comparison_type = MonitorSpec.IN_INTERVAL;
m_ptp.monitored_variable_type = MonitorSpec.POS;
m_ptp.lower_bound = -0.015 + 0.15;
m_ptp.upper_bound = +0.015 + 0.15;
g_ptp.monitors.append(m_ptp)
taskdb[g_ptp.id] = g_ptp


######## End Define TASK
        
# define state idle
class idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['e_idle'])

    def execute(self, userdata):
        rospy.loginfo('Executing: Idle Task')
        return 'e_idle'
    
def main():
    rospy.init_node('test_client')
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    
    # smach introspection FSM
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # open actiionlib client
    client = SimpleRHClient('sot_interface',taskdb)
    
    # Open the container
    with sm:
      # Add states to the container
      smach.StateMachine.add('addExternalTable', addTransform('add','table','triangle_right_arm_link'),
                      transitions={'e_op_done':'addExternalSpoon','e_op_failed':'aborted'})
      
      smach.StateMachine.add('addExternalSpoon', addTransform('add','spoon','triangle_right_arm_flange_link'),
                      transitions={'e_op_done':'ptp_dist','e_op_failed':'aborted'}) 
                      #transitions={'e_done':'doNothing', 'e_aborted':'aborted'})
      
      smach.StateMachine.add('ptp_dist', singleEventClient(client, "ptp_dist", 1, 'e_done'),
                      transitions={'e_done':'ptp_dist_any', 'e_aborted':'aborted'})
                      #transitions={'e_done':'doNothing', 'e_aborted':'aborted'})
      
      smach.StateMachine.add('lp_dist', singleEventClient(client, "lp_dist", 1, 'e_done'),
                      #transitions={'e_done':'sp_dist', 'e_aborted':'aborted'})
                      transitions={'e_done':'doNothing', 'e_aborted':'aborted'})

      smach.StateMachine.add('sp_dist', singleEventClient(client, "sp_dist", 1, 'e_done'),
                      transitions={'e_done':'ll_dist', 'e_aborted':'aborted'})
                      #transitions={'e_done':'doNothing', 'e_aborted':'aborted'})
      
      smach.StateMachine.add('ll_dist', singleEventClient(client, "ll_dist", 1, 'e_done'),
                      transitions={'e_done':'pp_angle', 'e_aborted':'aborted'})
                      #transitions={'e_done':'doNothing', 'e_aborted':'aborted'})

      smach.StateMachine.add('pp_angle', singleEventClient(client, "pp_angle", 1, 'e_done'),
                      transitions={'e_done':'vv_angle', 'e_aborted':'aborted'})
                      #transitions={'e_done':'doNothing', 'e_aborted':'aborted'})

      smach.StateMachine.add('vv_angle', singleEventClient(client, "vv_angle", 1, 'e_done'),
                      transitions={'e_done':'pv_angle', 'e_aborted':'aborted'})
                      #transitions={'e_done':'doNothing', 'e_aborted':'aborted'})

      smach.StateMachine.add('pv_angle', singleEventClient(client, "pv_angle", 1, 'e_done'),
                      transitions={'e_done':'lp_angle', 'e_aborted':'aborted'})
                      #transitions={'e_done':'doNothing', 'e_aborted':'aborted'})

      smach.StateMachine.add('lp_angle', singleEventClient(client, "lp_angle", 1, 'e_done'),
                      transitions={'e_done':'ptp_l_dist', 'e_aborted':'aborted'})
                      #transitions={'e_done':'doNothing', 'e_aborted':'aborted'})

      smach.StateMachine.add('ptp_l_dist', singleEventClient(client, "ptp_l_dist", 1, 'e_done'),
                      transitions={'e_done':'lp_l_dist', 'e_aborted':'aborted'})
                      #transitions={'e_done':'doNothing', 'e_aborted':'aborted'})

      smach.StateMachine.add('lp_l_dist', singleEventClient(client, "lp_l_dist", 1, 'e_done'),
                      transitions={'e_done':'sp_l_dist', 'e_aborted':'aborted'})
                      #transitions={'e_done':'doNothing', 'e_aborted':'aborted'})

      smach.StateMachine.add('sp_l_dist', singleEventClient(client, "sp_l_dist", 1, 'e_done'),
                      transitions={'e_done':'ll_l_dist', 'e_aborted':'aborted'})
                      #transitions={'e_done':'doNothing', 'e_aborted':'aborted'})
      
      smach.StateMachine.add('ll_l_dist', singleEventClient(client, "ll_l_dist", 1, 'e_done'),
                      transitions={'e_done':'pp_l_angle', 'e_aborted':'aborted'})
                      #transitions={'e_done':'doNothing', 'e_aborted':'aborted'})

      smach.StateMachine.add('pp_l_angle', singleEventClient(client, "pp_l_angle", 1, 'e_done'),
                      transitions={'e_done':'vv_l_angle', 'e_aborted':'aborted'})
                      #transitions={'e_done':'doNothing', 'e_aborted':'aborted'})

      smach.StateMachine.add('vv_l_angle', singleEventClient(client, "vv_l_angle", 1, 'e_done'),
                      transitions={'e_done':'pv_l_angle', 'e_aborted':'aborted'})
                      #transitions={'e_done':'doNothing', 'e_aborted':'aborted'})

      smach.StateMachine.add('pv_l_angle', singleEventClient(client, "pv_l_angle", 1, 'e_done'),
                      transitions={'e_done':'lp_l_angle', 'e_aborted':'aborted'})
                      #transitions={'e_done':'doNothing', 'e_aborted':'aborted'})

      smach.StateMachine.add('lp_l_angle', singleEventClient(client, "lp_l_angle", 1, 'e_done'),
                      transitions={'e_done':'ptp_dist_any', 'e_aborted':'aborted'})
                      #transitions={'e_done':'doNothing', 'e_aborted':'aborted'})

      smach.StateMachine.add('ptp_dist_any', singleEventClient(client, "ptp_dist_any", 1, 'e_done'),
                      transitions={'e_done':'ptp_dist', 'e_aborted':'aborted'})
                      #transitions={'e_done':'doNothing', 'e_aborted':'aborted'})




      smach.StateMachine.add('doNothing', idle(),
                      transitions={'e_idle':'succeeded'})

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
