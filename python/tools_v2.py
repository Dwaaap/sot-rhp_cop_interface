#! /usr/bin/env python
import roslib
roslib.load_manifest('robohow_common_msgs')
import rospy

from rh_cop_msgs.msg import GeometricExpressionSpec as gm
from rh_cop_msgs.msg import BehaviourSpec as bm
import map_frames_joints as m
 
type_Element = ["", "Point", "Versor", "Line", "Plane"]

targets_value_monitor = {}
def addTargetValueMonitor(exp_id, target):
  targets_value_monitor[exp_id] = target

def getTargetValueMonitor(exp_id):
  return targets_value_monitor[exp_id]

#Allow to construct the instructions to send to the dynamic-graph
instructions_dyn_list = []
def addInstructionDyn(instr):
  instructions_dyn_list.append(instr)

#Allow to constrcut the instructions to send to Rviz
instructions_rviz_list = []
def addInstructionRviz(instr):
  instructions_rviz_list.append(instr)

#Allow to construct the monitoring part
datas_for_monitors = []
def addMonitor(m_id, lambda_fun, e_risen):
  datas_for_monitors.append((m_id, lambda_fun, e_risen))

def createInstructions(goal):

  parseGeometricsConstraints(goal)
  parseMonitors(goal.monitors)
  setTaskGoal(goal.id, goal)

  return [compact(instructions_dyn_list), datas_for_monitors, instructions_rviz_list]

def parseGeometricsConstraints(goal):
  global instructions_dyn_list
  instructions_dyn_list[:] = []
  global instructions_rviz_list
  instructions_rviz_list[:] = []

  rospy.loginfo(instructions_dyn_list)
  rospy.loginfo("Create Primary Constraints")
  parseConstraint(goal.primary)

  rospy.loginfo("Create Auxiliary Constraints")
  parseConstraint(goal.auxiliary)

  rospy.loginfo("Create Safety Constraints")
  parseConstraint(goal.safety)
  
def parseMonitors(monitors):
  global datas_for_monitors
  datas_for_monitors[:] = []

  for monitor in monitors:
    [m_id, exp_id, e_risen, lambda_function] = parseMonitorSpec(monitor)

    addInstructionDyn("ros.rosPublish.add('double', '"+m_id+"', '/sot/"+m_id+"') if not '"+m_id+"' in ros.rosPublish.list() else None")
   
    addInstructionDyn("errorNorm_"+exp_id+" = Norm_of_vector('op_"+exp_id+"')")
    addInstructionDyn("addVector_"+exp_id+" = Add_of_vector('add_"+exp_id+"')")
    addInstructionDyn("plug(robot.features[robot.expressions['"+exp_id+"'][0]].error, addVector_"+exp_id+".sin1)")
     
    addInstructionDyn("addVector_"+exp_id+".sin2.value = "+intToStr(getTargetValueMonitor(exp_id)))
    addInstructionDyn("plug(addVector_"+exp_id+".sout, errorNorm_"+exp_id+".sin)")
    addInstructionDyn("plug(errorNorm_"+exp_id+".sout, ros.rosPublish.signal('"+m_id+"'))")
    addInstructionDyn("robot.monitors['"+m_id+"']=('"+exp_id+"', '"+e_risen+"')")
    
    addInstructionDyn("robot.plug['errorNorm_"+exp_id+"'] = errorNorm_"+exp_id)
    addInstructionDyn("robot.plug['addVector_"+exp_id+"'] = addVector_"+exp_id)
    addMonitor(m_id, lambda_function, e_risen)


def parseConstraint(constraintspeclist):
  for constraint in constraintspeclist:

    constraint_id = constraint.constraint_id
    lower = constraint.tr_gen_lower
    upper = constraint.tr_gen_upper
    [exp_id, expression, p1, p2] = parseGeometricExpression(constraint.output_expression)
    [frame_1, type_1, obj_1] = parseEntity(p1)
    [frame_2, type_2, obj_2] = parseEntity(p2)

    entity_1 = exp_id+"_"+frame_1+"_"+type_Element[type_1]
    entity_2 = exp_id+"_"+frame_2+"_"+type_Element[type_2]

    [type_traj, target, vel_max, acc_max, duration] = parseTrajectorySpec(constraint.tr_gen) 
    
    
    _frame_1_ = frame_1 if not frame_1 in m.mapping else m.mapping[frame_1]
    rospy.loginfo(frame_1) 
    _frame_2_ = frame_2 if not frame_2 in m.mapping else m.mapping[frame_2]
    rospy.loginfo(frame_2) 
    
    
    parseFeature(entity_1, _frame_1_, type_Element[type_1], obj_1)
    parseFeature(entity_2, _frame_2_, type_Element[type_2], obj_2)

    [type_behaviour, gain_stifness] = parseBehaviourSpec(constraint.behaviour)

    createDynEntitiesForRviz(entity_1, frame_1, type_Element[type_1])
    createDynEntitiesForRviz(entity_2, frame_2, type_Element[type_2])

    createMarkersForRviz(entity_1, entity_2, frame_1, frame_2, expression, type_behaviour, lower, upper, target)
    
    if type_behaviour == bm.POSITIONING:
      addTargetValueMonitor(exp_id, target)
    elif type_behaviour == bm.POSITION_LIMIT:
      addTargetValueMonitor(exp_id, (0,))
    elif type_behaviour == bm.INTERACTION:
      print "Not implemented yet"
    
    if type_1 == 0 or type_2 == 0:
      rospy.logerror("Entity type: UNKNOW. WARNING!")

    addInstructionDyn("createTaskRH(robot, '"+constraint_id+"', '"+\
        entity_1+"', '"+entity_2+"', '"+str(expression)+"', "+\
        intToStr(type_behaviour)+", "+intToStr(gain_stifness)+", "+\
        intToStr(type_traj)+", "+intToStr(lower)+", "+intToStr(upper)+", "+intToStr(target)+", "+intToStr(vel_max)+", "+intToStr(acc_max)+", "+intToStr(duration)+")")

    addInstructionDyn("robot.expressions['"+exp_id+"']=('"+constraint_id+"', '"+entity_1+"', '"+entity_2+"')")


def parseFeature(entity, frame, type_elt, obj):
  instruction = "createEntityRH(robot, "+type_elt+"Element('"+entity+"', robot, '"+frame

  if type_elt == "":
    rospy.logerror("Entity type: UNKNOW. WRANING!")
  elif type_elt == "Point":
    instruction = instruction + "', position=" + vector3ToStr(obj)+"))"
  elif type_elt == "Versor":
    instruction = instruction + "', versor=" + vector3ToStr(obj)+"))"
  elif type_elt == "Line":
    instruction = instruction + "', normal=" + vector3ToStr(obj.dir) +", position =" + vector3ToStr(obj.p)+"))"
  elif type_elt == "Plane":
    instruction = instruction + "', normal=" + vector3ToStr(obj.normal) +", position=" + vector3ToStr(obj.p)+"))"

  addInstructionDyn(instruction)

def createDynEntitiesForRviz(entity, frame, type_elt):

  _entity_ = entity.replace('-', '_')
  
  addInstructionDyn("ros.rosPublish.add('vector', 'rviz_"+_entity_+"', '/RvizFeatureSoTRH/"+_entity_+"') if not 'rviz_"+_entity_+"' in ros.rosPublish.list() else None")
  addInstructionDyn("convert_"+_entity_+" = EntityToMatrix('convert_"+entity+"')")

  if type_elt == "":
    rospy.logerror("Entity type: UNKNOW. WRANING!")
  elif type_elt == "Point":
    addInstructionDyn("convert_"+_entity_+".sin1.value = robot.entities['"+entity+"'].position")
    addInstructionDyn("convert_"+_entity_+".sin2.value = robot.entities['"+entity+"'].position")
  elif type_elt == "Versor":
    addInstructionDyn("convert_"+_entity_+".sin1.value = robot.entities['"+entity+"'].versor")
    addInstructionDyn("convert_"+_entity_+".sin2.value = robot.entities['"+entity+"'].versor")
  elif type_elt == "Line" or type_elt == "Plane":
    addInstructionDyn("convert_"+_entity_+".sin1.value = robot.entities['"+entity+"'].position")
    addInstructionDyn("convert_"+_entity_+".sin2.value = robot.entities['"+entity+"'].normal")
  elif type_elt == "Plane":
    addInstructionDyn("convert_"+_entity_+".sin1.value = robot.entities['"+entity+"'].position")
    addInstructionDyn("convert_"+_entity_+".sin2.value = robot.entities['"+entity+"'].normal")

  addInstructionDyn("plug(convert_"+_entity_+".sout, ros.rosPublish.signal('rviz_"+_entity_+"'))")
    
  addInstructionDyn("robot.plug['convert_"+_entity_+"'] = convert_"+_entity_)
  addInstructionRviz([_entity_, frame, type_elt])
 

def createMarkersForRviz(entity_1, entity_2, frame_1, frame_2, expression, type_behaviour, lower, upper, target):

  if entity_1[-5:] == 'Point':
    entity = entity_1
    frame = frame_1
  elif entity_2[-5:] == 'Point':
    entity = entity_2
    frame = frame_2
  else:
    entity = entity_1
    frame = frame_1

  _entity_ = entity.replace('-', '_')
  expr = ""
  if expression == gm.POINT_TO_POINT_DISTANCE or expression == gm.LINE_POINT_DISTANCE or expression == gm.SURFACE_POINT_DISTANCE or expression == gm.LINE_LINE_DISTANCE:
    expr = 'Distance'
  elif expression == gm.ANGLE_BTW_PLANES or expression == gm.ANGLE_BTW_VERSORS or expression == gm.ANGLE_BTW_PLANE_AND_VERSOR or expression == gm.ANGLE_BTW_LINE_AND_POINT:
    expr = 'Angle'
  elif expression == gm.PROJECTION_OF_POINT_ON_LINE:
    expr = 'Projection'
  
  _entity_expr_ = _entity_+'_'+expr
  addInstructionDyn("ros.rosPublish.add('vector', 'rviz_"+_entity_expr_+"', '/RvizFeatureSoTRH/"+_entity_expr_+"') if not 'rviz_"+_entity_expr_+"' in ros.rosPublish.list() else None")
  addInstructionDyn("convert_"+_entity_expr_+" = EntityToMatrix('convert_"+entity+"')")
  
  addInstructionDyn("convert_"+_entity_expr_+".sin1.value = robot.entities['"+entity+"'].position if 'position' in dir(robot.entities['"+entity+"']) else robot.entities['"+entity+"'].versor")
  
  if expr == 'Distance' or expr == 'Angle':
    if type_behaviour == bm.POSITIONING:
      targetVectStr = str((target, target, target))
    elif type_behaviour == bm.POSITION_LIMIT:
      targetVectStr = str((lower, upper, target))
    addInstructionDyn("convert_"+_entity_expr_+".sin2.value = "+targetVectStr)
    if type_behaviour == bm.INTERACTION:
      print "Not implement"
  elif expr == 'Projection':
    print "Not implement"

  addInstructionDyn("plug(convert_"+_entity_+".sout, ros.rosPublish.signal('rviz_"+_entity_expr_+"'))")
  addInstructionRviz([_entity_expr_, frame, expr])

  addInstructionDyn("robot.plug['convert_"+_entity_+"'] = convert_"+_entity_)


def setTaskGoal(_id, goal):

  list_tasks = "("
  for elt in goal.primary:
    list_tasks = list_tasks + "'" + elt.constraint_id + "', "
  for elt in goal.auxiliary:
    list_tasks = list_tasks + "'" + elt.constraint_id + "', "
  for elt in goal.safety:
    list_tasks = list_tasks + "'" + elt.constraint_id + "', "
  list_tasks = list_tasks + ")"
  addInstructionDyn("setTaskGoalRH(solver, robot, '"+_id+"', "+list_tasks+")")


def parseGeometricExpression(output_expression):
  return [output_expression.id, output_expression.expression, output_expression.p1, output_expression.p2]

def parseBehaviourSpec(behaviour):
  return [behaviour.type, behaviour.specification]

def parseTrajectorySpec(trajectory):
  return [trajectory.type, trajectory.target, trajectory.vel_max, trajectory.acc_max, trajectory.duration]

def parseEntity(f):

  frame = f.object_frame
  type_ = f.entity.type

  if type_ == 0:
    obj = None
  elif type_ == 1:
    obj = f.entity.point
  elif type_ == 2:
    obj = f.entity.versor
  elif type_ == 3:
    obj = f.entity.line
  elif type_ == 4:
    obj = f.entity.plane

  if type_ == 0:
    rospy.loginfo("Entity type: UNKNOW")

  return [frame, type_, obj]

def parseMonitorSpec(monitor):

  m_id = monitor.monitor_id
  exp_id = monitor.monitored_expr.id
  e_risen = monitor.event_risen

  comp = monitor.comparison_type
  type_ = monitor.monitored_variable_type
  lower_bound = monitor.lower_bound
  upper_bound = monitor.upper_bound

  if comp == monitor.LESS:
    fun = lambda x: x < lower_bound
    print "fun = lambda x: x < "+str(lower_bound)
  elif comp == monitor.MORE:
    fun = lambda x: x > upper_bound
    print "fun = lambda x: x > "+str(upper_bound)
  elif comp == monitor.IN_INTERVAL:
    fun = lambda x: x > lower_bound and x < upper_bound
    print "fun = lambda x: x > "+str(lower_bound)+" and x < "+str(upper_bound)
  elif comp == monitor.OUT_INTERVAL:
    fun = lambda x: x < lower_bound or x > upper_bound
    print "fun = lambda x: x < "+str(upper_bound)+" or x > "+str(upper_bound)
  else:
    rospy.logerror("Unknow comparaison type")

  return [m_id, exp_id, e_risen, fun]

def vector3ToStr(vec):
    st = "(%f, %f, %f)" % (vec.x, vec.y, vec.z)
    return st;

def vectorToStr(vec):
    st = '('
    for i in range(0, len(vec)):
      s = "%f, " % vec[i]
      st = st + s
    st = st + ')'
    return st

def intToStr(val):
  return '(%f, )'% val

def compact(instr):
  computeStr = ""
  for elt in instr:
    computeStr = computeStr + elt + ";"
  return computeStr
