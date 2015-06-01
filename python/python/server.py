#! /usr/bin/env python
import roslib
roslib.load_manifest('actionlib')
roslib.load_manifest('rh_cop')
import rospy
import actionlib

from rh_cop_msgs.msg import *
from rh_cop_msgs.srv import *
from std_msgs.msg import Float64

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point

roslib.load_manifest('dynamic_graph_actionlib')
from dynamic_graph_actionlib.msg import *

roslib.load_manifest ('dynamic_graph_bridge_msgs')
from dynamic_graph_bridge_msgs.srv import RunCommand
from dynamic_graph_bridge_msgs.msg import Vector

import tools_v2 as tools
import threading

#MArkers lists to display visual information into Rviz
markers_list = {}
#MarkerArray object from Rviz where Markers are stored
marker_array = MarkerArray()
#Lock to avoid conflict in critical parts of code
callback_builder = threading.Lock()


class rh_cop_server_SoT:
  #rosservice to send command to dynamic graph
  run_command = None
  #Lock to change the array concerning the events
  event_lock = {}
  #Lock to change boolean when a Task is cancel
  preempt_lock = threading.Lock()
  #ActionLib FeeddBack
  _feedback = TaskFeedback()
  #Result FeedBack
  _result = TaskResult()

  def __init__(self, name):
    rospy.init_node(name)

    #Boolean to know if the task is canceled or not
    self.preempted = False
    #Create the actionlib server
    self._server = actionlib.SimpleActionServer(name, TaskAction, self.execute, False)
    #Register the method in case of the task is canceled
    #By the way, it happens when the client which monitors 
    #an expression "say" the task is done
    self._server.register_preempt_callback(self.preempt_callback)
    #Start the actionlib server
    self._server.start()

    #Before to continue, need to wait the service run_command
    #provide with the dynamic graph
    #This service permit to communicate with the dynamic graph
    rospy.loginfo(rospy.get_name() + " waiting for run_command")
    rospy.wait_for_service ('run_command')
    rospy.loginfo(rospy.get_name() + " run_command obtained")
    #When the service is avaible
    self.run_command = rospy.ServiceProxy ('run_command', RunCommand)

  #Method to send command to dynamic graph
  #Instruciton is a string that contains instructions to run into the dynamic graph
  def run_dyn(self, instruction):
    result = self.run_command(instruction)
    #Check if the return value is an error
    #If that the case you need to check what you send
    #by checking the dynamic graph terminal
    if not result.standarderror == "":
      rospy.loginfo ("standarderror: \"%s\"", result.standarderror)

  #Method executed when a goal is received
  def execute(self, goal):
    rospy.loginfo(rospy.get_name() + " goal received: "+goal.id) 
    self._feedback.events = []
    self._server.publish_feedback(self._feedback)
    #Create instructions to send to dynamic graph based on goal msg 
    #and [monitor_id, lambda_expr, event_risen]
    [instructions, monitor_stuffs, rviz_features] = tools.createInstructions(goal)
    #Send instructions to dynamic graph
    self.run_dyn(instructions[0])
    rospy.sleep(2)
    self.run_dyn(instructions[1])
    #Create a set of callback to monitored the expressions, each monitor_id is unique
    self.dynamic_callback = {}
    #self.dynamic_callback[monitor_id] = [value of monitored expr, event_risen]
    #Fill the dynamic_callback set and create the callback function assoctiated to the
    #topic concerning the expr monitored
    for elt in monitor_stuffs:
      self.dynamic_callback[elt[0]] = [False, elt[2], None]
      #Store the topic where data are sent to be able to unscribe it when
      #the goal is accomplished
      self.dynamic_callback[elt[0]][2] = rospy.Subscriber('/sot/'+elt[0], Float64,\
                      self.callback_builder(elt[0], elt[1])) 
    
    rospy.loginfo('Dynamic callbacks created:')
    rospy.loginfo(self.dynamic_callback)
    #This part is about markers in Rviz to allow a better understanding of what
    #is happened for the users
    self.dynamic_rviz = {}
    for elt in rviz_features:
      #As previous, a generic callback to create a specific callback method
      #This takes for argument (type, id, frame)
      callback = callback_builder_for_Marker(elt[2], elt[0], elt[1])
      #As previous the topic is stored to be able to unsuscribe it later
      self.dynamic_rviz[elt[0]] =\
                      rospy.Subscriber('/RvizFeatureSoTRH/'+elt[0], Vector, callback)
    
    while ((not rospy.is_shutdown()) and not self.preempted):
      #Reset the feedback msg
      #self._feedback.events = []
      #Create the feedback msg
      for evt in self.dynamic_callback:
        if self.dynamic_callback[evt][0]:
          #Create the feedback msg based on what happened in callback that are associated
          #to a monitor expression
          self._feedback.events.append(self.dynamic_callback[evt][1])
          self._server.publish_feedback(self._feedback)
         #Show the current state of the monitoring expression
      rospy.loginfo(self.dynamic_callback)
      #Change the value to change Hz
      rospy.sleep(1)
    #Task goal is okay so clien delete the task
    if self.preempted:
    #Need to remove the tasks of the SoT solver in dynamic graph
    #Need to delete topic useless
      for elt in monitor_stuffs:
        #Stop to suscribe to stop the callback
        self.dynamic_callback[elt[0]][2].unregister()
        #Stop to publsih in /sot/elt[0] topic
        self.run_command("ros.rosPublish.rm('"+elt[0]+"')")
      for elt in rviz_features:
        #Stop to sucribe to the callback
        self.dynamic_rviz[elt[0]].unregister()
        #Stop to publish into rviz topics
        self.run_command("ros.rosPublish.rm('"+elt[0]+"')")
      #Remove all task previously added
      self.run_command("removeTaskGoalRH(robot, solver)")
      self.run_command("clearAllRH(robot, solver)")
      for elt in marker_array.markers:
        #Remove all the markers in the MarkerArray send to 'Rviz'
        elt.action = 2
      #Clear all topics used by the dynamic graph to have a correct state
      self.run_command("robot.entities.clear()")
      self.run_command("robot.expressions.clear()")
      self.run_command("robot.taskGoals.clear()")
      self.run_command("robot.monitors.clear()")
      self._server.set_preempted()
      #Reset the value to False for the next goal
      self.preempted = False
    #As the client cancel the task when it reached this should be never execute
    else:
      self._result.res = True
      for evt in self.dynamic_callback:
        if not evt[1]:
          self._result.res = False
      rospy.loginfo('%s: Succeeded' % goal.id)
      self._server.set_succeeded(self._result)

  #Allow to cancel a task
  def preempt_callback(self):
    self.preempted = True

  #Method to create callback method dynamically
  def callback_builder(self, id_, lambda_expr):
    #Lock is usefull to allow multiple callback to write 
    #in the same structure and avoid conflicts
    #self.event_lock[id_] = threading.Lock() 
    def callback(data):
      #self.event_lock[id_].acquire()
      #Compute and store the result of the monitored expr
      self.dynamic_callback[id_][0] = lambda_expr(data.data)
      #if self.dynamic_callback[id_][0]:
      #  print data.data
      #self.event_lock[id_].release()
    return callback



i_distance = 0
#Callback global method to create a callback for a marker in Rviz
def callback_builder_for_Marker(type_, id_, frame):
  #As we store the callback method we can create only one callback at the same time
  callback_builder.acquire()
  #Create an unspecified marker
  m = Marker()
  m.ns = "shape"

  #Need to improve this part concerning the database concerning the correspondance name
  #between SoT and robot
  if frame == 'right-wrist':
      frame = 'right_arm_flange_link'
  #Associated the corresponding frame
  m.header.frame_id = frame
  #Get the id for the marker
  m.id = len(markers_list)
  #Check if a previous marker with the same id still here
  #It shouldn't be the case
  #If not, add the current to the list
  if not id_ in markers_list:
    markers_list[id_] = m.id
    marker_array.markers.append(m)

  #Callback to create a point
  def callback_Point(data):
    m = marker_array.markers[markers_list[id_]]
    m.header.stamp = rospy.Time.now()
    m.action = 0
    m.type = m.SPHERE
    p = Pose()
    #The position is stored in the 3 first values of the array
    p.position = Point(data.data[0], data.data[1], data.data[2])
    m.pose = p
    #To change the size of the point in Rviz
    sphere_radius = 0.05
    m.scale.x = sphere_radius
    m.scale.y = sphere_radius
    m.scale.z = sphere_radius
    m.color.r = 1
    m.color.g = 0
    m.color.b = 0
    m.color.a = 0.6
    m.lifetime = rospy.Duration(0)
    m.frame_locked = False


  def callback_Line(data):
    m = marker_array.markers[markers_list[id_]]
    m.header.stamp = rospy.Time.now()
    m.action = 0
    m.type = m.LINE_STRIP
    p0 = Point()
    p1 = Point()
    p2 = Point()
    x = data.data[0]
    y = data.data[1]
    z = data.data[2]
    s_x = data.data[3]
    s_y = data.data[4]
    s_z = data.data[5]
    p0.x = x - 5 * s_x
    p0.y = y - 5 * s_y
    p0.z = z - 5 * s_z
    p1.x = x
    p1.y = y
    p1.z = z
    p2.x = x + 5 * s_x
    p2.y = y + 5 * s_y
    p2.z = z + 5 * s_z
    m.points = []
    m.points.append(p0)
    m.points.append(p1)
    m.points.append(p2)
    m.scale.x = 0.02
    m.scale.y = 0.02
    m.scale.z = 0.02
    m.color.r = 0
    m.color.g = 0
    m.color.b = 1
    m.color.a = 0.6
    m.lifetime = rospy.Duration(0)
    m.frame_locked = False

  def callback_Plane(data):
    m = marker_array.markers[markers_list[id_]]
    m.header.stamp = rospy.Time.now()
    m.action = 0
    m.type = m.CUBE
    p = Pose()
    #The position is stored in the 3 first values of the array
    p.position = Point(data.data[0], data.data[1], data.data[2])
    m.pose = p
    #TODO: Take care of the orientation / normal
    m.scale.x = 2
    m.scale.y = 2
    m.scale.z = 0.001
    m.color.r = 1
    m.color.g = 0
    m.color.b = 0
    m.color.a = 0.6
    m.lifetime = rospy.Duration(0)
    m.frame_locked = False
    return

  def callback_Versor(data):
    #print data
    m = marker_array.markers[markers_list[id_]]
    m.header.stamp = rospy.Time.now()
    m.action = 0
    m.type = m.ARROW
    p0 = Point()
    p1 = Point()
    x = 0
    y = 0
    z = 0
    s_x = data.data[0]
    s_y = data.data[1]
    s_z = data.data[2]
    p1.x = (x + 2 * s_x) * 0.2
    p1.y = (y + 2 * s_y) * 0.2
    p1.z = (z + 2 * s_z) * 0.2
    p0.x = x
    p0.y = y
    p0.z = z
    m.points = []
    m.points.append(p0)
    m.points.append(p1)
    m.scale.x = 0.02
    m.scale.y = 0.05
    m.scale.z = 0.08
    m.color.r = 1
    m.color.g = 1
    m.color.b = 0 
    m.color.a = 0.6
    m.lifetime = rospy.Duration(0)
    m.frame_locked = False

  def callback_Distance(data):
    #print data
    global i_distance
    m = marker_array.markers[markers_list[id_]]
    m.header.stamp = rospy.Time.now()
    m.action = 0
    m.type = m.SPHERE
    p = Pose()
    #print data.data[0], data.data[1], data.data[2]
    
    p.position = Point(data.data[0], data.data[1], data.data[2])
    if data.data[3] == data.data[4] :
      scale = data.data[3]
    else :
      i_distance = i_distance + 1
      scale = data.data[3]
      if i_distance%2 == 0:
        scale = data.data[4]
    m.pose = p
    m.scale.x = scale * 2
    m.scale.y = scale * 2
    m.scale.z = scale * 2
    m.color.r = 0
    m.color.g = 1
    m.color.b = 0
    m.color.a = 0.2
    m.lifetime = rospy.Duration(0)
    m.frame_locked = False

  def callback_Angle(data):
    return

  def callback_Projection(data):
    """
    m = marker_array.markers[markers_list[id_]]
    m.header.stamp = rospy.Time.now()
    m.action = 0
    m.type = m.LINE_STRIP
    p = Pose()
    #print data.data[0], data.data[1], data.data[2]
    p.position = Point(data.data[0], data.data[1], data.data[2])
    m.pose = p
    m.scale.x = data.data[3] * 2
    m.scale.y = data.data[4] * 2
    m.scale.z = data.data[5] * 2
    m.color.r = 0
    m.color.g = 1
    m.color.b = 0
    m.color.a = 0.2
    m.lifetime = rospy.Duration(0)
    m.frame_locked = False
    """
    return

  callback = {}
  callback["Point"] = callback_Point
  callback["Versor"] = callback_Versor
  callback["Plane"] = callback_Plane
  callback["Line"] = callback_Line
  callback["Distance"] = callback_Distance
  callback["Angle"] = callback_Angle
  callback["Projection"] = callback_Projection

  callback_builder.release()
  return callback[type_]


if __name__ == '__main__':
  node = rh_cop_server_SoT('sot_interface')
  #Add feature to see elements in rviz
  #sub_Marker = rospy.Subscriber("/RvizFeatureSoTRH/Marker", Transform, callbackMarker)
  pub_Marker = rospy.Publisher("/visualization_marker_array", MarkerArray)
  reset_sot = False
  try: 
    reset_sot = rospy.get_param("/client/reset_sot")
  except:
    rospy.loginfo('param reset_sot not defined')

  #if reset_sot:
    #node.run_command("removeTaskGoalRH(robot, solver)")
    #node.run_command")

  while not rospy.is_shutdown():
    pub_Marker.publish(marker_array)
    rospy.sleep(0.1)

  rospy.spin()





