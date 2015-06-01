#! /usr/bin/env python
import roslib
roslib.load_manifest('rh_cop')
import rospy

from rh_cop_msgs.msg import * 
from rh_cop_msgs.srv import *

roslib.load_manifest ('dynamic_graph_bridge_msgs')
from dynamic_graph_bridge_msgs.srv import RunCommand

class addTransform_service:
  
  run_command = None
  
  def __init__(self, name):
    server_name = name+'_server'
    rospy.init_node(server_name)
    rospy.wait_for_service ('run_command')
    self.run_command = rospy.ServiceProxy ('run_command', RunCommand)
    s = rospy.Service(name, ExternalFrame, self.handle_addTransform)
    
  def run_dyn(self, instruction):
    result = self.run_command(instruction)
    if not instruction == "": 
      rospy.logdebug ("run instruction: \"%s\"", instruction)
    if not result.standarderror == "":
      rospy.loginfo ("standarderror: \"%s\"", result.standarderror)
      return 1
    return 0
	
  def handle_addTransform(self, req):
    instruction = ""
    if req.operation == 'add':
      instruction = "ros.rosSubscribe.add('matrixHomoStamped','" + req.frame +"','"+req.frame+"') if not '"+req.frame+"' in ros.rosSubscribe.list() else None;"
      self.run_dyn(instruction)
      instruction = "BaseElement.frames['"+req.frame+"'] = ros.rosSubscribe.signal('"+req.frame+"');"
      err = self.run_dyn(instruction)
    elif req.operation == 'remove':
      rospy.loginfo("Not implemented yet")
      err = self.run_dyn(instruction)
      
    return err
if __name__ == '__main__':
  addTransform_service('addFrame')
  rospy.spin()