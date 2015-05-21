import roslib; roslib.load_manifest('actionlib')
import rospy
import actionlib
import smach

from rh_cop_msgs.msg import * 
from rh_cop_msgs.srv import *

__all__ = ['SimpleRHClient','singleEventClient','multipleEventClient','addTransform']

class SimpleRHClient():
  
    # Meta-states for this action
    WAITING_FOR_SERVER = 0
    INACTIVE = 1
    ACTIVE = 2
    PREEMPTING = 3
    COMPLETED = 4
    
    def __init__(
            self,
            #extra args
            name,
            taskdb
            ):
      
      self._taskdb = taskdb
      self._client = actionlib.SimpleActionClient(name,TaskAction)
      
      self._client.wait_for_server()
    
    def sendTask(self,name,monitor):
      #if not self._client.isServerConnected():
        #print "[RH_CLIENT: Server not found"
        #return 'aborted'
        
      self._client.send_goal(self._taskdb[name],feedback_cb=monitor)
        
    def deleteTask(self):
      self._client.cancel_goal()
      
    def execute(self, userdata):
      print "executing "+self._name
      self._counter += 1
      if self._counter < 100:
        return 'succeeded'

# class wrap container for a state, a lot of boilcode  
### singleEventClient: monitor policy over a specified event
class singleEventClient(smach.State):
  def __init__(self,client,taskname,sleeptime,event):
    smach.State.__init__(self, outcomes=[event,'e_aborted'])
    self._event = event
    self._sleeptime = sleeptime
    self._client = client
    self._taskname = taskname
    self._fb = TaskFeedback()
    
  def execute(self, userdata):
    self._client.sendTask(self._taskname,self.monitor_handle)
    exit_cond = False
    while ( (not rospy.is_shutdown()) and (not exit_cond)):
      rospy.sleep(self._sleeptime)
      for ev in self._fb.events:
        if ev == 'e_task_broken':
          self._client.deleteTask()
          return 'e_aborted'
        if ev==self._event:
          exit_cond = True
          break
      
    # Cancel the goal
    self._client.deleteTask()  
    return self._event
  
  def monitor_handle(self,fb):
    self._fb = fb
    
### multipleEventClient: monitor policy over a specified list of events
#### it returns if all the events have been satisfied (await mode)
# NOTE: in_event should be a list
class multipleEventClient(smach.State):
  def __init__(self,client,taskname,sleeptime,in_events,out_event):
    smach.State.__init__(self, outcomes=[out_event])
    self._out_event = out_event
    self._in_events  = in_events
    self._sleeptime = sleeptime
    self._client    = client
    self._taskname  = taskname
    self._fb = TaskFeedback()
    
  def execute(self, userdata):
    print self._taskname
    self._client.sendTask(self._taskname,self.monitor_handle)
    exit_cond = False
    while ( (not rospy.is_shutdown()) and (not exit_cond)):
      rospy.sleep(self._sleeptime)
      counter = 0
      for ev in self._in_events:
        for evfb in self._fb.events:
          if ev==evfb:
            counter += 1
            break
      
      if counter == len(self._in_events):
          exit_cond = True
          break
      
    # Cancel the goal
    self._client.deleteTask()  
    return self._out_event
  
  def monitor_handle(self,fb):
    self._fb = fb
    

# SMACH container which sends a service request about
# adding an expressiongraph transform
class addTransform(smach.State):
  def __init__(self,operation,o1,o2):
    smach.State.__init__(self, outcomes=['e_op_done','e_op_failed'])
    self._o1 = o1
    self._o2 = o2
    self._operation = operation
    
  def execute(self, userdata):
    rospy.wait_for_service('addFrame')
    try:
      add_transform = rospy.ServiceProxy('addFrame', ExternalFrame)
      resp = add_transform(self._operation,self._o1, self._o2)
      if resp.result == 0:
        return 'e_op_done'
      else:
        return 'e_op_failed'
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
      return 'e_op_failed'