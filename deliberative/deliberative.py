#!/usr/bin/env python

"""
  Deliberative layer module for a dual arm robot. 

  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.

  TODO: improve documentation across the board
"""


import rospy

# retrive files nessecary for websocket comms
file_dir = sys.path[0]
sys.path.append(file_dir + '/../..')
from rse_dam.communication import packing
from rse_dam.rse_dam_msgs.msg import HLtoDL, DLtoHL, OptoDL, DLtoOp
from rss_git_lite.common import rosConnectWrapper as rC
from rss_git_lite.common import ws4pyRosMsgSrvFunctions_gen as ws4pyROS


class DeliberativeModule(object):
    """
    The RSE Deliberative Module for a dual-armed robot.
    """
    
    def __init__(self):

        # Initialize rospy node
        rospy.init_node("deliberative_module")
        
        # Initialize communications for the layer
        self.comms_initialization()
        rospy.loginfo("Communcation interfaces setup")
        
    def comms_initialization(self):
        """
        Intialize communications for the deliberative module.
        
        TODO: TEST
        """
        
        # Setup publishers
        # NOTE: For now, we assume that the left arm movegroup is on the
        #       same computer as the DL. The right arm movegroup is assumed
        #       to be on a separate computer, requiring websocket comms.
        self.DL_to_left_HL_pub = rospy.Publisher("/deliberative/to_hl", DLtoHL, queue_size=1)
        self.DL_to_right_HL_pub = rC.RosMsg('ws4py', self.connection, "pub", "/deliberative/to_hl", "DLtoHL", self.pack_DLtoHL)
        self.DL_to_Op_pub = rospy.Publisher("/deliberative/to_hl", DLtoHl, queue_size=1)
        
        # Setup subscribers
        self.left_HL_to_DL_sub = rospy.Subscriber("/left_habitual/to_dl", HLtoDL, self.left_hb_cb)
        self.right_HL_to_DL_sub = rospy.Subscriber("/right_habitual/to_dl", HLtoDL, sefl.right_hb_cb)
        self.Op_to_DL = rospy.Subscriber("/Operator/to_dl", OptoDL, self.op_cb)
        
    def left_hb_cb(self, msg):
        """
        Callback for the left arm habitual module.
        """
        return msg
        
    def right_hb_cb(self, msg):
        """
        Callback for the right arm habitual module.
        """
        return msg    
    
    def op_cb(self, msg):
        """
        Callback for the operator.
        """
        return msg    
        
    def state_machine(self):
        """
        
        """
        
    def arbitration(self):
        """
        
        """
        
    def control(self):
        """
        
        """
        
    def tactics(self):
        """
        
        """
        
    def activity_manager(self):
        """
        
        """
        
    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        # Log shutdown
        rospy.loginfo("Shutting down node 'deliberative_module'")
        rospy.sleep(1)


if __name__ == "__main__":
    try:
        DeliberativeModule()
    except rospy.ROSInterruptException:
        pass

