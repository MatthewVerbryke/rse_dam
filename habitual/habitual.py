#!/usr/bin/env python

"""
  TODO: module description

  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.

  TODO: improve documentation across the board
"""


import rospy


class HabitualModule(object):
    """
    
    """
    
    def __init__(self, side):
        """
        
        """
        
        # Initialize rospy node
        rospy.init_node("{}_hab_module".format(side), anonymous=True)
        
        # Initialize communications for the layer
        self.comms_initialization()
        
    def comms_initialization(self):
        """
        
        """
        
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
        
        
if __name__ == "__main__":
    try:
        HabitualModule()
    except rospy.ROSInterruptException:
        pass
