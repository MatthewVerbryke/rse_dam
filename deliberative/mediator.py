#!/usr/bin/env python

"""
  Mediation submodule for dual-arm manipulaition with RSE.

  Copyright 2018 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.

  TODO: improve documentation across the board
"""


import rospy


class DualArmMediator(object):
    """
    A class to check the initial plan and monitor the execution of tasks
    in a dual-arm setup.
    """
    
    def __init__(self):
        """
        Initialize the mediator.
        """
        
        # Initialize rospy node
        rospy.init_node("dual_arm_mediator", anonymous=True)
        rospy.loginfo("Mediator node initialized")
        
        # Initialize cleanup for this node
        rospy.on_shutdown(self.cleanup)
        
    def check_trajectories(self):
        """
        Determine if the trajectory plans produced by the adapter and
        habitual layers are safe to execute
        """
        pass
        
    def monitor_execution(self):
        """
        Monitor the execution of the plans, and stop it if something goes
        wrong
        """
        pass

    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        # Log shutdown
        rospy.loginfo("Shutting down node 'dual_arm_mediator'")
        rospy.sleep(1)


if __name__ == "__main__":
    try:
        DualArmMediator()
    except rospy.ROSInterruptException:
        pass

