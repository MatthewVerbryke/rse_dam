#!/usr/bin/env python

"""
  Functions for merging directives in the arbitration component of RSE.
  
  Copyright 2020 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
  
  TODO: Add more complex merging techniques.
"""


def dumb_merge(directives):
    """
    'Merge' the directives by literally selecting the first listed 
    directive without considering anything else. Basically a 
    placeholder for more complex behavior.
    """
    
    act_directive = directives.pop(0)
    todo_directives = directives
    
    return act_directive, todo_directives
