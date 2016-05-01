#!/usr/bin/env python

###############################################################################################
###############################################################################################
##                                                                                           ##
## Hove's Raspberry Pi Python Quadcopter Flight Controller.  Open Source @ GitHub            ##
## PiStuffing/Quadcopter under GPL for non-commercial application.  Any code derived from    ##
## this should retain this copyright comment.                                                ##
##                                                                                           ##
## Copyright 2014 - 2016 Andy Baker (Hove) - andy@pistuffing.co.uk                           ##
##                                                                                           ##
###############################################################################################
###############################################################################################

from Quadcopter import Quadcopter

import os

os.nice(-10)

if __name__ == '__main__':
    #-------------------------------------------------------------------------------------
    # Off we go!
    #-------------------------------------------------------------------------------------
    Quadcopter().go()
