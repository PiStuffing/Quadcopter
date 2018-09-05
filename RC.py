#!/usr/bin/python

####################################################################################################
####################################################################################################
##                                                                                                ##
## Hove's Raspberry Pi Python Quadcopter Flight Controller.  Open Source @ GitHub                 ##
## PiStuffing/Quadcopter under GPL for non-commercial application.  Any code derived from         ##
## this should retain this copyright comment.                                                     ##
##                                                                                                ##
## Copyright 2012 - 2018 Andy Baker (Hove) - andy@pistuffing.co.uk                                ##
##                                                                                                ##
####################################################################################################
####################################################################################################

from __future__ import division
from smbus2 import SMBusWrapper, i2c_msg
import math
import socket
import struct
import sys
import select
import time
import RPi.GPIO as GPIO


def client():

    ################################################################################################
    #                              MAKE CONNECTION TO SERVER
    ################################################################################################

    poll = select.poll()

    go_go_go = False

    pack_format = "=ffffb?"
    pack_size = struct.calcsize(pack_format)

    unpack_format = "=?"
    unpack_size = struct.calcsize(unpack_format)

    client = socket.socket()
    addr = "192.168.42.1"
    port = 31415

    '''
    client_fd = client.fileno()
    poll.register(client_fd, select.POLLIN | select.POLLPRI)
    '''

    ################################################################################################
    #                                  SET UP THE JOYSTICKS FSM
    ################################################################################################

    RC_PASSIVE = 0
    RC_TAKEOFF = 1
    RC_FLYING = 2
    RC_LANDING = 3
    RC_DONE = 4
    RC_ABORT = 5

    state_name = ["PASSIVE", "TAKEOFF", "FLYING", "LANDING", "DONE", "ABORT"]

    status_quo = [0.0, 0.0, 0.0, 0.0]

    #-----------------------------------------------------------------------------------------------
    # Set up GPIO for button and buzzer
    #-----------------------------------------------------------------------------------------------
    GPIO.setmode(GPIO.BCM)

    GPIO_BUTTON = 10
    GPIO.setup(GPIO_BUTTON, GPIO.IN, GPIO.PUD_DOWN) # GPIO button pushed pulls GPIO high

    GPIO_BUZZER = 24
    GPIO.setup(GPIO_BUZZER, GPIO.OUT) # GPIO buzzer set high sounds the buzzer
    GPIO.output(GPIO_BUZZER, GPIO.LOW)

    #-----------------------------------------------------------------------------------------------
    # Acquire contact with the piDrone; only continue below once acquired.
    #-----------------------------------------------------------------------------------------------

    with SMBusWrapper(1) as bus:
        try:
            while True:

                #-----------------------------------------------------------------------------------
                # Connect to the server
                #-----------------------------------------------------------------------------------
                print "CONNECTING..."
                while True:
                    try:
                        client.connect((addr, port))
                    except:
                        time.sleep(0.1)
                    else:
                        break
                print "CONNECTED."        

                #-----------------------------------------------------------------------------------
                # We are connected to the piDrone.
                #-----------------------------------------------------------------------------------
                client_fd = client.fileno()
                poll.register(client_fd, select.POLLIN | select.POLLPRI)
                connected = True

                #-----------------------------------------------------------------------------------
                # Wait for a piDrone status update for go / stop
                #-----------------------------------------------------------------------------------
                go_go_go = False
                while connected:

                    ################################################################################
                    #                        HANDLE MESSAGES FROM THE SERVER
                    ################################################################################

                    #-------------------------------------------------------------------------------
                    # Sleep pending an input for the piDrone 
                    #-------------------------------------------------------------------------------
                    results = poll.poll(200) # milliseconds

                    #-------------------------------------------------------------------------------
                    # Check whether there's I/O from RC
                    #-------------------------------------------------------------------------------
                    for fd, event in results:
                        assert fd == client_fd, "WTF HAPPENED HERE"

                        #---------------------------------------------------------------------------
                        # Has piDrone told piRC to start?
                        #---------------------------------------------------------------------------
                        raw = client.recv(unpack_size)
                        assert (len(raw) == unpack_size), "Invalid data"

                        #---------------------------------------------------------------------------
                        # React on the action from the message from the piDrone
                        #---------------------------------------------------------------------------
                        formatted = struct.unpack(unpack_format, raw)

                        if formatted[0]: # is True
                            GPIO.add_event_detect(GPIO_BUTTON, GPIO.FALLING)

                            #-----------------------------------------------------------------------
                            # piDrone is connected, start the flight
                            #-----------------------------------------------------------------------
                            go_go_go = True
                            state = RC_TAKEOFF
                            takeoff_time = time.time()

                            print "GO GO GO"

                        else:
                            GPIO.remove_event_detect(GPIO_BUTTON)
                            connected = False

                            #-----------------------------------------------------------------------
                            # piDrone is disconnected
                            #-----------------------------------------------------------------------
                            go_go_go = False
                            poll.unregister(client_fd)
                            client.close()

                            print "STOP STOP STOP"
                    
                    #---------------------------------------------------------------------------
                    # If we're now not connected, report back.
                    #---------------------------------------------------------------------------
                    if not connected:
                        continue

                    #---------------------------------------------------------------------------
                    # Are we ready to go?                        
                    #---------------------------------------------------------------------------
                    if not go_go_go:
                        continue

                    #-------------------------------------------------------------------------------
                    # The piDrone is live and kicking; set the next flight increment.
                    #
                    # UD = Up / Down - upwards is positive
                    # YR = Yaw Rate - anticlockwise is positive
                    # LR = Left / Right - leftwards is positive
                    # FB = Forwards / Backwards - forwards is positive
                    #-------------------------------------------------------------------------------
                    msg = i2c_msg.read(0x40, 2)
                    bus.i2c_rdwr(msg)
                    data = list(msg)

                    assert (len(data) == 2), "Joystick 0 data len: %d" % len(data)

                    if (data[0] > 127):
                        UD = data[0] - 256
                    else:
                        UD = data[0]

                    if (data[1] > 127):
                        YR = data[1] - 256
                    else:
                        YR = data[1]

                    msg = i2c_msg.read(0x41, 2)
                    bus.i2c_rdwr(msg)
                    data = list(msg)

                    assert (len(data) == 2), "Joystick 1 data len: %d" % len(data)

                    if (data[0] > 127):
                        LR = data[0] - 256
                    else:
                        LR = data[0]

                    if (data[1] > 127):
                        FB = -(data[1] - 256)
                    else:
                        FB = -data[1]

                    #===============================================================================
                    # FSM INPUT, STATES, OUTPUT
                    #===============================================================================
                    beep = False

                    #-------------------------------------------------------------------------------
                    # Have we have a falling edge from the button?
                    #-------------------------------------------------------------------------------
                    if GPIO.event_detected(GPIO_BUTTON):
 
                        #---------------------------------------------------------------------------
                        # If we're flying, start landing otherise abort
                        #---------------------------------------------------------------------------
                        if state == RC_FLYING:
                            print "landing-landing-landing"
                            state = RC_LANDING
                            landing_time = time.time()
                        else:
                            print "abort-abort-abort"
                            state = RC_ABORT

                    #===============================================================================
                    # FSM INPUTS
                    #===============================================================================
                    if state == RC_TAKEOFF:
                        if time.time() - takeoff_time < 3.0: # seconds
                            UD = 0.33
                            YR = 0.0
                            LR = 0.0
                            FB = 0.0
                        else:
                            UD, YR, FB, LR = status_quo
                            state = RC_FLYING
                            beep = True

                            #-----------------------------------------------------------------------
                            # Take the timestamp of flight, and initiate initial height
                            #-----------------------------------------------------------------------
                            before = time.time()
                            height = 1.0

                    elif state == RC_FLYING:
                            #-----------------------------------------------------------------------
                            # Joysticks are +/- 80, convert these to +/- 1m/s.  The exception is the 
                            # yaw rate where +/-80 maps to +/- 90 degrees (pi/2) per second
                            #-----------------------------------------------------------------------
                            UD /= 80
                            YR /= (80 * 2 / math.pi)
                            FB /= 80
                            LR /= 80

                            #-----------------------------------------------------------------------
                            # Integrate the height increment and mark the landing timestamo
                            #-----------------------------------------------------------------------
                            now = time.time()
                            dt = now - before
                            before = now
                            height += UD * dt #RC! Flawed: future velocity x historic dt
                            landing_period = height / 0.33 + 1 #RC! OK?

                    elif state == RC_LANDING:
                        if time.time() - landing_time < landing_period: # seconds
                            UD = -0.33
                            YR = 0.0
                            FB = 0.0
                            LR = 0.0
                            beep = True
                        else:
                            UD, YR, FB, LR = status_quo
                            state = RC_DONE
                            beep = False

                    else:
                        assert state == RC_ABORT, "Should be on abort here!"
                        UD, YR, FB, LR = status_quo

                    output = struct.pack(pack_format, UD, YR, FB, LR, state, beep)
                    client.send(output)
                    print "SENT:  UD = %f | YR = %f | FB = %f | LR = %f | status = %s | beep = %d" % (UD, YR, FB, LR, state_name[state], beep)

                else:
                    print "DISCONNECTED DISCONNECTED DISCONNECTED"

        except KeyboardInterrupt:
            pass        

        else:
            pass

        finally:
            pass       

client()

'''
def server():

    poll = select.poll()

    unpack_format = "=ffffb?"
    unpack_size = struct.calcsize(unpack_format)

    pack_format = "=?"

    server = socket.socket()
    addr = "192.168.42.1"
    port = 31415
    server.bind((addr, port))
    server.listen(5)

    raw_input("Start the flight?")

    try:
        connection, addr = server.accept()
        connection_fd = connection.fileno()
        poll.register(connection_fd, select.POLLIN | select.POLLPRI)

        #-------------------------------------------------------------------------------------------
        # Tell the client to go-go-go!
        #-------------------------------------------------------------------------------------------
        output = struct.pack(pack_format, True)
        connection.send(output)

        #-------------------------------------------------------------------------------------------
        # Listen to the client and do what it says.
        #-------------------------------------------------------------------------------------------
        while True:
            results = poll.poll(200)

            #---------------------------------------------------------------------------------------
            # Check whether there's I/O from RC
            #---------------------------------------------------------------------------------------
            for fd, event in results:
                assert fd == connection_fd, "WHOSE FD IS THIS?"

                #-----------------------------------------------------------------------------------
                # Unpack the data received
                #-----------------------------------------------------------------------------------
                raw = connection.recv(unpack_size)
                assert (len(raw) == unpack_size), "Invalid data"

                #-----------------------------------------------------------------------------------
                # React on the action
                #-----------------------------------------------------------------------------------
                formatted = struct.unpack(unpack_format, raw)
                assert (len(formatted) == 6), "Bad formatted size"

                UD = formatted[0]
                YR = formatted[1]
                FB = formatted[2]
                LR = formatted[3]
                state = formatted[4]
                beep = formatted[5]

                print "RECEIVED: UD = %f | YR = %f | FB = %f | LR = %f | status = %s | beep = %d" % (UD, YR, FB, LR, state_name[status], beep)

    except KeyboardInterrupt:
        #-------------------------------------------------------------------------------------------
        # Tell the client to stop-stop-stop!
        #-------------------------------------------------------------------------------------------
        output = struct.pack(pack_format, False)
        connection.send(output)

    except Exception, err:
        print err

    finally:
       connection.close()


if len(sys.argv) != 2:
    print "Select DRONE or RC"
elif sys.argv[1] == "RC":
    client()
elif sys.argv[1] == "DRONE":
    server()
else:
    print "Select RC or DRONE"
'''
