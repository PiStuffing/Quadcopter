#!/usr/bin/python

from __future__ import division
from smbus2 import SMBusWrapper, i2c_msg
import math
import socket
import struct
import sys
import select
import time

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

    while True:
        try:
            client.connect((addr, port))
        except:
            time.sleep(0.1)
        else:
            break
        continue

    client_fd = client.fileno()
    poll.register(client_fd, select.POLLIN | select.POLLPRI)

    ################################################################################################
    #                                  SET UP THE JOYSTICKS FSM
    ################################################################################################

    PASSIVE = 0
    TAKEOFF = 1
    FLYING = 2
    LANDING = 3
    POWEROFF = 4

    state = TAKEOFF

    status_quo = [0.0, 0.0, 0.0, 0.0]

    #----------------------------------------------------------------------------------------------
    # Acquire contact with the piDrone; only continue below once acquired.
    #----------------------------------------------------------------------------------------------

    with SMBusWrapper(1) as bus:

#       bus.write_byte_data(0x40, 0x76, 2)
#       bus.write_byte_data(0x41, 0x76, 2)

        while state != POWEROFF:

            ########################################################################################
            #                        HANDLE MESSAGES FROM THE SERVER
            ########################################################################################

            results = poll.poll(500) # milliseconds

            #---------------------------------------------------------------------------------------
            # Check whether there's I/O from RC
            #---------------------------------------------------------------------------------------
            for fd, event in results:
                assert fd == client_fd, "WTF HAPPENED HERE"

                #-----------------------------------------------------------------------------------
                # Has piDrone told piRC to start?
                #-----------------------------------------------------------------------------------
                raw = client.recv(unpack_size)
                assert (len(raw) == unpack_size), "Invalid data"

                #-----------------------------------------------------------------------------------
                # React on the action
                #-----------------------------------------------------------------------------------
                formatted = struct.unpack(unpack_format, raw)

                if formatted[0]: # is True
                    go_go_go = True
                    state = TAKEOFF
                    takeoff_time = time.time()

                else:
                    go_go_go = False
                    state = POWEROFF

            if not go_go_go:
                continue

            ########################################################################################
            #                          HANDLE MESSAGES FROM THE JOYSTICKS
            ########################################################################################

            #---------------------------------------------------------------------------------------
            # UD = Up / Down - upwards is positive
            # YR = Yaw Rate - anticlockwise is positive
            # LR = Left / Right - leftwards is positive
            # FB = Forwards / Backwards - forwards is positive
            #---------------------------------------------------------------------------------------

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

            #=======================================================================================
            # FSM INPUT, STATES, OUTPUT
            #=======================================================================================
            beep = False

            #---------------------------------------------------------------------------------------
            # Special cases for takeoff and landing.  Only between these states do we tell the
            # piDrone what to do.
            #---------------------------------------------------------------------------------------
            if abs(UD) < 20 and YR > 20 and abs(FB) < 20 and LR < -20:
                #-------------------------------------------------------------------------------
                # Take-off - we send fixed takeoff param regardless of joystick for next 3 seconds.
                #-------------------------------------------------------------------------------
                if state == PASSIVE:
                    print "takeoff-takeoff-takeoff"
                    state = TAKEOFF
                    beep = True
                    takeoff_time = time.time()

            if abs(UD) < 20 and YR < -20 and abs(FB) < 20 and LR > 20:
                #-----------------------------------------------------------------------------------
                # Send a shut-down to the piDrone, and shut our selves down once we get a confirmation
                # from the piDrone.
                #-----------------------------------------------------------------------------------
                if state == FLYING:
                    print "landing-landing-landing"
                    state = LANDING
                    beep = True
                    landing_time = time.time()
                elif state == PASSIVE:
                    print "poweroff-poweroff-poweroff"
                    state = POWEROFF
                    beep = True

            #=======================================================================================
            # FSM INPUTS
            #=======================================================================================
            if state == TAKEOFF:
                if time.time() - takeoff_time < 3.0: # seconds
                    UD = 0.33
                    YR = 0.0
                    LR = 0.0
                    FB = 0.0
                else:
                    UD, YR, FB, LR = status_quo
                    state = FLYING
                    beep = True

                    #-------------------------------------------------------------------------------
                    # Take the timestamp of flight, and initiate initial height
                    #-------------------------------------------------------------------------------
                    before = time.time()
                    height = 1.0

            elif state == FLYING:
                    #-------------------------------------------------------------------------------
                    # Joysticks are +/- 80, convert these to +/- 1m/s.  The exception is the yaw rate
                    # where +/-80 maps to +/- 90 degrees (pi/2) per second
                    #-------------------------------------------------------------------------------
                    UD /= 80
                    YR /= (80 * 2 / math.pi)
                    FB /= 80
                    LR /= 80

                    #-------------------------------------------------------------------------------
                    # Integrate the height increment and mark the landing timestamo
                    #-------------------------------------------------------------------------------
                    now = time.time()
                    dt = now - before
                    before = now
                    height += UD * dt #RC! Flawed: future velocity x historic dt
                    landing_period = height / 0.33 + 1 #RC! OK?

            elif state == LANDING:
                if time.time() - landing_time < landing_period: # seconds
                    UD = -0.33
                    YR = 0.0
                    FB = 0.0
                    LR = 0.0
                else:
                    UD, YR, FB, LR = status_quo
                    state = PASSIVE
                    beep = True
                    passive_time = time.time()

            elif state == PASSIVE:
                UD, YR, FB, LR = status_quo
                if time.time() - passive_time > 60.0:
                    state = POWEROFF
                    beep = True

            else:
                assert state == POWEROFF, "Should be on poweroff state here!"
                UD, YR, FB, LR = status_quo

            output = struct.pack(pack_format, UD, YR, FB, LR, state, beep)
            client.send(output)
            print "SENT:  UD = %f | YR = %f | FB = %f | LR = %f | status = %d | beep = %d" % (UD, YR, FB, LR, state, beep)


        else:
            #---------------------------------------------------------------------------------------
            # We get here when the server sends us the "running = False"
            #---------------------------------------------------------------------------------------
            client.close()

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
            results = poll.poll(500)

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

                print "RECEIVED: UD = %f | YR = %f | FB = %f | LR = %f | status = %d | beep = %d" % (UD, YR, FB, LR, state, beep)

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
