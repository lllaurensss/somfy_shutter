#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys, re, argparse
import fcntl
import os
import locale
import time
import datetime
import ephem
import pigpio
import socket
import signal, atexit, traceback
import logging, logging.handlers
import threading

class Config:
    def __init__(self):
        self.TXGPIO = 16
        self.Shutters = {
            "1": {'name': 'Rolluik1', 'durationDown': 20, 'durationUp': 20, 'intermediatePosition': 50, 'code': 1},
            "2": {'name': 'Rolluik2', 'durationDown': 20, 'durationUp': 20, 'intermediatePosition': 50, 'code': 1},
        }
        self.SendRepeat = 2

    def setCode(self, shutterId, new_code):
        self.Shutters[shutterId]['code'] = new_code

class Shutter:
    #Button values
    buttonUp = 0x2
    buttonStop = 0x1
    buttonDown = 0x4
    buttonProg = 0x8

    class ShutterState:
        position = None
        lastCommandTime = None
        lastCommandDirection = None

        def __init__(self, initPosition = None):
            self.position = initPosition
            self.lastCommandTime = time.monotonic()

        def registerCommand(self, commandDirection):
            self.lastCommandDirection = commandDirection
            self.lastCommandTime = time.monotonic()

    def __init__(self, config = None):
        super(Shutter, self).__init__()
        self.lock = threading.Lock()
        
        if config != None:
            self.config = config

        if self.config.TXGPIO != None:
           self.TXGPIO=self.config.TXGPIO
        else:
           self.TXGPIO=16 # 433.42 MHz emitter on GPIO 16
        self.frame = bytearray(7)
        self.callback = []
        self.shutterStateList = {}
        self.sutterStateLock = threading.Lock()

    def getShutterState(self, shutterId, initialPosition = None):
        with self.sutterStateLock:
            if shutterId not in self.shutterStateList:
                self.shutterStateList[shutterId] = self.ShutterState(initialPosition)
            return self.shutterStateList[shutterId]

    def getPosition(self, shutterId):
        state = self.getShutterState(shutterId, 0)
        return state.position

    def setPosition(self, shutterId, newPosition):
        state = self.getShutterState(shutterId)
        with self.sutterStateLock:
            state.position = newPosition
        for function in self.callback:
            function(shutterId, newPosition)

    def waitAndSetFinalPosition(self, shutterId, timeToWait, newPosition):
        state = self.getShutterState(shutterId)
        oldLastCommandTime = state.lastCommandTime

        print("["+self.config.Shutters[shutterId]['name']+"] Waiting for operation to complete for " + str(timeToWait) + " seconds")
        time.sleep(timeToWait)

        # Only set new position if registerCommand has not been called in between
        if state.lastCommandTime == oldLastCommandTime:
            print("["+self.config.Shutters[shutterId]['name']+"] Set new final position: " + str(newPosition))
            self.setPosition(shutterId, newPosition)
        else:
            print("["+self.config.Shutters[shutterId]['name']+"] Discard final position. Position is now: " + str(state.position))

    def lower(self, shutterId):
        state = self.getShutterState(shutterId, 100)

        print("["+self.config.Shutters[shutterId]['name']+"] Going down")
        self.sendCommand(shutterId, self.buttonDown, self.config.SendRepeat)
        state.registerCommand('down')

        # wait and set final position only if not interrupted in between
        timeToWait = state.position/100*self.config.Shutters[shutterId]['durationDown']
        t = threading.Thread(target = self.waitAndSetFinalPosition, args = (shutterId, timeToWait, 0))
        t.start()

    def lowerPartial(self, shutterId, percentage):
        state = self.getShutterState(shutterId, 100)

        print("["+self.config.Shutters[shutterId]['name']+"] Going down") 
        self.sendCommand(shutterId, self.buttonDown, self.config.SendRepeat)
        state.registerCommand('down')
        time.sleep((state.position-percentage)/100*self.config.Shutters[shutterId]['durationDown'])
        print("["+self.config.Shutters[shutterId]['name']+"] Stop at partial position requested")
        self.sendCommand(shutterId, self.buttonStop, self.config.SendRepeat)

        self.setPosition(shutterId, percentage)

    def rise(self, shutterId):
        state = self.getShutterState(shutterId, 0)

        print("["+self.config.Shutters[shutterId]['name']+"] Going up")
        self.sendCommand(shutterId, self.buttonUp, self.config.SendRepeat)
        state.registerCommand('up')

        # wait and set final position only if not interrupted in between
        timeToWait = (100-state.position)/100*self.config.Shutters[shutterId]['durationUp']
        t = threading.Thread(target = self.waitAndSetFinalPosition, args = (shutterId, timeToWait, 100))
        t.start()

    def risePartial(self, shutterId, percentage):
        state = self.getShutterState(shutterId, 0)

        print("["+self.config.Shutters[shutterId]['name']+"] Going up")
        self.sendCommand(shutterId, self.buttonUp, self.config.SendRepeat)
        state.registerCommand('up')
        time.sleep((percentage-state.position)/100*self.config.Shutters[shutterId]['durationUp'])
        print("["+self.config.Shutters[shutterId]['name']+"] Stop at partial position requested")
        self.sendCommand(shutterId, self.buttonStop, self.config.SendRepeat)

        self.setPosition(shutterId, percentage)

    def stop(self, shutterId):
        state = self.getShutterState(shutterId, 50)

        print("["+self.config.Shutters[shutterId]['name']+"] Stopping")
        self.sendCommand(shutterId, self.buttonStop, self.config.SendRepeat)

        print("["+shutterId+"] Previous position: " + str(state.position))
        secondsSinceLastCommand = int(round(time.monotonic() - state.lastCommandTime))
        print("["+shutterId+"] Seconds since last command: " + str(secondsSinceLastCommand))

        # Compute position based on time elapsed since last command & command direction
        setupDurationDown = self.config.Shutters[shutterId]['durationDown']
        setupDurationUp = self.config.Shutters[shutterId]['durationUp']

        fallback = False
        if state.lastCommandDirection == 'up':
          if secondsSinceLastCommand > 0 and secondsSinceLastCommand < setupDurationUp:
            durationPercentage = int(round(secondsSinceLastCommand/setupDurationUp * 100))
            print("["+shutterId+"] Up duration percentage: " + str(durationPercentage) + ", State position: "+ str(state.position))
            if state.position > 0: # after rise from previous position
                newPosition = min (100 , state.position + durationPercentage)
            else: # after rise from fully closed
                newPosition = durationPercentage
          else:  #fallback
            print("["+shutterId+"] Too much time since up command.")
            fallback = True
        elif state.lastCommandDirection == 'down':
          if secondsSinceLastCommand > 0 and secondsSinceLastCommand < setupDurationDown:
            durationPercentage = int(round(secondsSinceLastCommand/setupDurationDown * 100))
            print("["+shutterId+"] Down duration percentage: " + str(durationPercentage) + ", State position: "+ str(state.position))
            if state.position < 100: # after lower from previous position
                newPosition = max (0 , state.position - durationPercentage)
            else: # after down from fully opened
                newPosition = 100 - durationPercentage
          else:  #fallback
            print("["+shutterId+"] Too much time since down command.")
            fallback = True
        else: # consecutive stops
            print("["+shutterId+"] Stop pressed while stationary.")
            fallback = True

        if fallback == True: # Let's assume it will end on the intermediate position ! If it exists !
            intermediatePosition = self.config.Shutters[shutterId]['intermediatePosition']
            if (intermediatePosition == None) or (intermediatePosition == state.position):
                print("["+shutterId+"] Stay stationary.")
                newPosition = state.position
            else:
                print("["+shutterId+"] Motor expected to move to intermediate position "+str(intermediatePosition))
                if state.position > intermediatePosition:
                    state.registerCommand('down')
                    timeToWait = abs(state.position - intermediatePosition) / 100*self.config.Shutters[shutterId]['durationDown']
                else:
                    state.registerCommand('up')
                    timeToWait = abs(state.position - intermediatePosition) / 100*self.config.Shutters[shutterId]['durationUp']
                # wait and set final intermediate position only if not interrupted in between
                t = threading.Thread(target = self.waitAndSetFinalPosition, args = (shutterId, timeToWait, intermediatePosition))
                t.start()
                return

        # Save computed position
        self.setPosition(shutterId, newPosition)

        # Register command at the end to not impact the lastCommand timer
        state.registerCommand(None)

    # Push a set of buttons for a short or long press.
    def pressButtons(self, shutterId, buttons, longPress):
        self.sendCommand(shutterId, buttons, 35 if longPress else 1)

    def program(self, shutterId):
        self.sendCommand(shutterId, self.buttonProg, 1)

    def registerCallBack(self, callbackFunction):
        self.callback.append(callbackFunction)

    def sendCommand(self, shutterId, button, repetition): #Sending a frame
    # Sending more than two repetitions after the original frame means a button kept pressed and moves the blind in steps 
    # to adjust the tilt. Sending the original frame and three repetitions is the smallest adjustment, sending the original
    # frame and more repetitions moves the blinds up/down for a longer time.
    # To activate the program mode (to register or de-register additional remotes) of your Somfy blinds, long press the 
    # prog button (at least thirteen times after the original frame to activate the registration.
        print("sendCommand: Waiting for Lock")
        self.lock.acquire()
        try:
            print("sendCommand: Lock aquired")
            checksum = 0
            teleco = int(shutterId, 16)
            code = int(self.config.Shutters[shutterId]['code'])

            # print (codecs.encode(shutterId, 'hex_codec'))
            self.config.setCode(shutterId, code+1)

            pi = pigpio.pi() # connect to Pi

            if not pi.connected:
                exit()

            pi.wave_add_new()
            pi.set_mode(self.TXGPIO, pigpio.OUTPUT)

            print("Remote  :      " + "0x%0.2X" % teleco + ' (' + self.config.Shutters[shutterId]['name'] + ')')
            print("Button  :      " + "0x%0.2X" % button)
            print("Rolling code : " + str(code))
            print("")

            self.frame[0] = 0xA7;       # Encryption key. Doesn't matter much
            self.frame[1] = button << 4 # Which button did  you press? The 4 LSB will be the checksum
            self.frame[2] = code >> 8               # Rolling code (big endian)
            self.frame[3] = (code & 0xFF)           # Rolling code
            self.frame[4] = teleco >> 16            # Remote address
            self.frame[5] = ((teleco >>  8) & 0xFF) # Remote address
            self.frame[6] = (teleco & 0xFF)         # Remote address

            outstring = "Frame  :    "
            for octet in self.frame:
                outstring = outstring + "0x%0.2X" % octet + ' '
                print(outstring)

            for i in range(0, 7):
                checksum = checksum ^ self.frame[i] ^ (self.frame[i] >> 4)

            checksum &= 0b1111; # We keep the last 4 bits only

            self.frame[1] |= checksum;

            outstring = "With cks  : "
            for octet in self.frame:
                outstring = outstring + "0x%0.2X" % octet + ' '
            print(outstring)

            for i in range(1, 7):
                self.frame[i] ^= self.frame[i-1];

            outstring = "Obfuscated :"
            for octet in self.frame:
                outstring = outstring + "0x%0.2X" % octet + ' '
            print(outstring)

            #This is where all the awesomeness is happening. You're telling the daemon what you wanna send
            wf=[]
            wf.append(pigpio.pulse(1<<self.TXGPIO, 0, 9415)) # wake up pulse
            wf.append(pigpio.pulse(0, 1<<self.TXGPIO, 89565)) # silence
            for i in range(2): # hardware synchronization
                wf.append(pigpio.pulse(1<<self.TXGPIO, 0, 2560))
                wf.append(pigpio.pulse(0, 1<<self.TXGPIO, 2560))
            wf.append(pigpio.pulse(1<<self.TXGPIO, 0, 4550)) # software synchronization
            wf.append(pigpio.pulse(0, 1<<self.TXGPIO,  640))

            for i in range (0, 56): # manchester enconding of payload data
                if ((self.frame[int(i/8)] >> (7 - (i%8))) & 1):
                    wf.append(pigpio.pulse(0, 1<<self.TXGPIO, 640))
                    wf.append(pigpio.pulse(1<<self.TXGPIO, 0, 640))
                else:
                    wf.append(pigpio.pulse(1<<self.TXGPIO, 0, 640))
                    wf.append(pigpio.pulse(0, 1<<self.TXGPIO, 640))

            wf.append(pigpio.pulse(0, 1<<self.TXGPIO, 30415)) # interframe gap

            for j in range(1,repetition): # repeating frames
                for i in range(7): # hardware synchronization
                    wf.append(pigpio.pulse(1<<self.TXGPIO, 0, 2560))
                    wf.append(pigpio.pulse(0, 1<<self.TXGPIO, 2560))
                wf.append(pigpio.pulse(1<<self.TXGPIO, 0, 4550)) # software synchronization
                wf.append(pigpio.pulse(0, 1<<self.TXGPIO,  640))

                for i in range (0, 56): # manchester enconding of payload data
                    if ((self.frame[int(i/8)] >> (7 - (i%8))) & 1):
                        wf.append(pigpio.pulse(0, 1<<self.TXGPIO, 640))
                        wf.append(pigpio.pulse(1<<self.TXGPIO, 0, 640))
                    else:
                        wf.append(pigpio.pulse(1<<self.TXGPIO, 0, 640))
                        wf.append(pigpio.pulse(0, 1<<self.TXGPIO, 640))

                wf.append(pigpio.pulse(0, 1<<self.TXGPIO, 30415)) # interframe gap

            pi.wave_add_generic(wf)
            wid = pi.wave_create()
            pi.wave_send_once(wid)
            while pi.wave_tx_busy():
                pass
            pi.wave_delete(wid)

            pi.stop()
        finally:
            self.lock.release()
            print("sendCommand: Lock released")

class OperateShutter:

    def startPIGPIO(self):
        if sys.version_info[0] < 3:
            import commands
            status, process = commands.getstatusoutput('sudo pidof pigpiod')
            if status:  #  it wasn't running, so start it
                print("pigpiod was not running")
                commands.getstatusoutput('sudo pigpiod -l -m')  # try to  start it
                time.sleep(0.5)
                # check it again
                status, process = commands.getstatusoutput('sudo pidof pigpiod')
        else:
            import subprocess
            status, process = subprocess.getstatusoutput('sudo pidof pigpiod')
            if status:  #  it wasn't running, so start it
                print("pigpiod was not running")
                subprocess.getstatusoutput('sudo pigpiod -l -m')  # try to  start it
                time.sleep(0.5)
                # check it again
                status, process = subprocess.getstatusoutput('sudo pidof pigpiod')

        if not status:  # if it was started successfully (or was already running)...
            pigpiod_process = process
            print("pigpiod is running, process ID is {} ".format(pigpiod_process))

            try:
                pi = pigpio.pi()  # local GPIO only
                if not pi.connected:
                    print("pigpio connection could not be established. Check logs to get more details.")
                    return False
                else:
                    self.LogInfo("pigpio's pi instantiated.")
            except Exception as e:
                start_pigpiod_exception = str(e)
                print("problem instantiating pi: {}".format(start_pigpiod_exception))
        else:
            print("start pigpiod was unsuccessful.")
            return False
        return True

class ConsoleOutput:

    def print_sep_line(self):
        print("#############################################################")

    def print(self, text: str) -> None:
        print(text)

if __name__ == "__main__":
    version = "0.0.1"

    console = ConsoleOutput()
    operate_shutter = OperateShutter()
    config = Config()
    shutter = Shutter(config)

    console.print_sep_line()
    console.print("Shutter control by Laurens:")
    console.print(f"Shutter control version: {version}")

    if not operate_shutter.startPIGPIO():
        console.print("Not able to start PIGPIO") 
        sys.exit(1)

    #console.print("program remote: ")
    shutter.program("1")
    #console.print("shutter programmed")
    shutter.rise("1")
    #time.sleep(30)
    #shutter.lower("1")
    