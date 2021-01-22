#!/usr/bin/env python3
# EC11 rotary encoder with button
# these files belong all together:
# RPi-Jukebox-RFID/scripts/rotary-encoder.py
# RPi-Jukebox-RFID/scripts/rotary_encoder.py
# RPi-Jukebox-RFID/misc/sampleconfigs/phoniebox-rotary-encoder.service.stretch-default.sample
# See wiki for more info: https://github.com/MiczFlor/RPi-Jukebox-RFID/wiki

import RPi.GPIO as GPIO
import logging

from signal import pause

import time
#import subprocess

logger = logging.getLogger(__name__)

class EC11RotaryEncoder:
    def __init__(self, pinA, pinB, pinC, 
                 fun_up=None, fun_down=None, fun_clk=None, 
                 fun_hold=None, hold_time=0.0,
                 name="EC11RotaryEncoder"):
        logger.debug('Initialize {name} EC11RotaryEncoder({pinA}, {pinB}, {pinC})'.format(
            pinA=pinA,pinB=pinB,pinC=pinC,
            name=name if name is not None else ''
        ))

        self.name = name

        # persist values
        self.pinA = pinA
        self.pinB = pinB
        self.pinC = pinC

        self.fun_down = fun_down
        self.fun_up = fun_up
        self.fun_clk = fun_clk
        self.fun_hold = fun_hold
        self.hold_time = hold_time

        # setup pins
        GPIO.setup(self.pinA, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.pinB, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.pinC, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self._is_active = False
        self.start()

    def __repr__(self):
        repr_str = '<{class_name}{object_name} on ({pinA},{pinB},{pinC}); is_active={is_active}>'
        return repr_str.format(
            class_name=self.__class__.__name__,
            object_name=':{}'.format(self.name) if self.name is not None else '',
            pinA=self.pinA, pinB=self.pinB, pinC=self.pinC,
            is_active=self.is_active)

    def start(self):
        logger.debug('Start Event Detection on ({},{},{})'.format(self.pinA, self.pinB, self.pinC))
        self._is_active = True
        GPIO.add_event_detect(self.pinA, GPIO.RISING, callback=self.rot_decode, bouncetime=10)
        GPIO.add_event_detect(self.pinC, GPIO.RISING, callback=self.rot_decode, bouncetime=10)

    def stop(self):
        logger.debug('Stop Event Detection on ({},{},{})'.format(self.pinA, self.pinB, self.pinC))
        GPIO.remove_event_detect(self.pinA)
        GPIO.remove_event_detect(self.pinC)
        self._is_active = False

    def __del__(self):
        if self.is_active:
            self.stop()

    @property
    def is_active(self):
        return self._is_active

    def rot_decode(self, pin):
        logger.debug('EC11 knob rotated')

        Switch_A = GPIO.input(self.pinA)
        Switch_B = GPIO.input(self.pinB)
        Switch_C = GPIO.input(self.pinC)
     
        if (Switch_C == 1):
            logger.debug("EC11 click down")

            # measure holding time
            startTime = time.perf_counter()

            if self.fun_hold is not None:
                logger.debug("Hold function defined. Checking for hold.")

                # Continously check if time is not over
                while True:
                    currentState = GPIO.input(self.pinC)
                    dt = time.perf_counter() - startTime

                    if self.hold_time < dt:
                        break

                    # Return if state does not match holding state
                    if (currentState != GPIO.HIGH):
                        logger.debug("Let go at {} seconds = regular click".format(dt))
                        self.fun_clk()
                        return

                    # Else: Wait

                if (currentState != GPIO.HIGH):
                    self.fun_clk()
                    return

                else:
                    logger.debug("Hold time reached ==> hold click")
                    self.fun_hold()
                    return

            else:
                logger.debug("No hold function ==> simple click")
                self.fun_clk()

        elif (Switch_A == 1) and (Switch_B == 0):
            logger.debug("EC11 direction -> ")
            self.fun_up()

            while Switch_B == 0:
                Switch_B = GPIO.input(self.pinB)
            while Switch_B == 1:
                Switch_B = GPIO.input(self.pinB)

            return
 
        elif (Switch_A == 1) and (Switch_B == 1):
            logger.debug("EC11 direction <- ")
            self.fun_down()

            while Switch_A == 1:
                Switch_A = GPIO.input(self.pinA)
            return

        else:
            return

if __name__ == "__main__":
    logging.basicConfig(level='DEBUG')

    logging.info("EC11 Rotary Encoder Test Program")
    GPIO.setwarnings(True)
    GPIO.setmode(GPIO.BCM)

    pinA = int(input('Right rotation pin = '))
    pinB = int(input('Left rotation pin = '))
    pinC = int(input('Click pin = '))

    #fun_up = lambda *args: subprocess.Popen(["amixer","set","Master","1+"])
    #fun_down = lambda *args: subprocess.Popen(["amixer","set","Master","1-"])

    fun_up = lambda *args: print("UP!")
    fun_down = lambda *args: print("DOWN!")
    fun_clk = lambda *args: print("CLICK!")
    fun_hold = lambda *args: print("HOLD!")

    ec11 = EC11RotaryEncoder(pinA, pinB, pinC, fun_up, fun_down, fun_clk, fun_hold, 2.0)

    print('running')
    pause()
