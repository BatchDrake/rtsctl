#
# Copyright (c) 2022 Gonzalo J. Carracedo <BatchDrake@gmail.com>
#
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

import Adafruit_GPIO.GPIO as GPIO
import RPi.GPIO as RGPIO
from smbus2 import SMBus
from mlx90614 import MLX90614

from datetime import datetime

THERMAL_CONTROL_DEFAULT_SETPOINT    = 35
THERMAL_CONTROL_PREHEAT_DELTA       = 10 # Degrees below setpoint
THERMAL_CONTROL_DEFAULT_LOG_FILE    = 'loop.csv'
THERMAL_CONTROL_DEFAULT_ISAVE_FILE  = 'intfp.csv'
THERMAL_CONTROL_DEFAULT_AVERAGE_LEN = 10

TEMP_MAX_DELTA = 10

TEMP_PWM_RES   = 10e-6 # seconds
TEMP_GPIO      = 12
TEMP_PWM_MAX   = 100   # %
TEMP_MSR_TO    = .5    # s
TEMP_PWM_FREQ  = 100   # Hz

MLX90614_ADDR  = 0x5a

TEMP_PWM_DCMIN = 0
TEMP_PWM_DCMAX = 90

PID_KC    = .375          # Critical gain, measured with no integration
PID_DT    = TEMP_MSR_TO   # s

PID_KP    = .45 * PID_KC  # As recommended by Ziegler and Nichols
PID_Pc    = 4 * 60        # s, measured from cycle
PID_TI    = PID_Pc / 1.2  # s, Ziegler & Nichols

PID_KI    = PID_KP / (PID_TI) * PID_DT

class ThermalControl:
    def __init__(
        self, 
        setpoint   = THERMAL_CONTROL_DEFAULT_SETPOINT, 
        log_file   = THERMAL_CONTROL_DEFAULT_LOG_FILE,
        isave_file = THERMAL_CONTROL_DEFAULT_ISAVE_FILE, 
        avlen      = THERMAL_CONTROL_DEFAULT_AVERAGE_LEN,
        logging    = True):
        self.setpoint = setpoint
        
        self.last_T   = None
        self.num_T    = avlen
        self.T_p      = 0
        self.overheat = False
        self.safe_T   = setpoint - 5
        self.logging  = logging
        self.T_rx     = 0
        self.T_amb    = 0
        self.pwm      = 0
        self.deltas   = False
        self.preheat  = True
        self.preheat_target = setpoint - THERMAL_CONTROL_PREHEAT_DELTA
        
        # Initialize temperature sensor
        bus = SMBus(1)
        self.sensor = MLX90614(bus, address = MLX90614_ADDR)

        # Initialize PWM-based thermal control
        pgpio = GPIO.get_platform_gpio()

        pgpio.setup(TEMP_GPIO, GPIO.OUT)
        pgpio.output(TEMP_GPIO, 1) # Disabled
        
        self.tc = RGPIO.PWM(TEMP_GPIO, TEMP_PWM_FREQ)
        self.tc.start(100)

        # Initialize log file
        self.logfp = open(log_file, "a")

        # Initialize integral save file
        try:
            self.intfp = open(isave_file, "r+")
        except FileNotFoundError:
            self.intfp = open(isave_file, "w+")

        intfp_str = self.intfp.read()
        try:
            self.err_accum = float(intfp_str)
        except:
            self.err_accum = 0

        self.cooldown()
        
    def set_logging(self, logging):
        self.logging = logging
    
    def log(self, string):
        now = datetime.now()
        timestamp = now.strftime("[%Y-%m-%d  %H:%M:%S]")
    
        if self.logging:
            print(f'{timestamp} -THERM- {string}')
    
    def save_current_error(self):
        self.intfp.truncate(0)
        self.intfp.seek(0)
        self.intfp.write(f"{self.err_accum}")
        self.intfp.flush()

    def smooth_T(self, T):
        if self.last_T is None:
            self.last_T = [T] * self.num_T
        else:
            self.last_T[self.T_p] = T
            self.T_p = (self.T_p + 1) % (self.num_T)
            
            T = sum(self.last_T) / self.num_T

        return T
    
    def log_temp(self, T, pwm, T_amb = 0):
        ts = datetime.timestamp(datetime.now())
        self.logfp.write("{0},{1:2.3f},{2},{3}\n".format(ts, T, pwm, T_amb))
        self.logfp.flush()

    def cooldown(self):
        self.tc.ChangeDutyCycle(0)
    
    def adjust_PI(self, T):
        err = T - self.setpoint
        norm_delta = - (err * PID_KP + self.err_accum * PID_KI)
        self.err_accum += err

        return norm_delta
    
    def adjust_resistor(self, T, T_amb = None): 
        if T_amb is None:
            T_amb = T
        else:
            if self.overheat:
                if T_amb < self.safe_T:
                    self.log(f"{T:2.2f}º C - \033[1;32mSafe temperature reached.\033[0m Restarting thermal control (T_amb: {T_amb:2.2f}º C)")
                    self.overheat = False
            elif self.preheat:
                if T >= self.preheat_target:
                    self.log(f"{T:2.2f}º C - \033[1;32mPreheat target reached.\033[0m Starting PID (T_amb: {T_amb:2.2f}º C)")
                    self.preheat = False
            else:
                if T > self.setpoint and T_amb > self.setpoint:
                    self.log(f"{T:2.2f}º C - \033[1;32mLNB Overheat.\033[0m Stopping thermal control (T_amb: {T_amb:2.2f}º C)")
                    self.overheat = True
                    self.err_accum = 0

        if self.overheat:
            dc = 0 # Cooldown
            self.log(f"{T:2.2f}º C - \033[1;31mStanding by (overheat)\033[0m (T_amb: {T_amb:2.2f}º C) - PWM: {dc:3.1f}%")
        elif self.preheat:
            dc = 100 # Set to max
            self.log(f"{T:2.2f}º C - \033[1;33mPreheating\033[0m (T_amb: {T_amb:2.2f}º C) - PWM: {dc:3.1f}%")
        else:
            norm_delta = self.adjust_PI(T)
            raw_delta  = norm_delta
            
            # Saturate above this value
            if abs(norm_delta) > 1:
                norm_delta /= abs(norm_delta)

            dc = 100 * (norm_delta * (TEMP_PWM_MAX / 100))
            if dc < TEMP_PWM_DCMIN:
                dc = TEMP_PWM_DCMIN
            elif dc > TEMP_PWM_DCMAX:
                dc = TEMP_PWM_DCMAX

            if norm_delta < 0:
                self.log(f"{T:2.2f}º C - \033[1;36mCOOL\033[0m (T_amb: {T_amb:2.2f}º C) - PWM: {dc:3.1f}%")
            else:
                self.log(f"{T:2.2f}º C - \033[1;31mHEAT ({norm_delta * 100:5.1f}%)\033[0m (T_amb: {T_amb:2.2f}º C) - PWM: {dc:3.1f}%")

        # Update duty cycle
        self.tc.ChangeDutyCycle(dc)

        return dc

    def loop(self):
        T_rx  = self.sensor.get_obj_temp()
        T_amb = self.sensor.get_amb_temp()

        if self.deltas:
            DeltaT_rx = T_rx - self.T_rx
            DeltaT_amb = T_amb - self.T_amb
            if abs(DeltaT_rx) > TEMP_MAX_DELTA or abs(DeltaT_amb) > TEMP_MAX_DELTA:
                self.log(f'Unreasonable temperature delta ({DeltaT_rx:2.2f}º C, {DeltaT_amb:2.2f}º C). \033[1;36mEmergency cooling.\033[0m')
                self.cooldown()
                return
        elif abs(T_rx) > 80 or abs(T_amb) > 80:
            self.log(f'Unreasonable temperature ({T_rx:2.2f}º C, {T_amb:2.2f}º C). \033[1;36mEmergency cooling.\033[0m')
            self.cooldown()
            return
        
        self.T_rx   = T_rx
        self.T_amb  = T_amb
        self.T_rx   = self.smooth_T(self.T_rx)
        self.pwm    = self.adjust_resistor(self.T_rx, self.T_amb)
        self.deltas = True
        
    def get_state(self):
        return self.setpoint, self.T_rx, self.T_amb, self.pwm

    def save_state(self):
        self.log_temp(self.T_rx, self.pwm, self.T_amb)
        self.save_current_error()

