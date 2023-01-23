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

from datetime import datetime

THERMAL_CONTROL_DEFAULT_SETPOINT    = 35
THERMAL_CONTROL_PREHEAT_DELTA       = 10 # Degrees below setpoint
THERMAL_CONTROL_DEFAULT_LOG_FILE    = 'loop.csv'
THERMAL_CONTROL_DEFAULT_ISAVE_FILE  = 'intfp.csv'
THERMAL_CONTROL_DEFAULT_AVERAGE_LEN = 10

TEMP_MAX_DELTA    = 10
TEMP_MAX_ABSOLUTE = 80

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
        self.preheat  = True
        self.preheat_target = setpoint - THERMAL_CONTROL_PREHEAT_DELTA
        
        self.enabled  = True
        self.sensor_table   = {}
        self.temp_table     = {}
        self.lna_sensor     = None
        self.amb_sensor     = None

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
        

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def is_enabled(self):
        self.enabled
    
    def register_sensor(self, name, sensor):
        self.sensor_table[name] = sensor
        self.temp_table[name]   = None

    def set_lna_sensor(self, name):
        if name is None:
            self.lna_sensor = None
        else:
            if name not in self.sensor_table:
                raise RuntimeError(fr'Sensor {name} has not been registered')
            self.lna_sensor = name
        
        self.last_T = None

    def set_amb_sensor(self, name):
        if name is None:
            self.amb_sensor = None
        else:
            if name not in self.sensor_table:
                raise RuntimeError(fr'Sensor {name} has not been registered')
            self.amb_sensor = name


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

    def get_temp_state(self):
        return self.temp_table.copy()
        
    def get_temps(self):
        ok = True
        for i in self.sensor_table:
            sensor = self.sensor_table[i]
            temp = sensor.get_temp()
            sensor_ok = True
            if temp > TEMP_MAX_ABSOLUTE:
                self.log(f'{sensor.get_desc()}: unreasonable temperature reading ({temp:2.2f}º C)')
                sensor_ok = False
            elif self.temp_table[i] is not None:
                deltaT = temp - self.temp_table[i]
                if abs(deltaT) > TEMP_MAX_DELTA:
                    self.log(f'{sensor.get_desc()}: unreasonable temperature increment ({deltaT:+2.2f}º C)')
                    sensor_ok = False

            if sensor_ok:
                self.temp_table[i] = temp
            
            ok = ok and sensor_ok            
        
        return ok
    
    def loop(self):
        if not self.get_temps():
            self.log(f'Error condition in temperature sensing. \033[1;36mEmergency cooling.\033[0m')
            self.cooldown()
            return
        
        if self.lna_sensor is None:
            self.log(f'No LNA temperature sensor defined (yet). Standing by...')
            self.cooldown()
            return

        if self.amb_sensor is not None:
            self.T_amb = self.temp_table[self.amb_sensor]
        else:
            self.T_amb = None
        
        T_rx = self.temp_table[self.lna_sensor]

        self.T_rx   = self.smooth_T(T_rx)


        if not self.enabled is None:
            self.log(f'Thermal control disabled (LNA temp: {self.T_rx:2.2f}º C)')
            self.cooldown()
            return

        self.pwm    = self.adjust_resistor(self.T_rx, self.T_amb)
        
    def get_state(self):
        return self.setpoint, self.T_rx, self.T_amb, self.pwm

    def save_state(self):
        self.log_temp(self.T_rx, self.pwm, self.T_amb)
        self.save_current_error()

