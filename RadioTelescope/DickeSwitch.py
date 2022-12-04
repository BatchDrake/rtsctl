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
import threading
from datetime import datetime

DICKE_SWITCH_DEFAULT_LOG_FILE_NAME = 'dicke.csv'
DICKE_SWITCH_DEFAULT_SWITCH_GPIO   = 21
DICKE_SWITCH_DEFAULT_ON_TIME       = 5
DICKE_SWITCH_DEFAULT_OFF_TIME      = 5

class DickeSwitch():
    def __init__(
        self, 
        on_time     = DICKE_SWITCH_DEFAULT_ON_TIME,
        off_time    = DICKE_SWITCH_DEFAULT_OFF_TIME,
        log_file    = DICKE_SWITCH_DEFAULT_LOG_FILE_NAME,
        switch_gpio = DICKE_SWITCH_DEFAULT_SWITCH_GPIO,
        logging     = True):

        self.on_time     = on_time
        self.off_time    = off_time
        self.gpio        = GPIO.get_platform_gpio()
        self.switch_gpio = switch_gpio
        self.running     = False
        self.logfp       = open(log_file, "a")
        self.logging     = logging
        self.mutex       = threading.RLock()
        self.logmutex    = threading.Lock()

        self.gpio.setup(self.switch_gpio, GPIO.OUT)
        self.set_ref()

    def set_logging(self, logging):
        self.logging = logging
    
    def log(self, string):
        if self.logging:
            with self.logmutex:
                now = datetime.now()
                timestamp = now.strftime("[%Y-%m-%d  %H:%M:%S]")
                print(f'{timestamp} -DICKE- {string}')
    
    def save_log(self):
        with self.mutex:
            now = datetime.now()
            ts = datetime.timestamp(now)
            self.logfp.write(f'{ts},{int(self.dicke_state)}\n')
            self.logfp.flush()

            if self.dicke_state:
                self.log('REF: \033[1;33mON\033[0m')
            else:
                self.log('REF: \033[1;32mOFF\033[0m')
        
    def set_ref(self):
        with self.mutex:
            self.dicke_state = True
            self.gpio.output(self.switch_gpio, 0)
            self.save_log()
        
    def set_antenna(self):
        with self.mutex:
            self.dicke_state = False
            self.gpio.output(self.switch_gpio, 1)
            self.save_log()
        
    def _toggle(self):
        with self.mutex:
            if self.running:
                if self.dicke_state: # Reference enabled?
                    self.set_antenna()
                else:
                    self.set_ref()
                
                t = threading.Timer(
                    self.on_time if self.dicke_state else self.off_time,
                    self._toggle)
                t.start()
            
    def start(self):
        with self.mutex:
            if not self.running:
                self.running = True
                self._toggle()
        
    def stop(self):
        with self.mutex:
            self.running = False

    def get_state(self):
        return self.dicke_state, self.running
    
