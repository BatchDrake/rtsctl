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

from datetime import datetime
import numpy as np
import socket
import threading

RMS_LISTEN_IP         = '0.0.0.0'
RMS_LISTEN_PORT       = 9999
RMS_DICKE_DELAY_MS    = 1000
RMS_DEFAULT_RMS_FILE  = 'rms.csv'

class RMSListener:
    def __init__(
        self,
        thermal_control,
        dicke_switch,
        host = RMS_LISTEN_IP,
        port = RMS_LISTEN_PORT,
        delay_ms = RMS_DICKE_DELAY_MS,
        rms_file = RMS_DEFAULT_RMS_FILE,
        logging = True):

        self.thermal_control = thermal_control
        self.dicke_switch    = dicke_switch
        self.logging         = logging
        self.host            = host
        self.port            = port
        self.delay_ms        = delay_ms
        self.running         = False
        self.prev_ref ,_     = dicke_switch.get_state()
        self.switch_time     = datetime.now()
        self.accept_thread   = None
        self.client_thread   = None
        self.curr_client     = None
        self.logmutex        = threading.Lock()
        self.data_list       = []

        self.last_ref_reading = None
        self.last_ant_reading = None

        # Initialize RMS save file
        try:
            self.fp = open(rms_file, "r+")
        except FileNotFoundError:
            self.fp = open(rms_file, "w+")

    def log(self, string):
        if self.logging:
            with self.logmutex:
                now = datetime.now()
                timestamp = now.strftime("[%Y-%m-%d  %H:%M:%S]")
                print(f'{timestamp} -RMS-S- {string}')
    
    def process_data(self):
      data  = np.array(self.data_list)
      means = np.mean(data, axis = 0)
      stds  = np.std(data, axis = 0)

      mean_power = means[0]
      mean_T_rx  = means[1]
      mean_T_amb = means[2]

      std_power  = stds[0]
      std_T_rx   = stds[1]
      std_T_amb  = stds[2]

      return mean_power, mean_T_rx, mean_T_amb, std_power, std_T_rx, std_T_amb

    def log_reading(self):
        time_now = datetime.now()

        mean_power, mean_T_rx, mean_T_amb, std_power, std_T_rx, std_T_amb = \
          self.process_data()

        self.fp.write(
          f'{self.switch_time.timestamp()},{time_now.timestamp()},{len(self.data_list)},' +
          f'{1 if self.prev_ref else 0},' +
          f'{mean_power},{mean_T_rx},{mean_T_amb},' +
          f'{std_power},{std_T_rx},{std_T_amb}\n')

        reading = (
          self.switch_time, time_now, len(self.data_list),
          mean_power, mean_T_rx, mean_T_amb,
          std_power, std_T_rx, std_T_amb)
        
        if self.prev_ref:
          self.last_ref_reading = reading
        else:
          self.last_ant_reading = reading
        
        self.fp.flush()

        return reading

    def get_last_ref(self):
      return self.last_ref_reading
    
    def get_last_ant(self):
      return self.last_ref_reading
    
    def parse_line(self, args):
        state ,_ = self.dicke_switch.get_state()
        time_now = datetime.now()

        # Switch detected! Log readings so far
        if self.prev_ref != state:
          reading = self.log_reading()
          self.data_list = [] # Reset current reading list

          # Switch back to REF, this marks the end of an ANT reading
          if state:
            ref_reading = self.get_last_ref()
            if ref_reading is not None:
              pwr = reading[3] / ref_reading[3]
              self.log(fr'------------------------------------------------------')
              self.log(f'\033[1mCALIBRATED POWER READING: {10 * np.log10(pwr):.3f} dB\033[0m')
              self.log(fr'------------------------------------------------------')
              # TODO: Calibrate a temperature model?

          # Remember switch time and the state we are currently in
          self.switch_time = datetime.now()
          self.prev_ref = state

        delta = (time_now - self.switch_time)
        delay_ms = delta.seconds * 1e3 + delta.microseconds * 1e-3
        
        if delay_ms > self.delay_ms:
          power = float(args[2])
          setpoint, T_rx, T_amb, pwm = self.thermal_control.get_state()
          self.data_list.append([power, T_rx, T_amb, setpoint, pwm])

    def client_loop(self):
        identified = False
        self.data_list = []
        self.last_ant_reading = None
        self.last_ref_reading = None

        command = ''

        try:
            while self.curr_client is not None:
                b = self.curr_client.recv(1)
                if len(b) == 0:
                    self.log('suscli RMS: left')
                    self.curr_client.close()
                    self.curr_client = None
                    return

                if b[0] == 10: # New line
                    # New command found!
                    if len(command) > 0:
                        args = command.split(",")
                        argc = len(args)
                        command = ''

                        if not identified:
                          if args[0] == "RATE":
                            identified = True
                          else:
                            raise Exception('Client does not look like Suscli RMS')
                        else:
                          if args[0] == "DESC":
                            self.log(f'\033[1;35mClient self-identification:\033[0;1m {args[1]}\033[0m')
                          elif argc == 4:
                              try:
                                  self.parse_line(args)
                              except BaseException as exception:
                                  self.log("Malformed line")
                    command = ''
                elif b[0] != 13:
                    command += b.decode('utf-8')
                
        except BaseException as exception:
            self.log(f'Client connection vanished ({exception})')
            self.curr_client.close()
            self.curr_client = None
            return

    def accept_loop(self):
        self.log(f'RMS listener daemon started on:   {self.host}:{self.port}')
        while self.running:
            connection, client_address = self.socket.accept()
            try:
                self.log(f'New RMS client: {client_address[0]}:{client_address[1]}')
                if self.curr_client is not None:
                    self.log('Kicking previous client from system')
                    self.send('KICKED\n')
                    self.curr_client.close()
                    self.client_thread.join()
                elif self.client_thread is not None:
                    self.client_thread.join()
                
                self.curr_client = connection
                self.client_thread = threading.Thread(target = self.client_loop)
                self.client_thread.start()
            except BaseException as exception:
                self.log(f'Failed to create client thread ({exception})')
                # Clean up the connection
                connection.close()
                if self.curr_client == connection:
                    self.curr_client = None
                
    def start(self): 
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.host, self.port))
        self.socket.listen(1)
        self.running = True
        self.accept_thread = threading.Thread(target = self.accept_loop)
        self.accept_thread.start()