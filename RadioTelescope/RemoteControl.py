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

import socket
import sys
from datetime import datetime
import threading
import shlex

REMOTE_CONTROL_LISTEN_IP   = '0.0.0.0'
REMOTE_CONTROL_LISTEN_PORT = 9090

class RemoteControl:
    def __init__(
        self, 
        thermal_control,
        dicke_switch,
        host       = REMOTE_CONTROL_LISTEN_IP,
        port       = REMOTE_CONTROL_LISTEN_PORT,
        logging    = True):

        self.thermal_control = thermal_control
        self.dicke_switch    = dicke_switch
        self.logging         = logging
        self.host            = host
        self.port            = port
        self.running         = False
        self.accept_thread   = None
        self.client_thread   = None
        self.curr_client     = None
        self.logmutex        = threading.Lock()

    def set_logging(self, logging):
        self.logging = logging
    
    def log(self, string):    
        if self.logging:
            with self.logmutex:
                now = datetime.now()
                timestamp = now.strftime("[%Y-%m-%d  %H:%M:%S]")
                print(f'{timestamp} -RCTRL- {string}')
    
    def get_dicke_switch_state(self):
        ref, running = self.dicke_switch.get_state()

        ref_str = 'REF' if ref else 'ANTENNA'
        run_str = 'RUNNING' if running else 'STOPPED'
        return f'DICKE SWITCH STATE: {ref_str} ({run_str})'

    def get_thermal_control_state(self):
        setpoint, T_rx, T_amb, pwm = self.thermal_control.get_state()
        temp_state = self.thermal_control.get_temp_state()

        if self.thermal_control.is_enabled():
            string = f'THERMAL CONTROL STATE: SETPOINT {setpoint:2.2f} RX {T_rx:2.2f} AMBIENT {T_amb:2.2f} PWM {pwm:2.2f}%\n'
        else:
            string = f'THERMAL CONTROL STATE: SETPOINT {setpoint:2.2f} RX {T_rx:2.2f} AMBIENT {T_amb:2.2f} DISABLED\n'
        
        for i in temp_state:
            string += f'THERMAL SENSOR {i}: {temp_state[i]:2.2f}\n'

        return string
    
    def send(self, string):
        if self.curr_client is not None:
            self.curr_client.sendall(bytes(string, 'utf-8'))
    
    def parse_command(self, args):
        cmd = args[0].upper()
        if cmd == 'DICKE':
            if len(args) == 1:
                self.send(f'{self.get_dicke_switch_state()}\n')
            elif len(args) == 2:
                if args[1].upper() == 'ON':
                    self.dicke_switch.set_ref()
                    self.log('Client sets Dicke Switch to ON state (REF)')
                elif args[1].upper() == 'OFF':
                    self.dicke_switch.set_antenna()
                    self.log('Client sets Dicke Switch to OFF state (ANTENNA)')
                elif args[1].upper() == 'START':
                    self.dicke_switch.start()
                    self.log('Client sets Dicke Switch to RUNNING state')
                elif args[1].upper() == 'STOP':
                    self.dicke_switch.stop()
                    self.log('Client sets Dicke Switch to STOPPED state')
                else:
                    self.send('DICKE COMMAND INVALID (UNKNOWN ORDER)\n')
                    return

                self.send(f'{self.get_dicke_switch_state()}\n')
            else:
                self.send('DICKE COMMAND INVALID (TOO MANY ARGS)\n')
        elif cmd == 'TEMP':
            if len(args) == 1:
                self.send(self.get_thermal_control_state())
            elif len(args) == 2:
                if args[1].upper() == 'ON':
                    self.thermal_control.enable()
                    self.log('Client enabled thermal control')
                elif args[1].upper() == 'OFF':
                    self.thermal_control.disable()
                    self.log('Client disabled thermal control')
                else:
                    self.send('TEMP COMMAND INVALID (UNKNOWN ORDER)\n')
                    return

                self.send(self.get_thermal_control_state())
            else:
                self.send('TEMP COMMAND INVALID (TOO MANY ARGS)\n')
        elif cmd == 'CLOSE':
            self.send('BYE\n')
            self.log('Client left gracefully')
            self.curr_client.close()
            self.curr_client = None
        else:
            self.send(f'UNRECOGNIZED COMMAND "{cmd}"\n')
        
    def client_loop(self):
        try:
            self.send("Welcome to GonzaloScope's remote control daemon\n")
            self.send(f'{self.get_dicke_switch_state()}\n')
            self.send(f'{self.get_thermal_control_state()}\n')
            self.send("WAITING FOR ORDERS\n")

            command = ''

            while self.curr_client is not None:
                b = self.curr_client.recv(1)
                if len(b) == 0:
                    self.log('Client left')
                    self.curr_client.close()
                    self.curr_client = None
                    return

                if b[0] == 10:
                    if len(command) > 0:
                        args = shlex.split(command)
                        if len(args) > 0:
                            try:
                                self.parse_command(args)
                            except BaseException as exception:
                                self.send(f'COMMAND EXCEPTION: {str(exception)}\n')
                        command = ''
                elif b[0] != 13:
                    command += b.decode('utf-8')
                
        except BaseException as exception:
            self.log(f'Client connection vanished ({exception})')
            self.curr_client.close()
            self.curr_client = None
            return

    def accept_loop(self):
        self.log(f'Remote control daemon started on: {self.host}:{self.port}')
        while self.running:
            connection, client_address = self.socket.accept()
            try:
                self.log(f'New remote control client: {client_address[0]}:{client_address[1]}')
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
        