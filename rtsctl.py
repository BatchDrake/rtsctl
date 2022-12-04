#!/usr/bin/env python3
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

import RadioTelescope as rts
import time
import sys
import datetime
import os

THERMAL_CONTROL_TIMEOUT = .5
running = True

dicke_folder = datetime.datetime.now().strftime('/home/pi/%d-%b/')

if not os.path.exists(dicke_folder):
    try:
        os.mkdir(dicke_folder)
    except os.OSError as exc:
        print(f'Cannot create log folder: {exc}')
        sys.exit(1)


print(f"""
    
        Welcome to EA1IYR's RadioTelescope Control Daemon
-----------------------------------------------------------------
Log folder: {dicke_folder}
-----------------------------------------------------------------
""")

ds = rts.DickeSwitch(log_file = f"{dicke_folder}/dicke.csv")
ds.start()

tc  = rts.ThermalControl(log_file = f"{dicke_folder}/loop.csv", isave_file = f"{dicke_folder}/intfp.csv")
rc  = rts.RemoteControl(tc, ds)
rms = rts.RMSListener(tc, ds, rms_file = f"{dicke_folder}/rms.csv")

rc.start()
rms.start()

n = 0
while(running):
    try:
        time.sleep(THERMAL_CONTROL_TIMEOUT)
        tc.loop()

        n += 1
        if n % 10 == 0:
            tc.save_state()
        
    except KeyboardInterrupt:
        print("Interrupted!")
        running = False
    except BaseException as e:
        print(f'Unhandled exception! ({e})')
        print('Failsafe condition detected. Stopping thermal control.')
        tc.cooldown()

print("Loop broken, cooling down and restoring state")
tc.save_state()

tc.cooldown()

sys.exit(0)

