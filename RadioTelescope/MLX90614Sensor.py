#
# Copyright (c) 2023 Gonzalo J. Carracedo <BatchDrake@gmail.com>
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

from .ThermalSensor import ThermalSensor
from smbus2 import SMBus
from mlx90614 import MLX90614

MLX90614_ADDR  = 0x5a

class MLX90614Sensor(ThermalSensor):
  def __init__(self, previous = None, sensor = 'object', address = MLX90614_ADDR, bus_id = 1):
    if previous is not None:
      self.sensor = previous.sensor
    else:
      bus = SMBus(bus_id)
      self.sensor = MLX90614(bus, address)

    sensor = sensor.lower()

    if sensor == 'object':
      self._get_temp = self.get_obj_temp
      self.desc      = 'MLX90614 (GAASFET)'
    elif sensor == 'ambient':
      self._get_temp = self.get_amb_temp
      self.desc      = 'MLX90614 (HOUSING)'
    else:
      raise RuntimeError(fr'Invalid sensor type {sensor}, only object and ambient are supported')
    
  def get_obj_temp(self):
    return self.sensor.get_obj_temp()

  def get_amb_temp(self):
    return self.sensor.get_amb_temp()
  
  def get_temp(self):
    return self._get_temp()

  def get_desc(self):
      return self.desc
    
