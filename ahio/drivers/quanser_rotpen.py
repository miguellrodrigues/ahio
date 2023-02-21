# -*- coding: utf-8; -*-
#
# Copyright (c) 2016 Álan Crístoffer
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import ahio.abstract_driver
import nidaqmx as nq
import numpy as np
from nidaqmx.constants import EncoderType
from enum import Enum


class ahioDriverInfo(ahio.abstract_driver.AbstractahioDriverInfo):
    NAME = 'Quanser Rotpen'
    AVAILABLE = True


class Encoder:
  def __init__(self, channel, name, steps_per_rev, pulses_per_rev, positive_dir):
    self._ctr_value = 0
    self._position = 0
    self._ang_position = .0
    self._last_ang_position = .0
    self._revolutions = .0

    self._last_ctr_value = self._ctr_value

    self._positive_dir = positive_dir
    self._steps_per_rev = steps_per_rev

    self.task = nq.Task()
    self.task.ci_channels.add_ci_ang_encoder_chan(
      channel,
      name,
      decoding_type=EncoderType.TWO_PULSE_COUNTING,
      pulses_per_rev=pulses_per_rev
    )

    self.task.start()

  def update(self, direction):
    self._last_ctr_value = self._ctr_value
    self._ctr_value = self.task.read()

    delta_ctr = (self._ctr_value - self._last_ctr_value)

    if direction == self._positive_dir:
      self._position += delta_ctr
    else:
      self._position -= delta_ctr

    self._revolutions = self._position / self._steps_per_rev

    self._last_ang_position = self._ang_position
    self._ang_position = self._revolutions * 2 * np.pi

    return self.read()

  def read(self, radians=True):
    if radians:
      return self._ang_position
    else:
      return np.rad2deg(self._ang_position)

  def destroy(self):
    self.task.stop()


class Driver(ahio.abstract_driver.AbstractDriver):
    Pins = Enum('Pins', 'ENCODERS OUTPUT')

    def setup(self):
        # verify if nidaqmx driver is installed
        try:
            nq.system.driver_version
        except:
            raise RuntimeError(
                'NIDAQmx driver not installed. Please install it to use this driver.'
            )
        

        def generate_tasks():
            task_dir = nq.Task()
            task_dir.di_channels.add_di_chan(
                "Dev1/port0/line0:7",
                "direction",
            )

            task_dir.start()

            task_voltage = nq.Task()
            task_voltage.ao_channels.add_ao_voltage_chan(
                "Dev1/ao0:1",
                "voltage",
            )

            return [task_dir, task_voltage]

        def destroy_tasks(tasks):
            for task in tasks:
                task.stop()

        self._tasks = generate_tasks()
        self.destroy_tasks = destroy_tasks
        
        for task in self._tasks:
            task.start()

        self.direction_task = self._tasks[0]
        self.write_task = self._tasks[1]

        base_encoder = Encoder(
            channel="Dev1/ctr0",
            name='base',
            steps_per_rev=1024,
            pulses_per_rev=1440,
            positive_dir=64
        )

        pendulum_encoder = Encoder(
            channel="Dev1/ctr1",
            name='pendulum',
            steps_per_rev=360,
            pulses_per_rev=4096,
            positive_dir=128
        )

        self._base_encoder = base_encoder
        self._pendulum_encoder = pendulum_encoder

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        pass

    def __create_pin_info(self, pid, pwm=False):
        is_input = pid.name == 'ENCODERS'

        return {
            'id': pid,
            'name': pid.name,
            'analog': {
                'input': is_input,
                'output': not is_input,
            },
            'digital': {
                'input': False,
                'output': False,
                'pwm': pwm
            }
        }

    def available_pins(self):
        return [self.__create_pin_info(pin) for pin in Driver.Pins]

    def _write(self, pin, value):
       self.write_task.write(value)

    def _read(self, pin):
        dir_value = int(self.direction_task.read())

        base_dir = dir_value & (1 << 6)
        pendulum_dir = dir_value & (1 << 7)

        self._base_encoder.update(base_dir)
        self._pendulum_encoder.update(pendulum_dir)

        return [
            self._base_encoder.read(),
            self._pendulum_encoder.read()
        ]
