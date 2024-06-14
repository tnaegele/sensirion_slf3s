#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
This script controls Sensirion SLF3S-XXXX flow sensors. This was built following
the instructions at https://sensirion.github.io/python-shdlc-driver/custom_commands.html#deriving-from-shdlccommand


@author: Tobias E. Naegele, 02/2024

Copyright (c) 2024 Tobias E. Naegele

'''

__author__ = "Tobias E. Naegele"
__maintainer__ = __author__
__version__ = "1.0"
__email__ = "github@tobiasnaegele.com"

from sensirion_shdlc_driver import ShdlcDeviceBase
from sensirion_shdlc_driver.command import ShdlcCommand
from struct import unpack
import time
import numpy as np
from serial.tools import list_ports
from .base_classes import *


class slf3s(ShdlcDeviceBase):
    '''
    '''
    def __init__(self, connection, slave_address=0):
        '''
        Connect and control Sensirion SLF3S type flow sensors via the SHDLC protocol.

        Parameters
        ----------
        connection : sensirion_shdlc_driver.ShdlcConnection
            ShdlcConnection, e.g. ShdlcConnection('COM3'). See example code
            
        slave_address : int, optional
            Slave address. The default is 0.

        Returns
        -------
        None.

        '''
        super(slf3s, self).__init__(connection, slave_address)
        self.flow_scale_factor = self.get_scale_factor()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()

    # start and stop measurement methods
    def start(self, water=True, sampling_rate=5):
        '''
        Start continuous measurement, flow rate can be read using get_last and get_full_buffer method

        Parameters
        ----------
        water : bool, optional
            Bool controlling whether the measured liquid is water. If false IPA calibration is used. The default is True.
        
        sampling_rate : int, optional
            Sampling rate in ms. The default is 5.

        Returns
        -------
        None.

        '''
        self.execute(
            StartContinuousMeasurement(
                water=water,
                sampling=sampling_rate))

    def stop(self):
        '''
        Stop the continuous measurement.

        '''
        self.execute(StopContinuousMeasurement())

    # read flow rate methods
    def _get_buffer(self):
        '''
        Get buffer command is a bit useless by itself so make it an internal method,
        use get_full_buffer instead.

        '''
        return self.execute(ExtendedMeasurementBuffer(self.flow_scale_factor))

    def get_last(self):
        '''
        Get the last measured flow rate

        Returns
        -------
        flow : float
            flow rate in ul/min.
       
        temp : float
            sensor temperature in degreesC.
        
        flag : bool
            1: air bubble detected, 0: no air bubble detected .

        '''
        
        flow,temp,flag =  self.execute(GetLastMeasurement(self.flow_scale_factor))
        return flow,temp,flag

    def get_full_buffer(self, sampling_rate):
        '''
        Read entire sensor buffer until empty.

        Parameters
        ----------
        sampling_rate : int
            Sampling rate in ms.

        Returns
        -------
        flow_values : numpy.ndarray
            array of flow rates in ul/min.
        
        temp_values : numpy.ndarray
            sensor temperatures in degrees C.
        
        flag_values : numpy.ndarray
            flags, 1: air bubble detected, 0: no air bubble detected.
        
        time_values : numpy.ndarray
            UNIX timestamps, be careful with that as these are only approximate timestamps.
        
        lost_packages_sum : int
            Number of lost packages due to sensor buffer overflow, read buffer more frequently to prevent data loss.

        '''
        flow_values = np.array([])
        temp_values = np.array([])
        flag_values = np.array([])
        lost_packages_sum = 0
        remaining_packages = 1
        while remaining_packages > 0:
            flow, temp, flags, remaining_packages, lost_packages = self._get_buffer()
            flow_values = np.concatenate((flow_values, flow), 0)
            temp_values = np.concatenate((temp_values, temp), 0)
            flag_values = np.concatenate((flag_values, flags), 0)
            lost_packages_sum += lost_packages

        # get current timestamp, interpret this as time when last data point
        # was measured
        read_time = time.time()
        sampling_s = sampling_rate / 1000  # calculate sampling rate in units of s
        l = len(flow_values)  # number of datapoints in buffer
        # create array of UNIX timestamps for each data point, do some acrobatics
        # as np.arange uses half open [start,stop) interval and we need the
        # opposite
        time_values = np.linspace(
            read_time - (l - 1) * sampling_s, read_time, l)

        return flow_values, temp_values, flag_values, time_values, lost_packages_sum

    # Totalizator methods
    def _set_totalizator_status(self, status):
        '''
        activates or deactivates totalizator, this is aninternal method:
            use start_totalizator or stop_totalizator instead

        Parameters
        ----------
        status : bool
            True: totalizator running, False: totalizator not running.

        Returns
        -------
        TYPE
            DESCRIPTION.

        '''
        return self.execute(SetTotalizatorStatus(status))

    def _get_totalizator_status(self):
        '''
        read totalizator status, internal method only

        Returns
        -------
        bool
            True: totalizator running, False: totalizator not running.

        '''
        return self.execute(GetTotalizatorStatus())

    def start_totalizator(self):
        '''
        Starts the totalizator and resets is value

        Returns
        -------
        start_time : float
            returns the time when totalizator was started as UNIX timestamp.

        '''
        self._set_totalizator_status(status=True)
        self.reset_totalizator()
        start_time = time.time()
        return start_time

    def stop_totalizator(self):
        '''
        Stops the totalizator ans resets its value

        Returns
        -------
        None.

        '''
        self._set_totalizator_status(status=False)
        self.reset_totalizator()

    def reset_totalizator(self):
        '''
        Sets the totalizator volume to 0 but leaves the totalizator running

        Returns
        -------
        bool
            True: totalizator running, False: totalizator not running.

        ''' 
        self.execute(ResetTotalizator())
        return self._get_totalizator_status()

    def get_total_volume(self, start_time):
        '''
        Reads out total dispensed volume, leaves value unchanged and keeps totalizator running

        Parameters
        ----------
        start_time : float
            Measurement start time as UNIX timestamp, use return value of start_totalizator method.

        Returns
        -------
        float
            total dispensed volume in µl.

        '''

        return self.execute(
            GetTotalizatorValue(
                start_time,
                self.flow_scale_factor))

    def set_sensor_type(self):
        '''
        Set the sensor type to SF06 in the cable's EEPROM

        Returns
        -------
        TYPE
            DESCRIPTION.

        '''
        return self.execute(SetSensorType(3))

    def get_scale_factor(self):
        ''' reads the sensor's flow rate scale factor' '''
        return float(self.execute(GetScaleFactor()))


def find_sensor_port():
    '''
    Experimental function which detects port of the sensor.
    TODO: handle multiple sensors

    Raises
    ------
    ConnectionError
        Connection error if no sensor found.

    Returns
    -------
    port : str
        Address of sensor. This can be COM port on Windows or a device address on Linux / MacOS.

    '''
    found_devices = list_ports.grep('0403:6001')
    try:
        port = next(found_devices).device
        return port
    except BaseException:
        raise ConnectionError('no sensor found')

