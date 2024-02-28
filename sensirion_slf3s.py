#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
This script controls Sensirion SLF3S-XXXX flow sensors. This was built following
the instructions at https://sensirion.github.io/python-shdlc-driver/custom_commands.html#deriving-from-shdlccommand


@author: Tobias E. Naegele, 02/2024

Copyright (c) 2024 Tobias E. Naegele

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
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


class StartContinuousMeasurement(ShdlcCommand):
    def __init__(self, water, sampling):
        ''' water = true: water
            water =  false: IPA
            sampling = sampling rate in ms'''
        sampling_bytes = int.to_bytes(
            sampling,
            length=2,
            byteorder='big')  # convert dampling rate in ms to bytes
        if water:
            data = sampling_bytes + b"\x36\x08"  # water
        else:
            data = sampling_bytes + b"\x36\x15"  # IPA
        super(StartContinuousMeasurement, self).__init__(
            id=0x33,  # Command ID as specified in the device documentation
            data=data,
            max_response_time=0.001,  # Maximum response time in Seconds
        )

    def interpret_response(self, data):
        # Convert the received raw bytes to the proper data types
        return 0


class StopContinuousMeasurement(ShdlcCommand):
    def __init__(self):
        super(StopContinuousMeasurement, self).__init__(
            id=0x34,  # Command ID as specified in the device documentation
            data=b'',
            max_response_time=0.001,  # Maximum response time in Seconds
        )

    def interpret_response(self, data):
        # Convert the received raw bytes to the proper data types
        return 0


class GetLastMeasurement(ShdlcCommand):
    ''' read last flow rate value from buffer'''

    def __init__(self, flow_scale_factor):
        super(GetLastMeasurement, self).__init__(
            id=0x35,  # get last measurement
            # data=b"",  # get only the flow rate
            data=b"\x22",  #
            max_response_time=0.2,  # Maximum response time in Seconds
        )
        # see data sheet, (ul/min)**-1
        self.flow_scale_factor = flow_scale_factor
        self.temp_scale_factor = 200  # in (degrees celsius)**-1

    def interpret_response(self, data):
        # Convert the received raw bytes to the proper data types
        # flow rate, >h corresponds to signed integer
        flow_rate = unpack('>h', data[0:2])[0] / self.flow_scale_factor
        temp = unpack('>h', data[2:4])[0] / \
            self.temp_scale_factor  # sensor temperature
        flag = unpack('>h', data[4:6])[0]  # air in system flag
        return flow_rate, temp, flag


class ExtendedMeasurementBuffer(ShdlcCommand):
    ''' read out the device buffer, better when plotting data
        returns flow rates, sensor temperatures, and air in sensor flags'''

    def __init__(self, flow_scale_factor):
        super(ExtendedMeasurementBuffer, self).__init__(
            id=0x36,
            data=b"\x03",  #
            max_response_time=0.2,  # Maximum response time in Seconds
        )
        # see data sheet, (ul/min)**-1
        self.flow_scale_factor = flow_scale_factor
        self.temp_scale_factor = 200  # in (degrees celsius)**-1

    def _unpack_int(self, x, signed=True):
        '''converts bytes objects to signed integers'''
        if isinstance(x, list):
            return [
                int.from_bytes(
                    i,
                    byteorder='big',
                    signed=signed) for i in x]
        else:
            return int.from_bytes(x, byteorder='big', signed=signed)

    def interpret_response(self, data):
        lost_packages = self._unpack_int(
            data[0:4], signed=False)  # number of packages lost
        # remaining packages which need to be read out with another call of
        # this method
        remaining_packages = self._unpack_int(data[4:6])
        # number of signals per data package, should be always 3
        interlaced_data = self._unpack_int(data[6:8])
        data_points = [[data[x:x + 2], data[x + 2:x + 4], data[x + 4:x + 6]]
                       for x in range(8, len(data) - 8, 6)]  # split remaining data into list of 3-tupels
        if data_points == []:  # catch case where buffer is empty
            return [], [], [], remaining_packages, lost_packages
        # convert data to int16, convert into numpy array to allow slicing;
        # numpy messes up bytes, so only do this after conversion
        data_points_unpacked = np.array(
            list(map(self._unpack_int, data_points)))
        flow = data_points_unpacked[:, 0] / self.flow_scale_factor
        temp = data_points_unpacked[:, 1] / self.temp_scale_factor
        flags = data_points_unpacked[:, 2]
        return flow, temp, flags, remaining_packages, lost_packages


class SetTotalizatorStatus(ShdlcCommand):
    ''' Enable or disable the Totalizator. The value of the Totalizator is not
        changed with this command
        status == True: enable
        status == False: disable'''

    def __init__(self, status):
        if not isinstance(status, bool):
            raise ValueError(
                'status argument of SetTotalizatorStatus must be boolean')
        self.status = int.to_bytes(status, length=1, byteorder='big')
        super(SetTotalizatorStatus, self).__init__(
            id=0x37,  # Command ID as specified in the device documentation
            data=self.status,
            max_response_time=0.001,  # Maximum response time in Seconds
        )

    def interpret_response(self, data):
        return 0


class GetTotalizatorStatus(ShdlcCommand):
    ''' Get the Status (enabled / disabled) of the Totalizator. '''

    def __init__(self):
        super(GetTotalizatorStatus, self).__init__(
            id=0x37,  # Command ID as specified in the device documentation
            data=b'',
            max_response_time=0.001,  # Maximum response time in Seconds
        )

    def interpret_response(self, data):
        if int.from_bytes(data, byteorder='big', signed=True) == b'\x00':
            return False
        else:
            return True


class ResetTotalizator(ShdlcCommand):
    ''' Set the Totalizator value to zero, the Totalizator Status (enabled/disabled) is
        not changed. The Totalizator can be reset anytime.'''

    def __init__(self):
        super(ResetTotalizator, self).__init__(
            id=0x39,
            data=b'',
            max_response_time=0.001,  # Maximum response time in Seconds
        )

    def interpret_response(self, data):
        return 0


class GetTotalizatorValue(ShdlcCommand):
    ''' Get the value of the Totalizator. This value is the sum of all unscaled
        measurements while in continuous measurement. Only the flow values (signal 1)
        are totalized '''

    def __init__(self, start_time, flow_scale_factor):
        # see data sheet, (ul/min)**-1
        self.flow_scale_factor = flow_scale_factor
        self.start_time = start_time
        self.end_time = time.time()
        super(GetTotalizatorValue, self).__init__(
            id=0x38,  # Command ID as specified in the device documentation
            data=b'',
            max_response_time=0.001,  # Maximum response time in Seconds
        )

    def interpret_response(self, data):
        total_unscaled = int.from_bytes(data, byteorder='big', signed=True)
        # get total measurement time in minutes
        total_measurement_time = (self.end_time - self.start_time) / 60
        total_volume = total_unscaled * total_measurement_time / \
            self.flow_scale_factor  # in units of ul
        return total_volume


class SetSensorType(ShdlcCommand):
    ''' Set the Sensor Type and save the setting in the sensor cable EEPROM.'''

    def __init__(self, sensor_type):
        sens_type_payload = int.to_bytes(
            sensor_type, length=1, byteorder='big')
        super(SetSensorType, self).__init__(
            id=0x24,
            data=sens_type_payload,
            max_response_time=0.025,  # Maximum response time in Seconds
        )

    def interpret_response(self, data):
        return 0


class GetScaleFactor(ShdlcCommand):
    ''' reads the sensor scale factor in units of (ul/min)**-1'''

    def __init__(self):
        super(GetScaleFactor, self).__init__(
            id=0x53,
            data=b"\x36\x08",  # 0x3608 = use water calibration
            # data=b"",  # The payload data to send
            max_response_time=0.2,  # Maximum response time in Seconds
        )

    def interpret_response(self, data):
        # Convert the received raw bytes to the proper data types
        scale_factor = unpack('>h', data[0:2])[0]
        if data[2:5] == b'\x08E\x00':
            # units of (ml/min)**-1, convert this to (ul/min)**-1
            scale_factor = scale_factor * 1e-3
        elif data[2:5] == b'\x08D\x00':
            # units are already on (ul/min)**-1
            pass
        else:
            raise ValueError(
                'received unknown binary code for scale factor units')
        return scale_factor


class slf3s(ShdlcDeviceBase):
    def __init__(self, connection, slave_address):
        super(slf3s, self).__init__(connection, slave_address)
        self.flow_scale_factor = self.get_scale_factor()

    def __enter__(self):
        pass

    def __exit__(self):
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

