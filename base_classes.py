#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Part of sensirion_slf3s package on https://github.com/tnaegele/sensirion_slf3s

@author: Tobias E. Naegele, 02/2024


"""

from sensirion_shdlc_driver import ShdlcDeviceBase
from sensirion_shdlc_driver.command import ShdlcCommand
from struct import unpack
import numpy as np
import time

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
