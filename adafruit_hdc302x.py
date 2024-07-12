# SPDX-FileCopyrightText: Copyright (c) 2024 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_hdc302x`
================================================================================

CircuitPython driver for the Adafruit HDC302x Precision Temperature/Humidity breakout


* Author(s): Liz Clark

Implementation Notes
--------------------

**Hardware:**

* `Adafruit HDC3021 Sensor - STEMMA QT / Qwiic <https://www.adafruit.com/product/5989>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

import struct
import busio
from adafruit_bus_device import i2c_device

try:
    from typing import Tuple, List, Dict
except ImportError:
    pass

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_HDC302x.git"


class HDC302x:
    """Driver for the HDC302x temperature and humidity sensor."""

    # Auto modes
    AUTO_MODES: Dict[str, int] = {
        "5MPS_LP0": 0x2032,
        "5MPS_LP1": 0x2024,
        "5MPS_LP2": 0x202F,
        "5MPS_LP3": 0x20FF,
        "1MPS_LP0": 0x2130,
        "1MPS_LP1": 0x2126,
        "1MPS_LP2": 0x212D,
        "1MPS_LP3": 0x21FF,
        "2MPS_LP0": 0x2236,
        "2MPS_LP1": 0x2220,
        "2MPS_LP2": 0x222B,
        "2MPS_LP3": 0x22FF,
        "4MPS_LP0": 0x2334,
        "4MPS_LP1": 0x2322,
        "4MPS_LP2": 0x2329,
        "4MPS_LP3": 0x23FF,
        "10MPS_LP0": 0x2737,
        "10MPS_LP1": 0x2721,
        "10MPS_LP2": 0x272A,
        "10MPS_LP3": 0x27FF,
        "EXIT_AUTO_MODE": 0x3093,
    }
    # Heater power options
    HEATER_POWERS: Dict[str, int] = {
        "OFF": 0x0000,
        "QUARTER_POWER": 0x009F,
        "HALF_POWER": 0x03FF,
        "FULL_POWER": 0x3FFF,
    }

    def __init__(self, i2c_bus: busio.I2C, address: int = 0x44) -> None:
        """
        Initialize the HDC302x sensor with the given I2C bus and address.

        :param i2c_bus: The I2C bus object.
        :param address: The I2C address of the sensor.
        """
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)
        self._current_auto_mode = self.AUTO_MODES["EXIT_AUTO_MODE"]

    @property
    def heater(self) -> bool:
        """
        Heater power

        :return: True if the heater is on, False otherwise.
        """
        status = self._read_command(0xF32D)
        return bool(status & (1 << 13))

    @heater.setter
    def heater(self, power: str) -> None:
        """
        :param power: The heater power level.
        """
        if power not in self.HEATER_POWERS:
            raise ValueError("Invalid heater power value.")
        _power = self.HEATER_POWERS[power]
        print(_power)
        if _power == self.HEATER_POWERS["OFF"]:
            self._write_command(0x3066)
        else:
            self._write_command(0x306D)
            self._write_command_data(0x306E, _power)

    @property
    def status(self) -> int:
        """
        Status of the sensor.

        :return: The status of the sensor.
        """
        return self._read_command(0xF32D)

    @property
    def manufacturer_id(self) -> int:
        """
        Manufacturer ID of the sensor.

        :return: The manufacturer ID.
        """
        return self._read_command(0x3781)

    @property
    def nist_id(self) -> List[int]:
        """
        NIST ID of the sensor.

        :return: NIST ID as a list of integers.
        """
        id_part1 = self._read_command(0x3683)
        id_part2 = self._read_command(0x3684)
        id_part3 = self._read_command(0x3685)
        return [
            id_part1 >> 8,
            id_part1 & 0xFF,
            id_part2 >> 8,
            id_part2 & 0xFF,
            id_part3 >> 8,
            id_part3 & 0xFF,
        ]

    @property
    def auto_mode(self) -> int:
        """
        Auto mode for measurements.

        :return: The current auto mode.
        """
        return self._current_auto_mode

    @auto_mode.setter
    def auto_mode(self, mode: int) -> None:
        """
        :param mode: The auto mode to set.
        """
        if mode not in self.AUTO_MODES:
            raise ValueError("Invalid auto mode value.")
        _mode = self.AUTO_MODES[mode]
        self._current_auto_mode = _mode
        self._write_command(_mode)

    @property
    def offsets(self) -> Tuple[float, float]:
        """
        Set temperature and relative humidity offsets

        :return: The temperature and humidity offsets.
        """
        combined_offsets = self._read_command(0xA004)
        rh_offset = (combined_offsets >> 8) & 0xFF
        temp_offset = combined_offsets & 0xFF
        return self._invert_offset(temp_offset, True), self._invert_offset(
            rh_offset, False
        )

    @offsets.setter
    def offsets(self, temp: float, humid: float) -> None:
        """
        :param values: A tuple containing the temperature and humidity offsets.
        """
        rh_offset = self._calculate_offset(humid, False)
        temp_offset = self._calculate_offset(temp, True)
        combined_offsets = (rh_offset << 8) | temp_offset
        self._write_command_data(0xA004, combined_offsets)

    @property
    def auto_temperature(self) -> float:
        """
        Read temperature in auto mode.

        :return: The temperature in degrees Celsius.
        """
        temp, _ = self._send_command_read_trh(0xE000)
        return temp

    @property
    def auto_relative_humidity(self) -> float:
        """
        Read relative humidity in auto mode.

        :return: The relative humidity in percent.
        """
        _, humid = self._send_command_read_trh(0xE000)
        return humid

    @property
    def temperature(self) -> float:
        """
        The measured temperature in degrees Celsius.

        :return: The temperature in degrees Celsius.
        """
        temp, _ = self._send_command_read_trh(0x2400)
        return temp

    @property
    def relative_humidity(self) -> float:
        """
        The measured relative humidity in percent.

        :return: The relative humidity in percent.
        """
        _, humid = self._send_command_read_trh(0x2400)
        return humid

    @property
    def high_alert(self) -> bool:
        """
        Check if the high alert is activated.

        :return: True if the high alert is activated, False otherwise.
        """
        status = self._read_command(0xF32D)
        return bool(status & ((1 << 10) | (1 << 9)))

    @property
    def low_alert(self) -> bool:
        """
        Check if the low alert is activated.

        :return: True if the low alert is activated, False otherwise.
        """
        status = self._read_command(0xF32D)
        return bool(status & ((1 << 12) | (1 << 11)))

    def set_high_alert(self, temp: float, humid: float) -> bool:
        """
        Set the high alert thresholds for temperature and humidity.

        :param temp: The temperature threshold in degrees Celsius.
        :param humid: The relative humidity threshold in percent.
        """
        self._alert_command(0x611D, temp, humid)

    def set_low_alert(self, temp: float, humid: float) -> bool:
        """
        Set the low alert thresholds for temperature and humidity.

        :param temp: The temperature threshold in degrees Celsius.
        :param humid: The relative humidity threshold in percent.
        """
        self._alert_command(0x6100, temp, humid)

    def clear_high_alert(self, temp: float, humid: float) -> bool:
        """
        Clear the high alert thresholds for temperature and humidity.

        :param temp: The temperature threshold in degrees Celsius.
        :param humid: The relative humidity threshold in percent.
        """
        self._alert_command(0x6116, temp, humid)

    def clear_low_alert(self, temp: float, humid: float) -> bool:
        """
        Clear the low alert thresholds for temperature and humidity.

        :param temp: The temperature threshold in degrees Celsius.
        :param humid: The relative humidity threshold in percent.
        """
        self._alert_command(0x610B, temp, humid)

    def _write_command(self, command: int) -> bool:
        with self.i2c_device as i2c:
            i2c.write(bytes([command >> 8, command & 0xFF]))

    def _write_command_data(self, command: int, data: int) -> bool:
        crc = self._calculate_crc8(struct.pack(">H", data))
        with self.i2c_device as i2c:
            i2c.write(
                bytes([command >> 8, command & 0xFF, data >> 8, data & 0xFF, crc])
            )

    def _read_command(self, command: int) -> int:
        with self.i2c_device as i2c:
            i2c.write_then_readinto(
                bytes([command >> 8, command & 0xFF]), result := bytearray(3)
            )
        crc = self._calculate_crc8(result[:2])
        if crc != result[2]:
            raise RuntimeError("CRC check failed")
        return (result[0] << 8) | result[1]

    def _send_command_read_trh(self, command: int) -> Tuple[float, float]:
        self._write_command(command)
        with self.i2c_device as i2c:
            i2c.readinto(result := bytearray(6))
        if (
            self._calculate_crc8(result[:2]) != result[2]
            or self._calculate_crc8(result[3:5]) != result[5]
        ):
            raise RuntimeError("CRC check failed")
        temp_raw = (result[0] << 8) | result[1]
        hum_raw = (result[3] << 8) | result[4]
        temperature = ((temp_raw / 65535.0) * 175.0) - 45.0
        relative_humidity = (hum_raw / 65535.0) * 100.0
        return temperature, relative_humidity

    def _alert_command(self, command: int, temp: float, humid: float) -> bool:
        raw_temp = int(((temp + 45.0) / 175.0) * 65535.0)
        raw_rh = int((humid / 100.0) * 65535.0)
        msb_rh = (raw_rh >> 9) & 0x7F
        msb_temp = (raw_temp >> 7) & 0x1FF
        threshold = (msb_rh << 9) | msb_temp
        self._write_command_data(command, threshold)

    @staticmethod
    def _calculate_crc8(data: bytes) -> int:
        """Calculate the CRC-8 checksum."""
        crc = 0xFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                crc = (crc << 1) ^ 0x31 if crc & 0x80 else crc << 1
        return crc & 0xFF

    @staticmethod
    def _calculate_offset(value: float, is_temp: bool) -> int:
        """Calculate the closest matching offset byte for temperature or relative humidity."""
        lsb = 0.1708984375 if is_temp else 0.1953125
        sign = 0x00 if value < 0 else 0x80
        abs_value = abs(value)
        offset = int(round(abs_value / lsb))
        return sign | offset

    @staticmethod
    def _invert_offset(offset: int, is_temp: bool) -> float:
        """Invert the offset byte to a floating point value."""
        lsb = 0.1708984375 if is_temp else 0.1953125
        is_negative = not offset & 0x80
        abs_offset = offset & 0x7F
        value = abs_offset * lsb
        return -value if is_negative else value
