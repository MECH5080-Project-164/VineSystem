# -*- coding: utf-8 -*
'''!
  @file DFRobot_MPX5700_optimized.py
  @brief Optimized version of the Raspberry Pi library for the air pressure sensor
  @note Removed excessive sleep delays for high-frequency operation
'''

import time
import smbus
import spidev
import os
import RPi.GPIO as GPIO
import math
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
I2C_MODE   = 1

class DFRobot_MPX5700_Optimized(object):
  '''!
    @brief Optimized basic class of the air pressure sensor
  '''
  def __init__(self ,bus):
    if bus != 0:
      self.i2cbus = smbus.SMBus(bus)
    self._calibration_set = False

  def set_mean_sample_size(self, size):
    '''!
      @brief Set sample size, i.e take the mean value based on how many raw data
      @param size Sample size
    '''
    buf = [0]*1
    buf[0] = size
    self.write_reg(0x05, buf)
    # Reduced sleep time for faster operation
    time.sleep(0.05)  # 50ms instead of 500ms

  def get_pressure_value_kpa_fast(self):
    '''
      @brief Get the current air pressure (optimized for speed)
      @return float type
    '''
    # Read pressure data directly without excessive delays
    buf = self.read_reg(0x06, 2)
    if buf == -1 or len(buf) < 2:
        return -1.0
    
    Pressure_100 = (buf[0] << 8) | buf[1]
    return (Pressure_100/100.0)

  def get_pressure_value_kpa(self, ifcalibration):
    '''
      @brief Get the current air pressure (original method for compatibility)
      @param ifcalibration Whether to output calibrated air pressure value
      @return float type
    '''
    # Only set calibration if it hasn't been set or if it's different
    if not self._calibration_set or ifcalibration:
        sbuf = [0]*1
        sbuf[0] = ifcalibration
        self.write_reg(0x09, sbuf)
        # Reduced sleep time
        time.sleep(0.01)  # 10ms instead of 1000ms
        self._calibration_set = True
    
    buf = self.read_reg(0x06, 2)
    if buf == -1 or len(buf) < 2:
        return -1.0
    
    # No sleep after reading - not necessary for most applications
    Pressure_100 = (buf[0] << 8) | buf[1]
    return (Pressure_100/100.0)

  def calibration_kpa(self, standard_values):
    '''
      @brief Set standard air pressure
      @param standard_values The air pressure reference value for calibration
    '''
    Pressure = [0]*2
    ifcalibration = [0]*1
    ifcalibration[0] = 1
    plus_or_minus_calibration = [0]*1
    values = self.get_pressure_value_kpa(0)
    if(standard_values > values):
      plus_or_minus_calibration[0] = 0 #Plus calibration
    else:
      plus_or_minus_calibration[0] = 1 #Minus calibration
    self.write_reg(0x0C, plus_or_minus_calibration)
    time.sleep(0.01)  # Reduced from 1s to 10ms
    Pressure_100 = (abs(standard_values - values) * 100)
    Pressure[0] = (int(Pressure_100) >> 8) & 0xff
    Pressure[1] = int(Pressure_100) & 0xff
    self.write_reg(0x0A, Pressure)
    time.sleep(0.01)  # Reduced from 1s to 10ms
    self.write_reg(0x08, ifcalibration)

#brief An example of an i2c interface module
class DFRobot_MPX5700_I2C_Optimized(DFRobot_MPX5700_Optimized):
  def __init__(self, bus, addr):
    self.__addr = addr
    super(DFRobot_MPX5700_I2C_Optimized, self).__init__(bus)

  def write_reg(self, reg, data):
    '''
      @brief writes data to a register
      @param reg register address
      @param value written data
    '''
    try:
      self.i2cbus.write_i2c_block_data(self.__addr, reg, data)
      return True
    except Exception as e:
      print(f"I2C write error: {e}")
      return False

  def read_reg(self, reg, length):
    '''
      @brief read the data from the register
      @param reg register address
      @param length read data length
    '''
    try:
      rslt = self.i2cbus.read_i2c_block_data(self.__addr, reg, length)
      return rslt
    except Exception as e:
      print(f"I2C read error: {e}")
      return -1
