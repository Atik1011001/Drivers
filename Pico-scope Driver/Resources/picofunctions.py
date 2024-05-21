import ctypes
from ctypes import *
from enum import IntEnum
from time import sleep


class Channels(IntEnum):
    CHANNEL_1 = 1
    CHANNEL_2 = 2
    CHANNEL_3 = 3
    CHANNEL_4 = 4
    CHANNEL_5 = 5
    CHANNEL_6 = 6
    CHANNEL_7 = 7
    CHANNEL_8 = 8
    MAX_CHANNELS = CHANNEL_8

class DataTypes(IntEnum):
    OFF = 0
    PT100 = 1
    PT1000 = 2
    RESISTANCE_TO_375R = 3
    RESISTANCE_TO_10K = 4
    DIFFERENTIAL_TO_115MV = 5
    DIFFERENTIAL_TO_2500MV = 6
    SINGLE_ENDED_TO_115MV = 7
    SINGLE_ENDED_TO_2500MV = 8



class picodriver:


    def __init__(self,serial=b''):

        '''
        Loads the dll library and open device with serial number
        '''
        self.serial = serial
        self.status = {}
        self.handle = c_short()

        if type(self.serial) is str:
            self.serial = self.serial.encode()
        try:
            mydll = ctypes.cdll.LoadLibrary("./Resources/Usbpt104.dll")
        except WindowsError:
            raise ImportError('usbpt104.dll not found. Check driver is installed.')
        self.mydll = mydll
        
    
    def connect(self):

        '''
        Opens Devices from serial number
        '''
        self.handle = c_short()
        self.status["openunit"] = self.mydll.UsbPt104OpenUnit(byref(self.handle), self.serial)

        if self.status["openunit"] != 0:
            raise Exception("Device not connected")
        else:
            print("Device Connected")

    
    def close(self):
        
        '''
        Closes Devices from serial number
        '''
        self.status["closeunit"] = self.mydll.UsbPt104CloseUnit(self.handle)

        if self.status["closeunit"] != 0:
            print("Device unable to close")
        else:
            print("Device closed")

    def set_frequncey(self, frequency):
        '''
        set device frequency
        '''
        if frequency == 60:
            sixty_hertz = c_ushort(1)
        if frequency == 50:
            sixty_hertz = c_ushort(0)
        elif frequency != 50 or 60:
            print("Frequency can only be 50Hz or 60Hz")

        self.status["frequency"] = self.mydll.UsbPt104SetMains(self.handle, sixty_hertz)
    
        if self.status["frequency"] != 0:
            print("Device unable to set frequency")
        else:
            print("Frequency:" + str(frequency))

    def set_channel(self, Channel, DataType):
        '''
        set device Channel
        '''
        nb_wires = 4
        self.status["channel"] = self.mydll.UsbPt104SetChannel(self.handle,Channel,DataType,nb_wires)

    def get_value(self, channel):
        '''
        get data from device
        '''
        valueee = c_long()
        self.status["value"] = self.mydll.UsbPt104GetValue(self.handle,channel,byref(valueee),False)
        #print(self.status["value"]/ 10.0 ** 3)
        return float(valueee.value) * 1/1000