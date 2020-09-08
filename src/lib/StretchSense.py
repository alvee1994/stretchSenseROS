#!/usr/bin/env python3

"""

    - Website : https://www.stretchsense.com

    - Important : This StretchSense Library has been designed to enable the connection of one or more "StretchSense Sensor" and "StretchSense IC Boards" to your Raspberry Pi

    - Author : Louis Germain

    - Copyright : 2017 StretchSense

    - Date : 26/07/2017

    - Version : 1.0.0

"""

from __future__ import print_function
from easydict import EasyDict
import argparse
import binascii
import time
import os
import sys
from threading import Timer, Lock
from bluepy import btle
import numpy as np

#ROS
#import rospy
#from std_msgs.msg import String
#ROS


class RepeatedTimer(object):
    #print("RepeatedTimer()")

    """
    Class used to create a timer which repeat every defined interval.

    :param object: int, function
        int : timeout

        function : is the function that you want to repeat (should be written as lambda: "function")

    :returns:

        timer
            Returns a timer which start when the function is called, you should do "timer.stop" to terminate it

    """

    def __init__(self, interval, function, *args, **kwargs):
        self._lock = Lock()
        self._timer = None
        self.function = function
        self.interval = interval
        self.args = args
        self.kwargs = kwargs
        self._stopped = True
        if kwargs.pop('autostart', True):
            self.start()

    def _run(self):
        self.start(from_run=True)
        self.function(*self.args, **self.kwargs)

    def start(self, from_run=False):
        self._lock.acquire()
        if from_run or self._stopped:
            self._stopped = False
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self._lock.release()

    def stop(self):
        self._lock.acquire()
        self._stopped = True
        self._timer.cancel()
        self._lock.release()


"""

Board Configuration Raspberry Pi 3 B+ SPI0:

NSS = 24
SCK = 23
MISO = 21
MOSI = 19

Board Configuration Raspberry Pi 3 B+ SPI1:

NSS = 26
SCK = 40
MISO = 35
MOSI = 38

"""
# GPIO Layout configuration

##GPIOLAYOUT = GPIO.BOARD
#GPIOLAYOUT = GPIO.BCM

# Set up the SPI pattern from the GPIO LAYOUT

##if (GPIOLAYOUT == GPIO.BOARD):
##    CE_PIN0 = 24
##    CE_PIN1 = 26
##    INTERRUPT_PIN = 13
##    TRIGGER_PIN = 15
##elif (GPIOLAYOUT == GPIO.BCM):
##    CE_PIN0 = 10
##    CE_PIN1 = 11
##    INTERRUPT_PIN = 2
##    TRIGGER_PIN = 3

# GPIO & SPI Pin Configuration

##GPIO.setmode(GPIOLAYOUT)
##GPIO.setwarnings(False)
##SPI0 = 0
##SPI1 = 1                    # SPI1 still in developpment

### Data Package options
##
##DATA = 0x00
##CONFIG = 0x01
##
### Output Data Rate
##
##RATE_OFF = 0x00
##RATE_25HZ = 0x01
##RATE_50HZ = 0x02
##RATE_100HZ = 0x03
##RATE_166HZ = 0x04
##RATE_200HZ = 0x05
##RATE_250HZ = 0x06
##RATE_500HZ = 0x07
##RATE_1KHZ = 0x08
##
### Interrupt Mode
##
##INTERRUPT_DISABLED = 0x00
##INTERRUPT_ENABLED = 0x01
##
### Trigger Mode
##
##TRIGGER_DISABLED = 0x00
##TRIGGER_ENABLED = 0x01
##
### Filter Mode
##
##FILTER_0PT = 0x00
##FILTER_1PT = 0x01
##FILTER_3PT = 0x03
##FILTER_7PT = 0x07
##FILTER_15PT = 0x0F
##FILTER_31PT = 0x1F
##FILTER_63PT = 0x3F
##FILTER_127PT = 0x7F
##FILTER_255PT = 0xFF
##
### Resolution Mode
##
##RESOLUTION_1pF = 0x00
##RESOLUTION_100fF = 0x01
##RESOLUTION_10fF = 0x02
##RESOLUTION_1fF = 0x03
##
### Config Transfer
##
##PADDING = 0x00
##
### Configuration Setup
##
##ODR_MODE = RATE_50HZ
##INTERRUPT_MODE = INTERRUPT_DISABLED
##TRIGGER_MODE = TRIGGER_DISABLED
##FILTER_MODE = FILTER_1PT
##RESOLUTION_MODE = RESOLUTION_100fF
##
##
##"""
##StretchSense Classes & generators for the different type of sensors.
##
##"""


class StretchSensePeripheral:
    #print("\033[0;35;40m StretchSensePeripheral()\033[0m")

    """
    Class which create a StretchSense peripheral with all the argument that can be used.

    :param addr: string:
        Contains the address of the device : "xx:xx:xx:xx:xx".

    :param uuid: string:
        Contains the UUID of the device "xxxxxxxx-7374-7265-7563-6873656e7365".

    :param value: int:
        Contains the value of the device in hexadecimal.

    :param channelNumber: int:
        For devices with multiple channels, contains the number of the channel.

    :param gen: string:
        This number is the generation of the device.

    :param color: string:
        This gives the device a color to use when displayed.

    """

    def __init__(self):
        #print("\033[0;35;40m __init__().StretchSensePeripheral()\033[0m")

        # Mac Address
        self.addr = ''

        # Unique Number Identifier
        self.uuid = ''

        # Current Capacitance value of the sensor
        self.value = 0x0000

        # A unique number for each sensor, based on the order when the sensor is connected to the device
        self.channelNumber = 0

        # Generation of the circuit, we initialize it at a Generation 2 which the common One-Channel sensor
        self.gen = ''

        # Background color of the circuit
        self.color = ''


class StretchSenseAPI():
    #print("\033[0;35;40m StretchSenseAPI()\033[0m")
    # This is the list of peripherals we are using to connect to the BLE

    listPeripheralInUse = [btle.Peripheral()]

    # This is the list of StretchSense Bluetooth peripherals detected by the Raspberry Pi during a scan event

    listPeripheralAvailable = [StretchSensePeripheral()]

    # This is the list of StretchSense Bluetooth peripherals which are connected to the Raspberry Pi after being scanned

    listPeripheralIsConnected = [StretchSensePeripheral()]

    # This is the list of StretchSense Bluetooth peripherals which are saved to the Raspberry Pi after being connected once

    listPeripheralIsOnceConnected = [StretchSensePeripheral()]

    """

    Variables : Services & Characteristics UUID

    """

    # The name we use to filter BLE scan results and find only StretchSense's sensors
    deviceName = 'StretchSense'

    # The UUID used to filter Bluetooth scan results and find the services from StretchSense sensors Gen 3
    serviceUUID3 = '00001701-7374-7265-7563-6873656e7365'
    # The UUID used to filter the device characteristics and find the "data characteristic" from StretchSense sensors Gen 3
    dataUUID3 = '00001702-7374-7265-7563-6873656e7365'
    # The UUID to set the Sampling Rate for Gen 3 with characteristics starting with UUID 000017
    freqUUID317 = '00001705-7374-7265-7563-6873656e7365'
    # The UUID to set the Sampling Rate for Gen 3 with characteristics starting with UUID 000019
    freqUUID319 = '00001922-7374-7265-7563-6873656e7365'

    # The UUID used to filter Bluetooth scan results and find the services from StretchSense sensors Gen 2
    serviceUUID2 = '00001501-7374-7265-7563-6873656e7365'
    # The UUID used to filter the device characteristics and find the "data characteristic" from StretchSense sensors Gen 2
    dataUUID2 = '00001502-7374-7265-7563-6873656e7365'

    # The UUID used to filter Bluetooth scan results and find the services from StretchSense circuit 10TT
    serviceUUID10TT = '00601001-7374-7265-7563-6873656e7365'
    # The UUID used to filter the device characteristics and find the "data characteristic" from StretchSense circuit 10TT
    dataUUID10TT = '00601002-7374-7265-7563-6873656e7365'

    """

    Variables : Set sensors & info

    """
    # Number of data samples within the filtering array

    numberOfSample = 30

    # Initialisation value of the sampling time value (SamplingTime = (value +1)*40ms)

    samplingTimeNumber = 0

    # Sized of the filter based on the number of samples

    filteringNumber = 0

    """

    Bluepy buffer Scanning class.

    """

    class ScanPrint(btle.DefaultDelegate):
        #print("\033[0;33;40m ScanPrint()\033[0m")

        def __init__(self, opts):
            #print("\033[0;33;40m __init__().ScanPrint()\033[0m")
            btle.DefaultDelegate.__init__(self)
            self.opts = opts

        def handleDiscovery(self, dev, isNewDev, isNewData):
            #print("\033[0;33;40m handleDiscovery()\033[0m")

            if isNewDev:
                status = "new"
            elif isNewData:
                if self.opts.new:
                    return
                status = "update"
            else:
                if not self.opts.all:
                    return
                status = "old"

            if dev.rssi < self.opts.sensitivity:
                return

            if not dev.scanData:
                print ('\t(no data)')
            print()

    """

    Functions : Scan, Print, Connect/Disconnect & Update

    """

    def ble_printAllPeripheralsAvailable(self):
        #print("\033[0;35;40m ble_printAllPeripheralsAvailable()\033[0m")

        """

        Display all the StretchSense's device addresses available around.


        """

        #return self.listPeripheralAvailable
    
        numberOfPeripheralAvailable = len(self.listPeripheralAvailable) - 1

        if (numberOfPeripheralAvailable == 0):
            return print("No devices available")

        elif numberOfPeripheralAvailable >= 1:

            print("Number of StretchSense devices detected : ", numberOfPeripheralAvailable)

            for myPeripheralAvailable in self.listPeripheralAvailable:

                if myPeripheralAvailable.addr != '':
                    print("StretchSense device : %s\n" % (myPeripheralAvailable.addr))

    def ble_printAllPeripheralsConnected(self):
        #print("\033[0;35;40m ble_printAllPeripheralsConnected()\033[0m")

        """
        Display all the StretchSense's device addresses connected.

        """

        numberOfPeripheralConnected = len(self.listPeripheralIsConnected) - 1

        if (numberOfPeripheralConnected == 0):
            return print("No sensors connected")

        elif numberOfPeripheralConnected >= 1:

            for myPeripheralConnected in self.listPeripheralIsConnected:

                if myPeripheralConnected.addr != '':
                    print("StretchSense Sensor Connected : %s\n" % (myPeripheralConnected.addr))

    def ble_scanning(self, scanTime):
        #print("\033[0;35;40m ble_scanning()\033[0m")

        """

        Scan for StretchSense devices in the area and store them in listPeripheralAvailable.

        :param scanTime: int :
            Time to scan for.

        July 29, 2020:
            argparse commented by Alvee to allow use with Jupyter Notebook
            added EasyDict instead to work with Jupyter Notebook
        """



        # parser = argparse.ArgumentParser()
        # parser.add_argument('-i', '--hci', action='store', type=int, default=0,
        #                     help='Interface number for scan')
        # parser.add_argument('-t', '--timeout', action='store', type=int, default=4,
        #                     help='Scan delay, 0 for continuous')
        # parser.add_argument('-s', '--sensitivity', action='store', type=int, default=-128,
        #                     help='dBm value for filtering far devices')
        # parser.add_argument('-d', '--discover', action='store_true',
        #                     help='Connect and discover service to scanned devices')
        # parser.add_argument('-a', '--all', action='store_true',
        #                     help='Display duplicate adv responses, by default show new + updated')
        # parser.add_argument('-n', '--new', action='store_true',
        #                     help='Display only new adv responses, by default show new + updated')
        # parser.add_argument('-v', '--verbose', action='store_true',
        #                     help='Increase output verbosity')
        # arg = parser.parse_args(sys.argv[1:])


        arg = EasyDict({
            'hci': 0,
            'timeout': 4,
            'sensitivity': -128,
            'discover': True,
            'all': True,
            'new': True,
            'verbose': True
        })

        # self.Debugging = arg.verbose
        scanner = btle.Scanner(arg.hci).withDelegate(self.ScanPrint(arg))
        devicesAvailable = scanner.scan(scanTime)
        self.listPeripheralInUse = []

        for devices in devicesAvailable:
            deviceAlreadyInTheList = False

            for (sdid, desc, val) in devices.getScanData():

                if (val == StretchSenseAPI.deviceName):

                    for myDeviceInTheList in self.listPeripheralAvailable:

                        if (myDeviceInTheList.addr == devices.addr):
                            deviceAlreadyInTheList = True

                    if deviceAlreadyInTheList is False:
                        self.listPeripheralAvailable.append(devices)

    def ble_connectOnePeripheral(self, myDeviceAddr):
        #print("\033[0;35;40m ble_connectOnePeripheral()\033[0m")

        """

        Connect one StretchSense device which is available using its address, generate one or more
        peripherals as StretchSensePeripheral and store them into listPeripheralIsConnected.
        The peripheral is also stored in listPeripheralInUse.

        :param myDeviceAddr: string :
            Address of the device that you want to connect.

        """

        numberOfPeripheralAvailable = len(self.listPeripheralAvailable) - 1

        if (len(self.listPeripheralIsConnected) - 1) > 0:
            pass
        else:
            self.listPeripheralIsConnected = []

        if numberOfPeripheralAvailable > 0:
            for myPeripheralAvailable in self.listPeripheralAvailable:
                if (myPeripheralAvailable.addr == myDeviceAddr):

                    myPeripheralConnected = btle.Peripheral(myPeripheralAvailable)
                    myPeripheralConnected.setDelegate(StretchSenseDelegate(myPeripheralConnected))
                    myPeripheralConnected.deviceAddr = myDeviceAddr
                    self.listPeripheralInUse.append(myPeripheralConnected)
                    listOfServices = sorted(myPeripheralConnected.services, key=lambda services: services.hndStart)

                    for services in listOfServices:

                        if services.hndStart == services.hndEnd:
                            continue

                        if services.uuid == self.serviceUUID2:
                            myPeripheralConnected.gen = "2"
                            myPeripheralConnected.uuid = self.serviceUUID2
                            self.ble_generateOneChannel(myPeripheralConnected)
                            characteristics = services.getCharacteristics()[0]
                            myPeripheralConnected.writeCharacteristic(characteristics.valHandle + 1, b"\x01\x00")
                            continue

                        if services.uuid == self.serviceUUID3:
                            myPeripheralConnected.gen = "3"
                            myPeripheralConnected.uuid = self.serviceUUID3
                            self.ble_generateTenChannel(myPeripheralConnected)
                            characteristics = services.getCharacteristics()[0]
                            myPeripheralConnected.writeCharacteristic(characteristics.valHandle + 1, b"\x01\x00")
                            # Added to change sampling rate to 100Hz for gen 3 devices
                            uuids = [char.uuid for char in services.getCharacteristics()]
                            if self.freqUUID317 in uuids:
                                idx = uuids.index(self.freqUUID317)
                                char = services.getCharacteristics()[idx]
                                myPeripheralConnected.writeCharacteristic(char.valHandle, b"\x64")

                            if self.freqUUID319 in uuids:
                                idx = uuids.index(self.freqUUID319)
                                char = services.getCharacteristics()[idx]
                                myPeripheralConnected.writeCharacteristic(char.valHandle, b"\x64")

                            # End
                            continue

                        if services.uuid == self.serviceUUID10TT:
                            myPeripheralConnected.gen = "10TT"
                            myPeripheralConnected.uuid = self.serviceUUID10TT
                            self.ble_generateTenChannel(myPeripheralConnected)
                            characteristics = services.getCharacteristics()
                            for char in characteristics:
                                if char.uuid == self.dataUUID10TT:
                                    myPeripheralConnected.writeCharacteristic(char.valHandle + 1, b"\x01\x00")
                            continue

    def ble_connectAllPeripheral(self):
        #print("\033[0;35;40m ble_connectAllPeripheral()\033[0m")

        """

        Connect all StretchSense devices which are available using their addresses, generate the
        peripherals as StretchSensePeripheral and store them into listPeripheralIsConnected.
        Peripherals are also stored in listPeripheralInUse.

        """

        numberOfPeripheralAvailable = len(self.listPeripheralAvailable) - 1
        self.listPeripheralIsConnected = []
        self.listPeripheralInUse = []

        if numberOfPeripheralAvailable > 0:

            for myPeripheralAvailable in self.listPeripheralAvailable:

                if myPeripheralAvailable.addr != '':
                    #print('Address we are trying to connect to : ', myPeripheralAvailable.addr)
                    myPeripheralConnected = btle.Peripheral(myPeripheralAvailable)
                    myPeripheralConnected.setDelegate(StretchSenseDelegate(myPeripheralConnected))
                    myPeripheralConnected.deviceAddr = myPeripheralAvailable.addr
                    self.listPeripheralInUse.append(myPeripheralConnected)
                    listOfServices = sorted(myPeripheralConnected.services, key=lambda services: services.hndStart)

                    for services in listOfServices:
                        if services.hndStart == services.hndEnd:
                            continue

                        if services.uuid == self.serviceUUID2:
                            myPeripheralConnected.gen = '2'
                            myPeripheralConnected.uuid = self.serviceUUID2
                            self.ble_generateOneChannel(myPeripheralConnected)
                            characteristics = services.getCharacteristics()[0]
                            myPeripheralConnected.writeCharacteristic(characteristics.valHandle + 1, b"\x01\x00")
                            continue

                        if services.uuid == self.serviceUUID3:
                            myPeripheralConnected.gen = '3'
                            myPeripheralConnected.uuid = self.serviceUUID3
                            self.ble_generateTenChannel(myPeripheralConnected)
                            characteristics = services.getCharacteristics()[0]
                            myPeripheralConnected.writeCharacteristic(characteristics.valHandle + 1, b"\x01\x00")
                            # Added to change sampling rate to 100Hz for gen 3 devices
                            uuids = [char.uuid for char in services.getCharacteristics()]
                            if self.freqUUID317 in uuids:
                                idx = uuids.index(self.freqUUID317)
                                char = services.getCharacteristics()[idx]
                                myPeripheralConnected.writeCharacteristic(char.valHandle, b"\x64")

                            elif self.freqUUID319 in uuids:
                                idx = uuids.index(self.freqUUID319)
                                char = services.getCharacteristics()[idx]
                                myPeripheralConnected.writeCharacteristic(char.valHandle, b"\x64")
                            # End
                            continue

                        if services.uuid == self.serviceUUID10TT:
                            myPeripheralConnected.gen = '10TT'
                            myPeripheralConnected.uuid = self.serviceUUID10TT
                            self.ble_generateTenChannel(myPeripheralConnected)
                            characteristics = services.getCharacteristics()[0]
                            myPeripheralConnected.writeCharacteristic(characteristics.valHandle + 1, b"\x01\x00")
                            continue

    def ble_disconnectOnePeripheral(self, myDeviceAddr):
        #print("\033[0;35;40m ble_disconnectOnePeripheral()\033[0m")

        """
        Disconnect one StretchSense device which is connected using its address, and remove it from
        listPeripheralIsConnected.
        The peripheral is also removed from listPeripheralInUse.

        :param myDeviceAddr: string :
            Address of the device that you want to disconnect.

        """

        numberOfPeripheralConnected = len(self.listPeripheralIsConnected)

        if numberOfPeripheralConnected > 0:

            i = 0
            for myPeripheralConnected in self.listPeripheralIsConnected:

                if myPeripheralConnected.addr == myDeviceAddr:
                    del self.listPeripheralIsConnected[i:i + 10]
                    break
                i += 1

            for myPeripheralInUse in self.listPeripheralInUse:

                if myPeripheralInUse.addr == myDeviceAddr:
                    self.listPeripheralInUse.remove(myPeripheralInUse)
                    myPeripheralInUse.disconnect()

    def ble_disconnectAllPeripherals(self):
        #print("\033[0;35;40m ble_disconnectAllPeripherals()\033[0m")

        """

        Disconnect all StretchSense devices which are connected, and remove them from both
        listPeripheralIsConnected and listPeripheralInUse.

        """

        for myPeripheralInUse in self.listPeripheralInUse:
            myPeripheralInUse.disconnect()
        del self.listPeripheralAvailable[1:]
        del self.listPeripheralIsConnected[1:]
        del self.listPeripheralInUse[1:]

    """

     Functions : Discover/Generate/Update Services & Characteristics, and wait for notifications

    """

    def ble_generateOneChannel(self, peripheral):
        #print("\033[0;35;40m ble_generateOneChannel()\033[0m")

        """

        When a StretchSense gen2 device is connected we create a newSensor receiving the
        specifications needed.

        :param peripheral: Peripheral :
            Using the BLE Peripheral format we can convert it into a StretchSensePeripheral
            format easier to use.

        :param periphUUID: UUID :
            UUID of the StretchSense circuit

        """

        # We create a newSensor with the address
        newSensor = StretchSensePeripheral()
        newSensor.addr = peripheral.addr
        newSensor.uuid = peripheral.uuid
        newSensor.value = 0
        newSensor.gen = peripheral.gen
        newSensor.channelNumber = 0

        self.listPeripheralIsConnected.append(newSensor)

    def ble_generateTenChannel(self, peripheral):
        #print("\033[0;35;40m ble_generateTenChannel()\033[0m")

        """

        When a StretchSense gen3 device is connected we create ten newSensor receiving the
        specifications needed.

        :param peripheral: Peripheral :
            Using the BLE Peripheral format we can convert it into a StretchSensePeripheral
            format easier to use.

        """

        # We create ten newSensor with the address
        newSensor1 = StretchSensePeripheral()
        newSensor1.addr = peripheral.addr
        newSensor1.uuid = peripheral.uuid
        newSensor1.value = 0
        newSensor1.gen = peripheral.gen
        newSensor1.channelNumber = 0

        newSensor2 = StretchSensePeripheral()
        newSensor2.addr = peripheral.addr
        newSensor2.uuid = peripheral.uuid
        newSensor2.value = 0
        newSensor2.gen = peripheral.gen
        newSensor2.channelNumber = 1

        newSensor3 = StretchSensePeripheral()
        newSensor3.addr = peripheral.addr
        newSensor3.uuid = peripheral.uuid
        newSensor3.value = 0
        newSensor3.gen = peripheral.gen
        newSensor3.channelNumber = 2

        newSensor4 = StretchSensePeripheral()
        newSensor4.addr = peripheral.addr
        newSensor4.uuid = peripheral.uuid
        newSensor4.value = 0
        newSensor4.gen = peripheral.gen
        newSensor4.channelNumber = 3

        newSensor5 = StretchSensePeripheral()
        newSensor5.addr = peripheral.addr
        newSensor5.uuid = peripheral.uuid
        newSensor5.value = 0
        newSensor5.gen = peripheral.gen
        newSensor5.channelNumber = 4

        newSensor6 = StretchSensePeripheral()
        newSensor6.addr = peripheral.addr
        newSensor6.uuid = peripheral.uuid
        newSensor6.value = 0
        newSensor6.gen = peripheral.gen
        newSensor6.channelNumber = 5

        newSensor7 = StretchSensePeripheral()
        newSensor7.addr = peripheral.addr
        newSensor7.uuid = peripheral.uuid
        newSensor7.value = 0
        newSensor7.gen = peripheral.gen
        newSensor7.channelNumber = 6

        newSensor8 = StretchSensePeripheral()
        newSensor8.addr = peripheral.addr
        newSensor8.uuid = peripheral.uuid
        newSensor8.value = 0
        newSensor8.gen = peripheral.gen
        newSensor8.channelNumber = 7

        newSensor9 = StretchSensePeripheral()
        newSensor9.addr = peripheral.addr
        newSensor9.uuid = peripheral.uuid
        newSensor9.value = 0
        newSensor9.gen = peripheral.gen
        newSensor9.channelNumber = 8

        newSensor10 = StretchSensePeripheral()
        newSensor10.addr = peripheral.addr
        newSensor10.uuid = peripheral.uuid
        newSensor10.value = 0
        newSensor10.gen = peripheral.gen
        newSensor10.channelNumber = 9

        self.listPeripheralIsConnected.append(newSensor1)
        self.listPeripheralIsConnected.append(newSensor2)
        self.listPeripheralIsConnected.append(newSensor3)
        self.listPeripheralIsConnected.append(newSensor4)
        self.listPeripheralIsConnected.append(newSensor5)
        self.listPeripheralIsConnected.append(newSensor6)
        self.listPeripheralIsConnected.append(newSensor7)
        self.listPeripheralIsConnected.append(newSensor8)
        self.listPeripheralIsConnected.append(newSensor9)
        self.listPeripheralIsConnected.append(newSensor10)

    def ble_discoverServices(self):
        #print("\033[0;35;40m ble_discoverServices()\033[0m")

        """

        Display on the terminal all the services for each StretchSense devices connected.

        """

        for myPeripheral in self.listPeripheralInUse:

            listOfServices = sorted(myPeripheral.services, key=lambda services: services.hndStart)

            for services in listOfServices:

                print(services)

    def ble_discoverCharacteristics(self):
        #print("\033[0;35;40m ble_discoverCharacteristics()\033[0m")

        """
        Display on the terminal all the characteristics for each StretchSense devices connected.

        """

        for myPeripheral in self.listPeripheralInUse:

            listOfServices = sorted(myPeripheral.services, key=lambda services: services.hndStart)

            for services in listOfServices:

                characteristics = services.getCharacteristics()

                for chars in characteristics:

                    print(chars)

    def ble_updateOneChannelWithNotifications(self, data, addr):
        #print("\033[0;35;40m ble_updateOneChannelWithNotifications()\033[0m")

        """

        Update data from one or more StretchSense gen2 devices connected using BLE notifications
        and stores its value.

        :param data: UTF-8 characters :
            data transmitted by the device once a notification is detected.

        :param addr: string :
            Address of the gen2 we want to update.

        """

        numberOfPeripheralConnected = len(globalSensor)

        if numberOfPeripheralConnected >= 1:
            for myPeripheral in globalSensor:
                if myPeripheral.addr == addr:
                    decimalValue = int(binascii.b2a_hex(data), 16) / 10.0
                    myPeripheral.value = decimalValue
                    #print("myPeripheral.value = ", myPeripheral.value)

    def ble_updateOneChannel(self):
        #print("\033[0;35;40m ble_updateOneChannel()\033[0m")

        """

        Update once the value of every StretchSense gen2 devices connected without using BLE notifications.

        """

        numberOfPeripheralConnected = len(self.listPeripheralIsConnected)

        if numberOfPeripheralConnected >= 1:

            for myPeripheral in self.listPeripheralInUse:

                for myPeripheralConnected in self.listPeripheralIsConnected:

                    if (myPeripheralConnected.gen == 2) & (myPeripheralConnected.addr == myPeripheral.addr):
                        characteristics = myPeripheral.getCharacteristics()

                        for chars in characteristics:

                            if chars.uuid == self.dataUUID2:
                                handler = chars.getHandle()
                                value = myPeripheral.readCharacteristic(handler)
                                decimalValue = int(binascii.b2a_hex(value), 16) / 10.0
                                myPeripheralConnected.value = decimalValue
                                print("myPeripheralConnected.value = ", myPeripheralConnected.value)

    def ble_updateTenChannelWithNotifications(self, data, addr):
        #print("\033[0;35;40m ble_updateTenChannelWithNotifications()\033[0m")

        """

        Update data from one or more StretchSense gen3 devices connected using BLE notifications
        and stores its value.

        :param data: UTF-8 characters :
            data transmitted by the device once a notification is detected.

        :param addr: string :
            Address of the gen3 we want to update.

        """

        numberOfPeripheralConnected = len(globalSensor)
        if numberOfPeripheralConnected >= 10:
            for myPeripheral in globalSensor:
                if myPeripheral.addr == addr:
                    index = globalSensor.index(myPeripheral)

                    decimalValue = (binascii.b2a_hex(data))
                    splitted = [decimalValue[i:i + 4] for i in range(0, len(decimalValue), 4)]
                    # val = np.array(list(map(lambda x: int((x), 16) / 10, splitted)))
                    # print(val)
                    globalSensor[index + 0].value = int((splitted[0]), 16) / 10.0
                    globalSensor[index + 1].value = int((splitted[1]), 16) / 10.0
                    globalSensor[index + 2].value = int((splitted[2]), 16) / 10.0
                    globalSensor[index + 3].value = int((splitted[3]), 16) / 10.0
                    globalSensor[index + 4].value = int((splitted[4]), 16) / 10.0
                    globalSensor[index + 5].value = int((splitted[5]), 16) / 10.0
                    globalSensor[index + 6].value = int((splitted[6]), 16) / 10.0
                    globalSensor[index + 7].value = int((splitted[7]), 16) / 10.0
                    globalSensor[index + 8].value = int((splitted[8]), 16) / 10.0
                    globalSensor[index + 9].value = int((splitted[9]), 16) / 10.0

                    break

    def ble_updateTenChannel(self):
        #print("\033[0;35;40m ble_updateTenChannel()\033[0m")

        """

        Update once the value of every StretchSense gen3 devices connected without BLE using notifications.

        """

        numberOfPeripheralConnected = len(self.listPeripheralIsConnected)

        if numberOfPeripheralConnected >= 10:

            for myPeripheral in self.listPeripheralInUse:

                for myPeripheralConnected in self.listPeripheralIsConnected:

                    if (myPeripheralConnected.gen == 3) & (myPeripheralConnected.addr == myPeripheral.deviceAddr):
                        characteristics = myPeripheral.getCharacteristics()

                        for chars in characteristics:

                            if chars.uuid == (self.dataUUID3 or self.dataUUID10TT):
                                handler = chars.getHandle()
                                value = myPeripheral.readCharacteristic(handler)
                                decimalValue = (binascii.b2a_hex(value))
                                splitted = [decimalValue[i:i + 4] for i in range(0, len(decimalValue), 4)]

                                for channel in range(0, 10, 1):

                                    if channel == myPeripheralConnected.channelNumber:
                                        myPeripheralConnected.value = int((splitted[channel]), 16) / 10.0
                                        print("myPeripheralConnected.value = ", myPeripheralConnected.value)
                                        break
                                break

    def ble_updateAllPeripherals(self):
        #print("\033[0;35;40m ble_updateAllPeripherals()\033[0m")

        """

        Update the value of the capacitance of each StretchSense devices which are connected.

        """

        listPeripheralUpdated = []
        numberOfPeripheralInUse = len(self.listPeripheralInUse)

        if numberOfPeripheralInUse > 0:

            for myPeripheral in self.listPeripheralInUse:

                if myPeripheral.deviceAddr != '':
                    skipThisPeripheral = False

                    for myPeripheralUpdated in listPeripheralUpdated:

                        if myPeripheral.addr == myPeripheralUpdated.deviceAddr:
                            skipThisPeripheral = True

                    if skipThisPeripheral is False:
                        listOfServices = sorted(myPeripheral.services, key=lambda services: services.hndStart)

                        for services in listOfServices:

                            if services.hndStart == services.hndEnd:
                                continue

                            if services.uuid == self.serviceUUID2:
                                self.ble_updateOneChannel()
                                listPeripheralUpdated.append(myPeripheral)
                                continue

                            if services.uuid == self.serviceUUID3:
                                self.ble_updateTenChannel()
                                listPeripheralUpdated.append(myPeripheral)
                                continue

    def ble_waitNotifications(self):
        #print("\033[0;35;40m ble_waitNotifications()\033[0m")

        """
        When called, run into all connected devices waiting for notification from each of them
        and store the new data in their value slot.

        """

        numberOfPeripheralInUse = len(self.listPeripheralInUse)

        if numberOfPeripheralInUse > 0:

            global globalSensor
            globalSensor = self.listPeripheralIsConnected

            for myPeripheral in self.listPeripheralInUse:
                if myPeripheral.waitForNotifications(0.9):
                    continue
                self.listPeripheralIsConnected = globalSensor

    """

    Functions : Lists of Peripherals

    """

    def ble_getListPeripheralAvailable(self):
        #print("\033[0;35;40m ble_getListPeripheralAvailable()\033[0m")

        """

        Returns the list of all devices available in the area.

        :returns: [Peripheral] : List of all the devices available.

        """

        return self.listPeripheralAvailable

    def ble_getListAddrPeripheralAvailable(self):
        #print("\033[0;35;40m ble_getListAddrPeripheralAvailable()\033[0m")

        """

        Returns the list of all devices address available in the area.

        :returns: [addr] : List of all devices available addresses.

        """

        listAddr = []
        numberOfPeripheralAvailable = len(self.listPeripheralAvailable)

        if (self.listPeripheralAvailable[0].addr == ''):
            return 0

        elif numberOfPeripheralAvailable != 0:

            for i in [numberOfPeripheralAvailable - 1]:
                listAddr.append(self.listPeripheralAvailable[i].addr)
                print(listAddr)

        return listAddr

    def ble_getListPeripheralIsConnected(self):
        #print("\033[0;35;40m ble_getListPeripheralIsConnected()\033[0m")

        """

        Returns the list of all devices connected.

        :returns: [StretchSensePeripheral] : List of all devices connected.

        """

        return self.listPeripheralIsConnected

    def ble_getListPeripheralOnceConnected(self):
        #print("\033[0;35;40m ble_getListPeripheralOnceConnected()\033[0m")

        """

        Returns the list of all devices once connected.

        :returns: [StretchSensePeripheral] : List of all devices once connected.

        """

        return self.listPeripheralIsOnceConnected

    def ble_getListPeripheralInUse(self):
        #print("\033[0;35;40m ble_getListPeripheralInUse()\033[0m")

        """

        Returns the list of all devices currently in use.

        :returns: [Peripheral] : List of all devices we are using.

        """

        return self.listPeripheralInUse

    def ble_listToCsv(self):
        #print("\033[0;35;40m ble_listToCsv()\033[0m")

        """

        Displays on the terminal the values of each connected devices in a csv format.

        """

        listToCsv = ""
        numberOfPeripheralConnected = len(self.listPeripheralIsConnected)

        if numberOfPeripheralConnected >= 1:

            for myPeripheral in self.listPeripheralIsConnected:
                listToCsv += ("%s ," % myPeripheral.value)

        listToCsv += ("\n")
        print(listToCsv)

    def ble_getValuesCsv(self):
        #print("\033[0;35;40m ble_getValuesCsv()\033[0m")

        """

        Returns the values of each connected devices in a csv format.

        :returns: string : Values of each connected devices.

        """

        listToReturn = ""
        listOfFloat = []
        numberOfPeripheralConnected = len(self.listPeripheralIsConnected)

        if numberOfPeripheralConnected > 0:

            for myPeripheral in self.listPeripheralIsConnected:
                listOfFloat.append(myPeripheral.value)
                listToReturn += ("%s , " % myPeripheral.value)

        return listOfFloat


"""

Class StretchSenseDelegate : Class to handle the BLE notifications

"""


class StretchSenseDelegate(btle.DefaultDelegate):
    #print("\033[0;35;40m StretchSenseDelegate()\033[0m")

    def __init__(self, peripheral):
        #print("\033[0;35;40m __init__().StretchSenseDelegate()\033[0m")

        btle.DefaultDelegate.__init__(self)
        self.peripheral = peripheral
        self.addr = self.peripheral.addr

    def handleNotification(self, cHandle, data):
        #print("\033[0;35;40m StretchSenseDelegateHandleNotification()\033[0m")

        for myPeripheral in globalSensor:

            if myPeripheral.addr == self.addr:
                if myPeripheral.uuid == StretchSenseAPI.serviceUUID2:
                    StretchSenseAPI.ble_updateOneChannelWithNotifications(self, data, self.addr)

                if myPeripheral.uuid == StretchSenseAPI.serviceUUID3:
                    StretchSenseAPI.ble_updateTenChannelWithNotifications(self, data, self.addr)

                if myPeripheral.uuid == StretchSenseAPI.serviceUUID10TT:
                    StretchSenseAPI.ble_updateTenChannelWithNotifications(self, data, self.addr)
                break

"""

Global lists of values

"""

globalSensor = [StretchSensePeripheral()]


"""    Main initialisation

Main initial declaration to compile examples and files that you are using.

"""

if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit("Usage:\n  %s <mac-address> [random]" % sys.argv[0])

    if not os.path.isfile(btle.helperExe):
        raise ImportError("Cannot find required executable '%s'" % btle.helperExe)

    devAddr = sys.argv[1]
    if len(sys.argv) == 3:
        addrType = sys.argv[2]
    else:
        addrType = btle.ADDR_TYPE_PUBLIC
    print("Connecting to: {}, address type: {}".format(devAddr, addrType))
    conn = btle.Peripheral(devAddr, addrType)
    try:
        for svc in conn.services:
            print(str(svc), ":")
            for ch in svc.getCharacteristics():
                print("    {}, hnd={}, supports {}".format(ch, hex(ch.handle), ch.propertiesToString()))
                chName = btle.AssignedNumbers.getCommonName(ch.uuid)
                if (ch.supportsRead()):
                    try:
                        print("    ->", repr(ch.read()))
                    except btle.BTLEException as e:
                        print("    ->", e)

    finally:
        conn.disconnect()
