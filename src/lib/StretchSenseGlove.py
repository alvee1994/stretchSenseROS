#!/usr/bin/env python3

from bluepy.btle import Scanner, DefaultDelegate
import bluepy.btle as btle
from easydict import EasyDict

class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if isNewDev:
            print("Discovered device", dev.addr)
        elif isNewData:
            print ("Received new data from", dev.addr)

    def handleNotification(self, cHandle, data):
        #print("\033[0;35;40m StretchSenseDelegateHandleNotification()\033[0m")
