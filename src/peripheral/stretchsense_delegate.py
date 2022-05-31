import binascii
import numpy as np
from bluepy import btle
from typing import List, Optional

class StretchSenseDelegate(btle.DefaultDelegate):
    """Handles notifications from the glove
    
    This class handles the notifications sent from the glove via Bluetooth
    Low Energy.

    Attributes:
        capacitance:
            A numpy array used to store the capacitance data read from the
            stretchsense peripheral.
    """

    def __init__(self):
        """Constructor for StretchSenseDelegate"""
        super().__init__()
        self.capacitance: np.ndarray

    def handleNotification(self, cHandle, data) -> None:
        """Implementation of the handleNotification method in DefaultDelegate.
        
        Takes in bytestring data from the glove via BLE and converts it into
        capacitatance data in the form of a numpy array vector of type int.

        Args:
            cHandle:
                Unused parameter.
            data:
                Bytestring data from glove's sensors.
        """

        hex_vals = (binascii.b2a_hex(data))
        split_vals = np.array([int(hex_vals[i:i + 4], 16) / 10 
                               for i in range(0, len(hex_vals), 4)])
        capacitance = split_vals[split_vals != 0]
        self.capacitance = capacitance