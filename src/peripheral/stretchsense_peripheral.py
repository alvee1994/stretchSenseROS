"""Classes that encapsulate a single Stretchsense peripheral."""

from bluepy import btle
from abc import ABC
import numpy as np
from peripheral import stretchsense_delegate
from typing import Optional

class StretchSensePeripheral(btle.Peripheral, ABC):
    """An abstract class providing a blueprint for concrete peripherals.
    
    Attributes:
        ACTIVE_SENSORS:
            A numpy array representing the sensors that represent each joint.
        NUM_SENSORS:
            An integer representing the number of sensors on the glove.
        SIDE:
            A string representing which hand the glove is for.
    """

    def __init__(self, address: str, pkg_directory: str):
        super().__init__(address, "random")
        self.ACTIVE_SENSORS: np.ndarray
        self.NUM_SENSORS: int
        self.SIDE: str
        self._pkg_directory: str = pkg_directory
        self._SERVICE_UUID: str
        self._delegate = stretchsense_delegate.StretchSenseDelegate()
        self._address: str = address

    def get_default_theta_path(self) -> str:
        return self._pkg_directory

    def setup(self) -> None:
        """Sets up the glove for data collection."""

        self.withDelegate(self._delegate)
        print(f"connected to {self._address}")
        svc = self.getServiceByUUID(self._SERVICE_UUID)
        char = svc.getCharacteristics()[0]
        handle = char.valHandle
        self.writeCharacteristic(handle + 1, b'\x01\x00')  # turn on notifications
        self.writeCharacteristic(29, b'\x5a')  # change sampling rate to 90Hz
        
    def read_sensors(self) -> Optional[np.ndarray]:
        """Gets a sample of capacitance data.

        Waits for the glove to send a sample of capacitance data with a timeout
        of 1 second. Then retrieves the capacitatance data from
        the delegate and returns it if it is a valid data set.

        Returns:
            A numpy array with n capacitance readings where n = the number of
            sensors on the glove or None.
        """

        if self.waitForNotifications(1.0):
            cap = self._delegate.capacitance
            if len(cap) == self.NUM_SENSORS:
                return cap
            
class RightStretchSenseGlove(StretchSensePeripheral):
    """Represents a particular right handed Stretchsense glove."""

    def __init__(self, address: str, pkg_directory: str):
        super().__init__(address, pkg_directory)

        # Set up constants
        self._SERVICE_UUID: str = '00001701-7374-7265-7563-6873656e7365'
        self.ACTIVE_SENSORS: np.ndarray = np.array([
            [1, 1, 0, 1, 0, 0, 0, 0], # thumb
            [0, 1, 1, 1, 0, 0, 0, 0], # thumb
            [0, 0, 0, 0, 1, 0, 0, 0], # index
            [0, 0, 0, 0, 0, 1, 0, 0], # middle
            [0, 0, 0, 0, 0, 0, 1, 0], # ring
            [0, 0, 0, 0, 0, 0, 0, 1]  # pinky
        ])
        self.NUM_SENSORS: int = 7
        self.SIDE: str = "right"

    def get_default_theta_path(self) -> str:
        """Gets the file path to the default theta values.
        
        Returns:
            A string containing the file path to the csv file containing
            the default theta values for this glove.
        """
        
        return (super().get_default_theta_path()
                + "/src/data/theta_default.csv")


class LeftStretchSenseGlove(StretchSensePeripheral):
    """Represents a particular left handed Stretchsense glove."""

    def __init__(self, address: str, pkg_directory: str):
        super().__init__(address, pkg_directory)

        # Set up constants
        self._SERVICE_UUID: str = '00001701-7374-7265-7563-6873656e7365'
        self.ACTIVE_SENSORS: np.ndarray = np.array([
            [1, 1, 0, 1, 0, 0, 0, 0], # thumb
            [0, 1, 1, 1, 0, 0, 0, 0], # thumb
            [0, 0, 0, 0, 1, 0, 0, 0], # index
            [0, 0, 0, 0, 0, 1, 0, 0], # middle
            [0, 0, 0, 0, 0, 0, 1, 0], # ring
            [0, 0, 0, 0, 0, 0, 0, 1]  # pinky
        ])
        self.NUM_SENSORS: int = 7
        self.SIDE: str = "left"

    def get_default_theta_path(self) -> str:
        """Gets the file path to the default theta values.
        
        Returns:
            A string containing the file path to the csv file containing
            the default theta values for this glove.
        """
        
        return (super().get_default_theta_path()
                + "/src/data/theta_default_left.csv")