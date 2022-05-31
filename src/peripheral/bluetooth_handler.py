"""Handler for StretchSense Peripherals using Bluetooth."""

from typing import List, Dict, Optional
import numpy as np
from bluepy import btle
from peripheral import stretchsense_delegate as ssd
from peripheral import stretchsense_peripheral as ssp
import yaml

class BluetoothHandler:
    """Handles connecting to a StretchSense device via Bluetooth Low Energy."""
    def __init__(self, pkg_directory: str):
        self._pkg_directory: str = pkg_directory

        # get name of known peripherals from yaml file
        known_peripherals_yaml = open(self._pkg_directory + 
                                      "/src/data/known_peripherals.yaml")
        self._known_peripherals: Dict = yaml.load(known_peripherals_yaml,
                                                  Loader=yaml.FullLoader)

    def _get_available_peripherals(self) -> List:
        """Gets a list of the available Stretchsense Peripherals."""

        scanner = btle.Scanner().withDelegate(
            ssd.StretchSenseDelegate())
        devices = scanner.scan(3)
        available_peripherals = []
                   
        for dev in devices:
            for (sdid, desc, val) in dev.getScanData():
                if val == 'StretchSense':
                    available_peripherals.append(dev.addr)
        return available_peripherals

    def _select_peripheral(self, 
                           available_peripherals: List) -> str:
        """Allow user to select a peripheral to connect to.
        
        Args:
            available_peripherals:
                The list of the addresses of the available StretchSense
                bluetooth devices.

        Returns:
            The address of the user-selected peripheral.
        """
        
        print('Select a glove to connect\n')
        for idx, addr in enumerate(available_peripherals):
            if addr in self._known_peripherals.keys():
                print(f"{idx}. {addr}")
            else:
                print(f"{idx}. Unknown, addr: {addr}")

        selected = int(input("\n Select glove from 0 to " +
                                f"{len(available_peripherals) - 1}: "))
        return available_peripherals[selected]
    
    def _get_glove(self, address: str) -> ssp.StretchSensePeripheral:
        """Creates a glove object corresponding to the given address."""

        if address in self._known_peripherals.keys():
            if self._known_peripherals[address] == "left_glove":
                # JUST A PLACEHOLDER UNTIL I WRITE THE LEFT GLOVE
                print("\n Left Glove detected.")
                return ssp.LeftStretchSenseGlove(address, self._pkg_directory)
            elif self._known_peripherals[address] == "right_glove":
                print("\n Right Glove detected.")
                return ssp.RightStretchSenseGlove(address, self._pkg_directory)

        else:
            # unknown peripheral selected
            side = input("\n Is this a right or left handed glove? R/L: ")
            if side in ["l", "left"]:
                # JUST A PLACEHOLDER UNTIL I WRITE THE LEFT GLOVE
                return ssp.LeftStretchSenseGlove(address, self._pkg_directory)
            elif side in ["r", "right"]:
                return ssp.RightStretchSenseGlove(address, self._pkg_directory)
            else:
                print("\n Error: invalid input")
                return self._get_glove(address)

    def connect_glove(self) -> Optional[ssp.StretchSensePeripheral]:
        """Connect to a selected StretchSense Peripheral.
        
        Scans for available Bluetooth Low Energy devices, allows user to
        choose a Stretchsense Peripheral to connect to, sets the glove up, then
        returns it.

        Returns:
            A StretchSensePeripheral object or None.
        """

        available_peripherals = self._get_available_peripherals()

        # user selects a glove to connect
        if len(available_peripherals) > 0:
            addr = self._select_peripheral(available_peripherals)
            glove = self._get_glove(addr)
            print(f"\n connecting to addr: {addr}")
            glove.setup()
            return glove

        else:
            print(' No gloves found.\n')

    