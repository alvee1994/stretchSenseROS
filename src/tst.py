from bluepy import btle
import binascii
import numpy as np

class MyDelegate(btle.DefaultDelegate):
    def __init__(self, params):
        btle.DefaultDelegate.__init__(self)
        # ... initialise here

    def handleNotification(self, cHandle, data):
        decimalValue = (binascii.b2a_hex(data))
#         decimalValue = int(decimalValue,16)
        splitted = [decimalValue[i:i+4] for i in range(0, len(decimalValue),4)]
        val = np.array(list(map(lambda x: int((x),16), splitted)))
        print(val)

# p.disconnect()
# Initialisation  -------
address = 'F8:A6:1C:CB:ED:A6'
p = btle.Peripheral( address, 'random' )
p.setDelegate( MyDelegate(p) )

svc = p.getServiceByUUID('00001701-7374-7265-7563-6873656e7365')
char = svc.getCharacteristics()[0]
handle = char.valHandle
p.writeCharacteristic(handle+1, b'\x01\x00')

while True:
    if p.waitForNotifications(0.9):
        continue

    print ("Waiting...")
