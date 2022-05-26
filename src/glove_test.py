import rospy, rospkg
from peripheral import bluetooth_handler
rospack = rospkg.RosPack()

if __name__ == "__main__":
    PACKAGE_DIRECTORY = rospack.get_path('stretchsense')
    bth = bluetooth_handler.BluetoothHandler(PACKAGE_DIRECTORY)
    glove = bth.connect_glove()
    if glove:
        for _ in range(10):
            print(glove.read_sensors())