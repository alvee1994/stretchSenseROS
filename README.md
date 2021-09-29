# StretchSenseROS

This package is used to connect to stretchsense smart glove via bluetooth and publish sensor data to the ROS network.

clone with:
```
git clone git@github.com:alvee1994/stretchSenseROS.git stretchsense
```

in order to use bluepy without sudo, navigate to the module path and run:
sudo setcap 'cap_net_raw,cap_net_admin+eip' bluepy-helper
