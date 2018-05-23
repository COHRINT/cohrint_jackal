# cohrint_jackal
All required files for creating new jackal robot

# For installing udev rules
Example:
- $ sudo cp 51-ros-indigo-jackal-firmware.rules /lib/udev/rules.d/
- # udevadm control --reload-rules && udevadm trigger

# For establishing GPS
[As root] usermod -a -G dialout MY_USER_NAME 

Notes:
- Green LED indicates antenna is recieving data 
- SUDO permissions wipe upon logout
- dmesg |grep tty to identify USB port
