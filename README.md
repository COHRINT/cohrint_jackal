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

# For Setting up Individal Jackal Network

Chain:
MSI -> Gig E Switch -> PoE Injector -> Rocket M2

To Set Up M2:
- Go to 192.168.1.20
- Disable airMAX if present
- Under WIRELESS:
	- Set to Station
	- Scan and connect to desired AP [lock]
- Under NETWORK:
	- Set to bridge
	- Choose static IP
		- IP: 192.168.1.xxx
		- Netmask: 255.255.255.0
		- Gateway IP: 192.168.1.1

* If problems connecting try editing the /etc/network/interfaces *
-Add: 
	- allow-hotplug eth0
	- iface eth0 inet static
	- address 192.168.1.xxx
	- gateway 192.168.1.1
	- netmask 255.255.255.0