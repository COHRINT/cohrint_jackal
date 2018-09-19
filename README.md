# cohrint_jackal
All required files for creating new jackal robot

# For installing udev rules
```
$ sudo cp udev/* /lib/udev/rules.d/
$ sudo udevadm control --reload-rules && sudo udevadm trigger
```
You can now plug in necessary jackal peripherals, and they will be recognized.

# For establishing GPS
```
$ usermod -a -G dialout MY_USER_NAME
```

Notes:
- Green LED indicates antenna is recieving data 
- SUDO permissions wipe upon logout
- dmesg |grep tty to identify USB port

# For Setting up Individal Jackal Network

Chain:
MSI -> Gig E Switch -> PoE Injector -> Rocket M2

To Set Up M2:
- Edit the /etc/network/interfaces and restart... sudo service networking/network-manager/resolvconf restart
-Add: 
	- allow-hotplug eth0
	- iface eth0 inet static
	- address 192.168.1.xxx
	- gateway 192.168.1.1
	- netmask 255.255.255.0

- Go to 192.168.1.20
- Disable airMAX if present
- Under WIRELESS:
	- Set to Station
	- Scan and connect to desired AP [lock]
- Under NETWORK:
	- Set to bridge
- Test and Apply changes
- revert /etc/network/interfaces


