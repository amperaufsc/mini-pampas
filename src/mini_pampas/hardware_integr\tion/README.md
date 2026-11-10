# Hardware Integration

Creating an Ethernet SSH Interface to connect to the Raspberry Pi

1. Identify the name of the Ethernet Interface you are using
   
    nmcli device status

This will list the active connetions, search for the one that has the "Ethernet" label


2. Create a profile for the Board on your current device

    nmcli connection add type ethernet ifname [INTERFACE-NAME] con-name "Raspberry-Crawler"

Replacing "Interface Name" with the connection you found in the first step, and "Raspberry-Crawler" with whatever you may want to name the board


3. Configure the IPv4 to its "shared" mode

     nmcli connection modify "Raspberry-Crawler" ipv4.method shared

This will make it so the Network Manager will automatically attribute an IP Address to the Board


4. Activate the connection



