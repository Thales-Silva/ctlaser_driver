# ctlaser_driver

This package provides an interface to operate Optris CTlaser pyrometers from ROS environment.
For now, it relies on using the Digi RealPort CT Ethernet Adapter that creates a TCP server that communicates with the serial interface.
The communication path is as follows:

optris pyrometer --> (serial) --> CT Ethernet Adapter --> (tcp socket server/client) --> Computer with ROS.

This document shows how to propperly configure and use the provided software in three steps:
- How to configure the TCP server on the CT Ethernet Adapter;
- How to install the ctlaser_driver software;
- How to set basic parameters.

## How to install the ctlaser_driver rospackage

## Setting up basic parameters

## Configuring the tcp server on the CT Ethernet Adapter

Plug the ethernet cable to CT Ethernet Adapter and make sure both the adapter and you computer are in the same network. The DHCP server will
most likelly assign an ip address to your device. In case it doesn't, press the adapter reset button while it turned off, then turn it on.
Keep the button pressed until the network lights blink in a pattern 1-5-1. The device will be reset to factory configurations and the DHCP
server will be able to assign a new ip address to you adapter.

- Provided that after these steps the adapter's ip address is known (suppose 146.164.53.228), go your browser and access the firmware webpage
```http://146.164.53.228/login.htm``` as in the figure directly bellow.

![Firware webpage](./images/adapter-web.png)

- Login with the default ```user name: root``` and ```password: dbps```. Proceed to ```Serial Ports``` in Configurations, then click the according
port as in the figure bellow. A new tab with Port Profile Settings will be shown.

![Firware webpage](./images/adapter-config1.png)

- After clicking on the desired port, the Serial Port Configuration - Interface will show up. Click on the Port Profile Settings tab, then
Change Profile. You will be prompted to choose one option on the Select Port Profile tab. Choose TCP Sockets as in the figure bellow.

![Firware webpage](./images/adapter-config3.png)
