# ctlaser_driver

This package serves as an interface for operating Optris CTlaser pyrometers within the ROS environment.
Using the Digi RealPort CT Ethernet Adapter, we use a TCP Socket server/client to facilitate communication with the serial interface.

### Communication Flow:

1. **Optris Pyrometer** ⟶ (Serial) ⟶ **CT Ethernet Adapter** ⟶ (TCP Socket Server/Client) ⟶ **Computer with ROS**

This document presents three steps to ensure a smooth configuration and utilization of the provided software:

## 1. Install the ctlaser_driver Software.

## 2. Set Basic Parameters.

This section provides insights into operating the `ctlaser_driver` according to your specific requirements.
Tune the settings to achieve optimal performance while reading the temperature of a given surface with Optris
CTlaser pyrometers.

By following these three steps, you'll be well-equipped to harness the full potential of the `ctlaser_driver` package, enhancing your
experience in working with Optris CTlaser pyrometers within the ROS framework. Feel free to explore the documentation for more in-depth
information and troubleshooting tips.

## 3. Configure the TCP Server on the CT Ethernet Adapter.

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

- After choosing TCP Sockets you will be prompted to enable the desired TCP ports. The communication between the ctlaser_driver and the device is made
using the Raw TCP port which defaults to port ```port: 2101```. As in the figure bellow, enable at least the Raw TCP and Telnet so it gets easier to check
for communication problems using Telnet.

![Firware webpage](./images/adapter-config4.png)

- Another fundamental step is to configure the serial port settings in order for the adapter to be able to communicate with the serial port. In order to do so,
still on the Serial Port Configurations - Interface tab, right bellow the Port Profile Settings you will find Basic Serial Settings. As in the figure bellow,
set the baud rate to the same one configured on the device. The other properties can be found in pyrometer's the manual.

![Firware webpage](./images/adapter-config5.png)
