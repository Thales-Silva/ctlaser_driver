# ctlaser_driver

This package provides an interface to operate Optris CTlaser pyrometers from ROS environment.
For now, it relies on using the Digi RealPort CT Ethernet Adapter that creates a TCP server that communicates with the serial interface.
The communication path is as follows:
optris pyrometer --> (serial) --> CT Ethernet Adapter --> (tcp socket server/client) --> Computer with ROS.

This document shows how to propperly configure and use the provided software in three steps:
- How to configure the TCP server on the CT Ethernet Adapter;
- How to install the ctlaser_driver software;
- How to set basic parameters.

## Configuring the tcp server on the CT Ethernet Adapter

