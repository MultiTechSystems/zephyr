.. mulititech:code-sample:: lorawan-shell
   :name: LoRaWAN class A shell
   :relevant-api: lorawan_api

   Commands to set OTAA device crentials and save to NVM
       Once device credentials have been set the device can join and send uplinks



   

Overview
********

A simple application to demonstrate the :ref:`LoRaWAN subsystem <lorawan_api>` of Zephyr.

Building and Running
********************

This sample can be found under
:zephyr_file:`samples/subsys/lorawan/class_a` in the Zephyr tree.

Before building the sample, make sure to select the correct region in the
``prj.conf`` file.

The following commands build and flash the sample.

.. zephyr-app-commands::
   :zephyr-app: samples/boards/multitech/lorawan/shell
   :board: mulitech_mdot, multitech_xdot, multitech_xdot_ad, multitech_xdot_es
   :goals: build flash
   :compact:

Shell Commands
**************

uart:~$ lw help
   lw - LoRaWAN Commands
   Subcommands:
   deveui   : Set/get [deveui:HEX8]
   joineui  : Set/get [joineui:HEX8]
   appkey   : Set/get [appkey:HEX16]
   save     : Save Config
   join     : Join Network
   send     : <port> <0:UNC,1:CNF> <payload:HEX>

uart:~$ lw deveui 1122334455667788
uart:~$ lw joineui 1122334455667788
uart:~$ lw appkey 11223344556677881122334455667788
uart:~$ lw save
uart:~$ lw join
uart:~$ lw send 1 0 FFEEDD