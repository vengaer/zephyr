.. zephyr:code-sample:: dsa-ksz8463mli
   :name: KSZ8463MLI DSA sample
   :relevant-api: dsa_core

   Microchip KSZ8463MLI DSA sample.

Overview
********

Sample showing how to configure Zephyr for using the Microchip KSZ8463MLI
3-port Ethernet switch.

Source code for the sample is available at
:zephyr_file:`samples/net/ethernet/dsa_ksz8463mli`.

Build and flash the example code, connect an Ethernet cable to one, or both,
of the ports on the switch, open the UART and run ``net iface`` in the shell.
You should see an interface, the name of which is on the format ``swpX`` where
``X`` is the zero-based port index, for each user port on the switch

.. note::

	This sample uses MII. Using RMII to communicate with a KSZ8463RL or
	KSZ8463FRL instead requires changes to the pins being mutex as well as
	changing the ``phy-connection-type`` strings to ``"rmii"``.

Requirements
************

- :ref:`networking_with_host`

Building and Running
********************

Build the sample by running

.. zephyr-app-commands::
   :zephyr-app: samples/net/ethernet/dsa_ksz8463mli
   :board: <board to use>
   :conf: prj.conf
   :goals: build
   :compact:
