This program supports a cheap interface for sending/receiving messages to/from a CAN bus with Python-CAN.

Parts:
1. Arduino Nano (ATmega328)
2. MCP2515 CAN Bus Module
3. Mini-USB Cable

Software:
1. CAN over Serial by Python-CAN https://python-can.readthedocs.io/en/stable/interfaces/serial.html
2. CAN over Serial for Arduino by Henrik Soderlund https://github.com/henriksod/Arduino_CANOverSerial
3. Arduino MCP2515 CAN interface library by autotwp https://github.com/autowp/arduino-mcp2515 

Instructions (Windows):
1. Connect a pin jumper at J1 on MCP2515, and connect MCP2515 to CAN bus.
2. Look at bottom of Arduino Nano board, there is a Schottky diode underneath the mini-USB port: carefully solder a wire to the anode of this diode, this wire will serve as the 5V supply for the MCP2515.
3. Wire MCP2515 to Arduino Nano, ensuring that the 5V pin on the MCP2515 is connected to the wire from Step 3, not the 5V pin on the Arduino Nano.
4. Plug the Arduino Nano into your computer, and take note of which COM port it is using.
5. Download software items 1 and 2 as zip files and load them into a sketch in the Arduino IDE so that Arduino IDE knows where to find them.
6. Download can_interface.ino, load this sketch into Arduino IDE and then load it onto the Arduino Nano via the COM port from Step 4.
7. Python-CAN CAN over Serial interface can now be used to send/receive messages to/from the CAN bus (see test_connection.py for an example).
