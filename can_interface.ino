/*
 * Ryland Nezil
 * Fall 2024
 * University of Victoria, Canada
*/

#include <mcp2515.h>
#include <SerialCAN.h>

// Initialize MCP2515 object, where SPI CS is connected to pin 10.
MCP2515 can_bus(10);

// Initialize serial CAN bus.
serial_can::SerialCAN can_serial{&Serial};

// Variable for holding CAN bus frames.
struct can_frame can_bus_frame;

// Variable for holding CAN serial frames.
serial_can::Frame can_serial_frame; 

// Interrupt flag.
volatile bool interrupt = false;

// Reply flag.
volatile bool reply = false;

// Iteration counter.
uint32_t timestamp = 0;

// CAN serial timeout in ms.
uint32_t timeout = 100;

// Interrupt request handler for inbound CAN bus frames.
void irqHandler() {
  interrupt = true;
}

void setup() {
  // Message receive interrupt.
  attachInterrupt(0, irqHandler, FALLING);

  // Begin serial communication via SerialCAN interface.
  can_serial.begin(115200);

  // Initialize MCP2515 for 1Mbps baud @ 16MHz.
  can_bus.reset();
  can_bus.setBitrate(CAN_1000KBPS, MCP_16MHZ);
  can_bus.setNormalMode();
  
  // Give it a second.
  delay(1000);
}

void loop() {
  timestamp += 1;
  if (reply) {
    // If waiting on reply from cartridge...
    if (interrupt) {
      // Read CAN bus frame.
      interrupt = false;

      uint8_t irq = can_bus.getInterrupts();

      if (irq & MCP2515::CANINTF_RX0IF) {
        if (can_bus.readMessage(MCP2515::RXB0, &can_bus_frame) == MCP2515::ERROR_OK) {
          // Frame received from RXB0 message.
          // Copy message contents.
          can_serial_frame.arbitration_id = can_bus_frame.can_id ;
          can_serial_frame.dlc = can_bus_frame.can_dlc;
          for (int i = 0; i < can_bus_frame.can_dlc; i++) {
            can_serial_frame.payload[i] = can_bus_frame.data[i];
          }

          // Send message to CAN serial.
          can_serial.send(&can_serial_frame, timestamp);

          // Indicate that reply has been sent.
          reply = false;
        }
      }

      if (irq & MCP2515::CANINTF_RX1IF) {
        if (can_bus.readMessage(MCP2515::RXB1, &can_bus_frame) == MCP2515::ERROR_OK) {
          // Frame received from RXB1 message.
          // Copy message contents.
          can_serial_frame.arbitration_id = can_bus_frame.can_id ;
          can_serial_frame.dlc = can_bus_frame.can_dlc;
          for (int i = 0; i < can_bus_frame.can_dlc; i++) {
            can_serial_frame.payload[i] = can_bus_frame.data[i];
          }

          // Send message to CAN serial.
          can_serial.send(&can_serial_frame, timestamp);

          // Indicate that reply has been sent.
          reply = false;
        }
      }

      if (reply) {
        // Error: failed to read inbound message.
        reply = false;
      }
    }
  }
  else{
    // If waiting on instruction from master...
    if (can_serial.receive(&can_serial_frame, timeout)) {
      // Copy message contents.
      can_bus_frame.can_id = can_serial_frame.arbitration_id;
      can_bus_frame.can_dlc = can_serial_frame.dlc;
      for (int i = 0; i < can_bus_frame.can_dlc; i++) {
        can_bus_frame.data[i] = can_serial_frame.payload[i];
      }

      // Send message to CAN bus.
      can_bus.sendMessage(MCP2515::TXB1, &can_bus_frame);

      // Indicate that a reply is needed.
      if (can_serial_frame.can_dlc == 0) {
        reply = true;
      }
    }
  }
}

/**********************************************************************************************
 * SerialCAN, CAN communication over Serial bus - Version 1.0.0
 * by Henrik Söderlund <henrik.a.soderlund@gmail.com>
 *
 * Copyright (c) 2023 Henrik Söderlund

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 **********************************************************************************************/
