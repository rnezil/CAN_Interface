#include <mcp2515.h>
#include <SerialCAN.h>

MCP2515 mcp(10); //initialize MCP2515 object, where SPI CS is connected to pin 10
serial_can::SerialCAN serialCAN{&Serial}; //initialize serial CAN bus
struct can_frame frame; // initialize variable for holding CAN bus frames
serial_can::Frame serial_frame; // initialize variable for holding CAN serial frames
volatile bool interrupt = false;
uint32_t timestamp = 0;

void irqHandler() {
  interrupt = true;
}

void setup() {
  DDRD |= _BV(DDD4);  // set D4 as an output
  PORTD &= !_BV(PORTD4);  // set D4 low

  attachInterrupt(0, irqHandler, FALLING);  // message receive interrupt

  serialCAN.begin(115200); // begin serial communication
  // Serial.println("CAN Interface");
  // Serial.println("------- CAN Read ----------");
  // Serial.println("ID  DLC   DATA");

  mcp.reset();
  mcp.setBitrate(CAN_1000KBPS, MCP_16MHZ);
  mcp.setNormalMode();
  delay(1000);  //give the thing a second

  // frame.can_id = 0x06900001;  // 29-bit CAN identifier 
  // frame.can_dlc = 0x8;    // indicate 8 bytes of data in payload
  // frame.data[0] = 0x3C;   // data is 0011 1100
  // frame.data[1] = 0x4B;
  // frame.data[2] = 0x5A;
  // frame.data[3] = 0x69;
  // frame.data[4] = 0x78;
  // frame.data[5] = 0x87;
  // frame.data[6] = 0x96;
  // frame.data[7] = 0xA5;
}

void loop() {
  timestamp += 1;
  // send a message every 2 seconds
  // if (mcp.sendMessage(MCP2515::TXB1, &frame) == MCP2515::ERROR_OK) {
  //   Serial.println("Send Succeeded");
  // }
  // else {
  //   Serial.println("Send Failed");
  // }

  if (interrupt) {
    // inbound message from CAN bus
    interrupt = false;

    uint8_t irq = mcp.getInterrupts();

    if (irq & MCP2515::CANINTF_RX0IF) {
      if (mcp.readMessage(MCP2515::RXB0, &frame) == MCP2515::ERROR_OK) {
        // frame received from RXB0 message

        // print frame contents
        // Serial.print(frame.can_id, HEX);
        // Serial.print(" ");
        // Serial.print(frame.can_dlc, HEX);
        // Serial.print(" ");

        // for (int i = 0; i<frame.can_dlc; i++) {
        //   Serial.print(frame.data[i], HEX);
        //   Serial.print(" ");
        // }

        // Serial.print("RXB0");
        // Serial.println();
      }
    }

    if (irq & MCP2515::CANINTF_RX1IF) {
      if (mcp.readMessage(MCP2515::RXB1, &frame) == MCP2515::ERROR_OK) {
        // frame received from RXB1 message

        // print frame contents
        // Serial.print(frame.can_id, HEX);
        // Serial.print(" ");
        // Serial.print(frame.can_dlc, HEX);
        // Serial.print(" ");

        // for (int i = 0; i<frame.can_dlc; i++) {
        //   Serial.print(frame.data[i], HEX);
        //   Serial.print(" ");
        // }

        // Serial.print("RXB1");
        // Serial.println();
      }
    }
  }

  
}
