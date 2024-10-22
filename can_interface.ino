#include <mcp2515.h>
#include <SerialCAN.h>

// Initialize MCP2515 object, where SPI CS is connected to pin 10.
MCP2515 mcp(10);

// Initialize serial CAN bus.
serial_can::SerialCAN serialCAN{&Serial};

// Variable for holding CAN bus frames.
struct can_bus_frame frame;

// Variable for holding CAN serial frames.
serial_can::Frame can_serial_frame; 

// Interrupt flag.
volatile bool interrupt = false;

// Iteration counter.
uint32_t timestamp = 0;

void irqHandler() {
  interrupt = true;
}

void setup() {
  // Message receive interrupt.
  attachInterrupt(0, irqHandler, FALLING);

  // Begin serial communication via SerialCAN interface.
  serialCAN.begin(115200);

  // Initialize MCP2515 for 1Mbps baud @ 16MHz.
  mcp.reset();
  mcp.setBitrate(CAN_1000KBPS, MCP_16MHZ);
  mcp.setNormalMode();
  
  // Give it a second.
  delay(1000);
}

void loop() {
  timestamp += 1;

  if (interrupt) {
    // Inbound message from CAN bus.
    interrupt = false;

    uint8_t irq = mcp.getInterrupts();

    if (irq & MCP2515::CANINTF_RX0IF) {
      if (mcp.readMessage(MCP2515::RXB0, &can_bus_frame) == MCP2515::ERROR_OK) {
        // Frame received from RXB0 message.
      }
    }

    if (irq & MCP2515::CANINTF_RX1IF) {
      if (mcp.readMessage(MCP2515::RXB1, &can_bus_frame) == MCP2515::ERROR_OK) {
        // Frame received from RXB1 message.
      }
    }
  }
}
