#include "EEPROM.h"
#include "cc1101.h"

/*
DON'T PANIC

Seriously... All this stuff is annoyingly difficult to learn because the data sheets are 
close enought to incomprehensible and the examples are obtuse and...

Working with microcontrollers doesn't need to be so difficult. It's their fault, not yours.

Better writers, better social advocates and better engagement would make this a lot more fun ffs...

IT'S NOT YOU, that's the problem...

Good luck 

@gareth__
*/

CC1101 cc1101;

SPI spi;

// Texas Instruments CC1101 reference
// http://www.ti.com/lit/ds/symlink/cc1101.pdf (pdf)

// Pins are

// MOSI: 11
// MISO: 12
// CSN: 10
// SCK: 13
// GDO0: 2
// And 3.3V for power

// flag that a packet has been received
boolean packetAvailable = false;

// interruptPin
int interruptPin = 2;

// Handle interrupt from CC1101 (INT0) gdo0 on pin2(?)
void cc1101signalsInterrupt(void) {
  // set the flag that a package is available
  packetAvailable = true;
}

// SETUP HERE
void setup()
{
  // Setting alternate SCK pin to 14 does not work but pin 13 (LED) works fine.
  // Set alternate SCK pin https://www.pjrc.com/teensy/td_libs_SPI.html#altpins
  // **I am using pin 13 in this sketch**
  // SPI.setSCK(14);
  // spi.init(10,11,12,14);

  //pinMode(ledPin, OUTPUT);
  Serial.begin(9600);

  // Set GDO pin as an input?
  pinMode(interruptPin, INPUT);
  
  // Set interrupt
  attachInterrupt(digitalPinToInterrupt(interruptPin), cc1101signalsInterrupt, FALLING);

  
  // SyncWord
  uint8_t syncH = 0xEE; // 11101110 twice gives you a sync word of 1110111011101110
  uint8_t syncL = 0xEE;

  // Initialize the CC Chip
  cc1101.init();

  cc1101.setSyncWord(syncH, syncL, false);
  cc1101.setCarrierFreq(CFREQ_433);
  cc1101.disableAddressCheck();
  //cc1101.setTxPowerAmp(PA_LowPower);

  // Messing with direct register changes below to then look at in Inspectrum
  // https://github.com/miek/inspectrum

  // MDMCFG4 - channel bandwidth and exponent for calculating data rate
  cc1101.writeReg(0x10, 0xE5);

  // MDMCFG3 - Data Rate
  // DRATE = 1000000.0 * MHZ * (256+drate_m) * powf(2,drate_e) / powf(2,28);
  cc1101.writeReg(0x11, 0xC3);

  // MDMCFG2 - Modulation type (OOK/ASK) / manchester / sync mode
  // 00110010 - DC blocking enabled, OOK/ASK, No manchester, 16/16 syncword bits detected
  cc1101.writeReg(0x12, 0x32); // was 0x30

  // MDMCFG1 - FEC / preamble
  // 00000010 - No FEC, 2 bytes of preamble, reserved, two bit exponent of channel spacing
  cc1101.writeReg(0x13, 0x02); // Changed from 0x22

  // MDMCFG0 - Channel spacing
  cc1101.writeReg(0x14, 0xF8);

  // PKTCTRL0 - Set CRC
  cc1101.writeReg(0x08, 0x04); // Was 0x00 for no CRC check and fixed packet length

  // FREND0 - Select PATABLE index to use when sending a '1'
  cc1101.writeReg(0x22, 0x11);

  // Define what a strong '1' signal is in the second byte of the PATABLE
  set_patable();

  delay(1000);

  Serial.println("Radio initialising\n");
  Serial.print("CC1101_PARTNUM ");
  Serial.println(cc1101.readReg(CC1101_PARTNUM, CC1101_STATUS_REGISTER));
  Serial.print("CC1101_VERSION ");
  Serial.println(cc1101.readReg(CC1101_VERSION, CC1101_STATUS_REGISTER));
  Serial.print("CC1101_MARCSTATE ");
  Serial.println(cc1101.readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & 0x1f);
  Serial.print("PKTCTRL1: PQT / RSSI, LQI & CRC OK - ");
  Serial.println(cc1101.readReg(0x07, CC1101_CONFIG_REGISTER));
  Serial.print("PKTCTRL0: Data whitening / Packet format / CRC Check / Packet length - ");
  Serial.println(cc1101.readReg(0x08, CC1101_CONFIG_REGISTER));
  Serial.print("MDMCFG4: Channel BW - ");
  Serial.println(cc1101.readReg(0x10, CC1101_CONFIG_REGISTER));
  Serial.print("MDMCFG3: Data Rate (Baud) - ");
  Serial.println(cc1101.readReg(0x11, CC1101_CONFIG_REGISTER));
  Serial.print("MDMCFG2: Modulation / Manchester / Sync Mode - ");
  Serial.println(cc1101.readReg(0x12, CC1101_CONFIG_REGISTER));
  Serial.println("device initialized");
}

// SEND DATA TO RF1101SE
void send_data() {

  Serial.println("sending data");

  CCPACKET data;

  data.length = 5;

  // Alphanumerics get converted to hexidecimal. So if you asign a byte as 15 (DEC) you'll actually transmit 0F (HEX).
  // So you can either assign the alphanumeric or the direct hex representation.
  
  data.data[0] = 0x61; // 'a'
  data.data[1] = 'p'; // 0x70
  data.data[2] = 0x70; // 'p'
  data.data[3] = 'l'; // 0x6C
  data.data[4] = 'e'; // 0x65

  // Handy trick to invert bits in python
  // hex(~0b1111101011111011111111001111110111111110 & 0xFFFFFFFFFF)

  if (cc1101.sendData(data)) {
    Serial.println(" sent ok :)");
  } else {
    Serial.println("sent failed :(");
  }
}

void ReadLQI()
{
  byte lqi = 0;
  byte value = 0;
  lqi = (cc1101.readReg(CC1101_LQI, CC1101_STATUS_REGISTER));
  value = 0x3F - (lqi & 0x3F);
  Serial.print("CC1101_LQI ");
  Serial.println(value);
}

void ReadRSSI()
{
  byte rssi = 0;
  byte value = 0;

  rssi = (cc1101.readReg(CC1101_RSSI, CC1101_STATUS_REGISTER));

  if (rssi >= 128)
  {
    value = 255 - rssi;
    value /= 2;
    value += 74;
  }
  else
  {
    value = rssi / 2;
    value += 74;
  }
  Serial.print("CC1101_RSSI ");
  Serial.println(value);
}

void receive_data() {
  Serial.println("receiving data");
  // Disable wireless reception interrupt
  detachInterrupt(digitalPinToInterrupt(interruptPin));

  ReadRSSI();
  ReadLQI();
  // clear the flag
  packetAvailable = false;

  CCPACKET packet;

  if (cc1101.receiveData(&packet) > 0) {
    if (packet.crc_ok && packet.length > 1) {
      Serial.print("Packet length is ");
      Serial.print(packet.length);
      Serial.print("Packet data: ");
      for (int i = 0; i < packet.length; ++i) {
        Serial.print(packet.data[i], HEX);
        Serial.print(" ");
      }
    }
  }
  attachInterrupt(digitalPinToInterrupt(interruptPin), cc1101signalsInterrupt, FALLING);
}

void loop()
{
  //send_data();
  delay(1000);
  if (packetAvailable) {
    receive_data();
  }
  delay(1000);
}


// Set the PATABLE[1] with signal strength of a binary '1'. In OOK mode it will use PATABLE[0] for '0' strength.
// Table 30 on page 50 shows the optimal power setting per frequency. 0xC0 is highest, 0x60 is default.
void set_patable()
{
  byte PA_TABLE[] = {0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  cc1101.writeBurstReg(0x3E, PA_TABLE, 8);
}
