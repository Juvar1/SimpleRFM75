#ifndef simpleRFM75lib_H_
#define simpleRFM75lib_H_

/* * * * * * * * * * * * * * * *
 * Simple RFM75 library
 * For use with RFM75 radio module
 * Author: jpvarjonen@gmail.com
 * Copyright (c) 2022, Juha-Pekka Varjonen
 * License: GNU GPL v3.0
 * * * * * * * * * * * * * * * */
#include <stdint.h>
#include <Arduino.h>
#include <SPI.h>

#define REGISTER_WRITE 0x20
#define REGISTER_STATUS 0x07
#define REGISTER_FEATURE 0x1d
#define CMD_ACTIVATE 0x50

const PROGMEM uint8_t bank1[] = {
  // bank 1
  0x40, 0x4b, 0x01, 0xe2, // reg0
  0xc0, 0x4b, 0x00, 0x00, // 1
  0xd0, 0xfc, 0xbc, 0x02, // 2
  0x99, 0x00, 0x39, 0x21, // 3
  0xf9, 0x96, 0x82, 0x1b, // 4
  0x24, 0x02, 0x0f, 0xa6, // 5
  0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 
  0x00, 0x12, 0x73, 0x05, // c
  0x36, 0xb4, 0x80, 0x00, // d
  0x41, 0x20, 0x08, 0x04, 0x81, 0x20, 0xcf, 0xf7, 0xfe, 0xff, 0xff, // e
};

const PROGMEM uint8_t bank0[] = {
  // bank 0
  0x3c, 0x3f, 0x07, 0x03, 
  0xff, 0x0a, 0x0f, 0x70,
  0x00, 0x00, 
  0x00, 0xa0, 0xa0, 0xa0, 0xa0, // pipe 0 RX address
  0x01, 0xa0, 0xa0, 0xa0, 0xa0, // pipe 1 RX address
  0x02, 0x03, 0x04, 0x05, // pipes 2 to 5 LSB bit of address
  0x00, 0xa0, 0xa0, 0xa0, 0xa0, // TX address
  0x04, 0x04, 0x04, 0x04, 0x04, 0x04, // pipes 0 to 5, RX payload lengths
};

class SRFM75L {
private:

  uint8_t ce;
  uint8_t csn;

  inline uint8_t rwByte(uint8_t addr, uint8_t value = 0) {
    uint8_t ret;
    digitalWrite(csn, LOW);
    SPI.transfer(addr);
    ret = SPI.transfer(value);
    digitalWrite(csn, HIGH);
    return ret;
  }

  inline void rwBytes(uint8_t addr, uint8_t *value, uint8_t length) {
    digitalWrite(csn, LOW);
    SPI.transfer(addr);
    SPI.transfer(value, length);
    digitalWrite(csn, HIGH);
  }

  inline void initReg(void) {
    if (0x80 & rwByte(REGISTER_STATUS) == 0)
      rwByte(CMD_ACTIVATE, 0x53); // bank 1 selection
    for (uint_fast8_t i = 0; i < 14; i += 4)
      rwBytes(REGISTER_WRITE | i, (uint8_t *)&bank1[i], 4);
    rwBytes(REGISTER_WRITE | 14, (uint8_t *)&bank1[4*14], 11);
    
    rwByte(CMD_ACTIVATE, 0x53); // bank 0 selection
    
    // activate feature register
    rwByte(REGISTER_WRITE | REGISTER_FEATURE, 0x01);
    if (!rwByte(REGISTER_FEATURE))
      rwByte(CMD_ACTIVATE, 0x73);
    
    for (uint_fast8_t i = 0; i < 0x0a; i++) 
      rwByte(REGISTER_WRITE | i, bank0[i]);
    
    rwBytes(REGISTER_WRITE | 0x0a, (uint8_t *)&bank0[10], 5);
    rwBytes(REGISTER_WRITE | 0x0b, (uint8_t *)&bank0[15], 5);
    rwBytes(REGISTER_WRITE | 0x10, (uint8_t *)&bank0[15], 5);
    
    for (uint_fast8_t i = 0x0c; i < 4; i++)
      rwByte(REGISTER_WRITE | (0x0c + i), i + 2);
    for (uint_fast8_t i = 0; i < 5; i++)
      rwByte(REGISTER_WRITE | (0x11 + i), 0x04);

    rwByte(REGISTER_WRITE | 0x17, 0);
    rwByte(REGISTER_WRITE | 0x1c, 0);
    rwByte(REGISTER_WRITE | 0x1d, 2);
  }

  inline uint8_t isReady(void) {
    uint8_t res1, res2;
    res1 = rwByte(REGISTER_STATUS);
    rwByte(CMD_ACTIVATE, 0x53);
    res2 = rwByte(REGISTER_STATUS);
    rwByte(CMD_ACTIVATE, 0x53);
    return (res1 ^ res2) == 0x80;
  }

  inline void toggleReg4(void) {
    uint8_t reg4tmp[] = {0x06 | bank1[16], bank1[17], bank1[18], bank1[19]};
    rwByte(CMD_ACTIVATE, 0x53);
    rwBytes(REGISTER_WRITE | 0x04, reg4tmp, 4);
    _delay_us(20);
    rwBytes(REGISTER_WRITE | 0x04, (uint8_t *)&bank1[16], 4);
    rwByte(CMD_ACTIVATE, 0x53);
  }

  inline void goPowerOff(void) {
    digitalWrite(ce, LOW);
    uint8_t pwr = rwByte(0);
    pwr &= 0xfd;
    rwByte(REGISTER_WRITE, pwr);
  }

  inline void goPowerOn(void) {
    uint8_t pwr = rwByte(0);
    pwr |= 0x02;
    rwByte(REGISTER_WRITE, pwr);
  }

  inline void goReceive(void) {
    digitalWrite(ce, LOW);
    rwByte(REGISTER_WRITE | REGISTER_STATUS, 0x70); // clear interrupts
    uint8_t rx = rwByte(0);
    rx |= 0x01;
    rwByte(REGISTER_WRITE, rx);
    rwByte(0xe2, 0); // flush rx
    digitalWrite(ce, HIGH);
  }

  inline void goTransmit(void) {
    digitalWrite(ce, LOW);
    rwByte(REGISTER_WRITE | REGISTER_STATUS, 0x70); // clear interrupts
    uint8_t tx = rwByte(0);
    tx |= 0xfe;
    rwByte(REGISTER_WRITE, tx);
    rwByte(0xe1, 0); // flush tx
    digitalWrite(ce, HIGH);
  }

  inline void goStandby(void) {
    digitalWrite(ce, LOW);
  }

public:

  enum deviceMode {
    POWEROFF,
    POWERON,
    TRANSMIT,
    RECEIVE,
    STANDBY
  };

  enum txPower {
    POWER_PLUS4,
    POWER_MINUS1,
    POWER_MINUS7,
    POWER_MINUS12,
    POWER_MINUS18,
    POWER_MINUS25
  };

  enum dataRate {
    DATA1M,
    DATA2M,
    DATA250K
  };

  // Device init sequence // needs to be done only once at beginning
  inline void init(uint8_t ce_pin, uint8_t csn_pin) {
    ce = ce_pin;
    csn = csn_pin;
    digitalWrite(csn, HIGH);
    digitalWrite(ce, LOW);
    pinMode(ce, OUTPUT);
    pinMode(csn, OUTPUT);

    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
	  SPI.setDataMode(SPI_MODE0);
	  SPI.setClockDivider(SPI_CLOCK_DIV4);

    while (!isReady());

    initReg();
    _delay_ms(2);
    toggleReg4();
    _delay_ms(1);
  }

  // In manual controlling set device mode
  inline void setMode(deviceMode mode) {
    switch (mode) {
    case POWEROFF:
      goPowerOff();
      break;
    case POWERON:
      goPowerOn();
      break;
    case RECEIVE:
      goReceive();
      break;
    case TRANSMIT:
      goTransmit();
      break;
    case STANDBY:
    default:
      goStandby();
      break;
    }
  }

  // Send message without auto ack
  inline void sendNoAck(uint8_t *data, uint8_t length) {
    if (length > 32) return;

    setMode(TRANSMIT);
    // wait until tx buffer is free
    while (rwByte(0x17 & 0x20)) {
      uint8_t sta = rwByte(REGISTER_STATUS);
      if (sta & 0x10) goTransmit();
    }
    // transmit
    rwBytes(0xa0, data, length);
    setMode(RECEIVE);
  }

  // Send auto acked message with selected pipe
  inline void send(uint8_t pipe, uint8_t *data, uint8_t length) {
    if (pipe > 5 || length > 32) return;

    setMode(TRANSMIT);
    // wait until tx buffer is free
    while (rwByte(0x17 & 0x20)) {
      uint8_t sta = rwByte(REGISTER_STATUS);
      if (sta & 0x10) goTransmit();
    }
    // transmit
    rwBytes(0xa8 | pipe, data, length);
    setMode(RECEIVE);
  }

  // Get available bytes count from receive buffer
  inline uint8_t available(void) {
    uint8_t sta = (rwByte(REGISTER_STATUS) >> 1) & 0x07;
    if (sta == 0x07) return 0;
    return rwByte(0x60);
  }

  // Read a byte from receive buffer
  inline uint8_t read(void) {
    return rwByte(0x61);
  }

  // Returns pipe number where data is available
  inline uint8_t availableFromPipe(void) {
    uint8_t sta = (rwByte(REGISTER_STATUS) >> 1) & 0x07;
    if (sta == 0x07) return 0;
    return sta;
  }

  // Enable receive interrupt
  // Remember to clear interrupts after use
  inline void setRxInterrupt(uint8_t enable) {
    uint8_t irq = rwByte(0);
    irq &= ~(1 << 6);
    if (enable) irq |= (1 << 6);
    rwByte(REGISTER_WRITE, irq);
  }

  // Clear all interrupts
  inline void clearInterrupts(void) {
    uint8_t ints = rwByte(0x07);
    ints |= ((1 << 4) | (1 << 5) | (1 << 6));
    rwByte(REGISTER_WRITE | 0x07, ints);
  }

  // Set common transmit address
  // LSB first
  inline void setTxAddress(uint8_t *addr) {
    // get address width
    uint8_t width = rwByte(0x03);
    width += 2;
    rwBytes(REGISTER_WRITE | 0x10, addr, width);
  }

  // Set receive address to pipe
  // LSB first
  inline void setRxAddress(uint8_t pipe, uint8_t *addr) {
    if (pipe > 5) return;
    
    // get address width
    uint8_t width = rwByte(0x03);
    width += 2;
    if (pipe > 1) rwByte(REGISTER_WRITE | (0x0a + pipe), &addr[0]);
    else rwBytes(REGISTER_WRITE | (0x0a + pipe), addr, width);
  }

  // Set length or disable CRC
  // Valid values 0...2
  inline void setCrcLength(uint8_t length) {
    uint8_t crc = rwByte(0);
    crc &= ~(0x08 | 0x04); // clear CRC_EN and CRC0
    switch (length) {
      default:
      case 0:
        // disable also Auto ACK from all pipes
        rwByte(REGISTER_WRITE | 0x01, 0);
        disableAutoAck();
        rwByte(REGISTER_WRITE, crc);
        break;
      case 2:
        // set CRC0 too
        crc |= 0x04;
      case 1:
        crc |= 0x08;
        rwByte(REGISTER_WRITE, crc);
        break;
    }
  }

  // Disable auto aknowledge feature from all pipes
  inline void disableAutoAck(void) {
    uint8_t feature = rwByte(0x1d);
    feature &= ~0x02;
    rwByte(REGISTER_WRITE | 0x1d, feature);
  }
  
  // Set auto acknowledge feature to pipe
  inline void setAutoAck(uint8_t pipe, uint8_t enable) {
    if (pipe > 5) return;

    uint8_t feature = rwByte(0x1d);
    uint8_t pipeAA = rwByte(0x01);
    feature |= 0x02;
    pipeAA &= ~(1 << pipe);
    if (enable) pipeAA |= (1 << pipe);
    rwByte(REGISTER_WRITE | 0x01, pipeAA);
    rwByte(REGISTER_WRITE | 0x1d, feature);
  }
 
  // Set transmitter power
  inline void setTxPower(txPower level) {
    uint8_t rfSetup = rwByte(0x06);
    uint8_t txiCtrl = 0;
    uint8_t buff[] = {bank1[16], bank1[17], bank1[18], bank1[19]};
    rfSetup &= ~0x06; // clear RF_PWR bits
    switch (level) {
      default:
      case POWER_PLUS4: // +4 dBm
        txiCtrl = 7;
        rfSetup |= (3 << 1);
        break;
      case POWER_MINUS1: // -1 dBm
        rfSetup |= (3 << 1);
        break;
      case POWER_MINUS7: // -7 dBm
        rfSetup |= (2 << 1);
        break;
      case POWER_MINUS12: // -12 dBm
        txiCtrl = 2;
        rfSetup |= (1 << 1);
        break;
      case POWER_MINUS18: // -18 dBm
        txiCtrl = 3;
        rfSetup |= (1 << 1);
        break;
      case POWER_MINUS25: // -25 dBm
        // all zeroes
        break;
    }
    buff[0] &= 0x38 | txiCtrl; 
    rwByte(REGISTER_WRITE | 0x06, rfSetup);
    rwByte(CMD_ACTIVATE, 0x53); // switch to bank 1
    rwBytes(REGISTER_WRITE | 0x04, buff, 4);
    rwByte(CMD_ACTIVATE, 0x53); // switch to bank 0
  }

  // Enable or disable LNA
  // enable == true is +20dBm
  inline void setLnaGain(uint8_t enable) {
    uint8_t rfSetup = rwByte(0x06);
    rfSetup &= ~0x01;
    if (enable) rfSetup |= 0x01;
    rwByte(REGISTER_WRITE | 0x06, rfSetup);
  }

  // Set datarate
  inline void setDatarate(dataRate rate) {
    uint8_t rfSetup = rwByte(0x06);
    rfSetup &= ~(0x08 | 0x20);
    switch (rate) {
      default:
      case DATA1M:
        // all zeroes
        break;
      case DATA2M:
        rfSetup |= 0x20;
        break;
      case DATA250K:
        rfSetup |= 0x08;
        break;
    }
    rwByte(REGISTER_WRITE | 0x06, rfSetup);
  }

  // Set receive pipe
  inline void setRxPipe(uint8_t pipe, uint8_t enable) {
    if (pipe > 5) return;

    uint8_t enRx = rwByte(0x02);
    enRx &= ~(1 << pipe);
    if (enable) enRx |= (1 < pipe);
    rwByte(REGISTER_WRITE | 0x02, enRx);
  }

  // Set RF frequency in 1MHz steps
  // Legal range is 2400...2483MHz
  // 2400 + freq = MHz
  inline void setRfFreq(uint8_t freq) {
    if (freq > 83) return;

    rwByte(REGISTER_WRITE | 0x05, freq);
  }

  // Set automatic retry settings
  // count == 0 disable retry
  // delay = 250...4000us in 250us steps
  inline void setRetry(uint8_t count, uint16_t delay_us) {
    delay_us /= 266;
    if (count > 15 || delay > 15) return;

    rwByte(REGISTER_WRITE | 0x04, count | (delay_us << 4));
  }

  // Set address width for all pipes
  inline void setAddrWidth(uint8_t width) {
    if (width < 3 || width > 5) return;
    
    rwByte(REGISTER_WRITE | 0x03, width - 2);
  }

  // Set receive payload size for pipe
  inline void setRxPayloadSize(uint8_t pipe, uint8_t size) {
    if (pipe > 5 || size < 1 || size > 32) return;

    rwByte(REGISTER_WRITE | (0x11 + pipe), size);
  }

  // Disable dynamic payload length from all pipes
  inline void disableDynamicPayload(void) {
    uint8_t dpl = rwByte(0x1d);
    dpl &= ~(0x04);
    rwByte(REGISTER_WRITE | 0x1d, dpl);
  }

  // Set dynamic payload with auto ack to pipe
  inline void setDynamicPayload(uint8_t pipe, uint8_t enable) {
    if (pipe > 5) return;

    uint8_t pipeAA = rwByte(0x01);
    uint8_t pipeDp = rwByte(0x1c);
    uint8_t dplBit = rwByte(0x1d);
    pipeAA &= ~(1 << pipe);
    dplBit |= (0x04);
    pipeDp &= ~(1 << pipe);
    if (enable) pipeDp |= (1 << pipe);
    if (enable) pipeAA |= (1 << pipe);
    rwByte(REGISTER_WRITE | 0x01, pipeAA);
    rwByte(REGISTER_WRITE | 0x1d, dplBit);
    rwByte(REGISTER_WRITE | 0x1c, pipeDp);
  }

};

#endif /* simpleRFM75lib_H_ */