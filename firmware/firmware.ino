// Copyright 2018 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <SPI.h>

// For pinPeripheral(), so we can change PINMUX
#include "wiring_private.h"

// For libxsvf, so we can program the CPLD
#include "src/libxsvf/libxsvf.h"

// This code is for an ATSAMD21E18A connected to an acorn_postbox CPLD
// Functions provided:
// - USB serial interface for all control
// - USB SVF+JTAG interface to program the CPLD
// - CPLD clock generation (2MHz)

// Pinout:

// Pin assignments from the Adafruit Circuit Playground Express (ATSAMD21G18A)
// are used here.  Be careful mapping these to actual pins, because the G18A and
// our E18A have different pinouts.

// Mapping: A(x) = D(x+14), e.g. A0 = D14 and A10 = D24

// CPLD JTAG port

// PA04 - D24 / A10
#define TDO_PIN 24
// PA06 - D16 / A2
#define TMS_PIN 16
// PA05 - D15 / A1
#define TCK_PIN 15
// PA07 - D17 / A3
#define TDI_PIN 17

// CPLD comms

// PA08 / SERCOM2.0 (SERCOM-ALT) - D35 - cpld_MOSI
#define CPLD_MOSI_PIN 35
// In slave mode, DI is MOSI
#define SPI_RX_PAD SERCOM_RX_PAD_0
// PA09 / SERCOM2.1 (SERCOM-ALT) - D23 - cpld_SCK
#define CPLD_SCK_PIN 23
// PA14 / SERCOM2.2 (just SERCOM, not ALT) - D5 - cpld_SS
#define CPLD_SS_PIN 5
// PA15 / SERCOM2.3 (just SERCOM, not ALT) - D7 - cpld_MISO
#define CPLD_MISO_PIN 7
// In slave mode, DO is MISO
#define SPI_TX_PAD SPI_PAD_3_SCK_1

// PA10 - D34 - 2MHz clock out
#define CLOCK_2MHZ_PIN 34
// PA11 - D22 - rx_ready
#define RX_READY_PIN 22
// PA16 (MCU pin 17) - tx_pending
// This is shown as pin 11 in the comment at the top of circuitplay/variant.cpp,
// but is pin 30 in g_APinDescription.
#define TX_PENDING_PIN 30
// PA17 (MCU pin 18) - want_tx
// Unfortunately this is also the LED pin, so the bootloader might conflict with the CPLD here
#define WANT_TX_PIN 13
// PA28 (MCU pin 27) - reset_in
#define TARGET_RESET_PIN 4
// PA02 (MCU pin ?) - transaction direction (0 = output from target, 1 = input to target)
#define LAST_TRANSACTION_WAS_INPUT_PIN 12

// Uncomment this to show every byte sent and received over SPI
// #define SHOW_ALL_SPI_TRANSFERS


// libxsvf (xsvftool-arduino) entry point
extern void arduino_play_svf(int tms_pin, int tdi_pin, int tdo_pin, int tck_pin, int trst_pin);

// SPI on SERCOM2 to talk to the CPLD at 24MHz
SPIClass cpld_spi(&sercom2, CPLD_MISO_PIN, CPLD_SCK_PIN, CPLD_MOSI_PIN, SPI_TX_PAD, SPI_RX_PAD);

enum {
  WAITING_FOR_READY_BYTE = 0,
  TARGET_WAITING_FOR_COMMAND,
  PROXYING_POST_OUTPUT,
  TARGET_EXECUTING_COMMAND
} state;

enum {
  ARCHIMEDES = 0,
  RISC_PC
} machine = ARCHIMEDES;

void HardFault_Handler(void) { __BKPT(3); }
void NMI_Handler      (void) { __BKPT(3); }
void SVC_Handler      (void) { __BKPT(3); }
void PendSV_Handler   (void) { __BKPT(3); }
void PM_Handler       (void) { __BKPT(3); }
void SYSCTRL_Handler  (void) { __BKPT(3); }
void WDT_Handler      (void) { __BKPT(3); }
void RTC_Handler      (void) { __BKPT(3); }
void EIC_Handler      (void) { __BKPT(3); }
void NVMCTRL_Handler  (void) { __BKPT(3); }
void DMAC_Handler     (void) { __BKPT(3); }
// void USB_Handler      (void) { __BKPT(3); }
void EVSYS_Handler    (void) { __BKPT(3); }
void SERCOM0_Handler  (void) { __BKPT(3); }
void SERCOM1_Handler  (void) { __BKPT(3); }
void SERCOM2_Handler  (void) { __BKPT(3); }
void SERCOM3_Handler  (void) { __BKPT(3); }
// void SERCOM4_Handler  (void) { __BKPT(3); }
void SERCOM5_Handler  (void) { __BKPT(3); }
void TCC0_Handler     (void) { __BKPT(3); }
void TCC1_Handler     (void) { __BKPT(3); }
void TCC2_Handler     (void) { __BKPT(3); }
void TC3_Handler      (void) { __BKPT(3); }
void TC4_Handler      (void) { __BKPT(3); }
void TC5_Handler      (void) { __BKPT(3); }
void TC6_Handler      (void) { __BKPT(3); }
void TC7_Handler      (void) { __BKPT(3); }
void ADC_Handler      (void) { __BKPT(3); }
void AC_Handler       (void) { __BKPT(3); }
void DAC_Handler      (void) { __BKPT(3); }
void PTC_Handler      (void) { __BKPT(3); }
void I2S_Handler      (void) { __BKPT(3); }

// Set MOSI/SCK/MISO/SS pins up for SPI comms with CPLD.
void select_spi() {
  cpld_spi.begin();
  sercom2.disableSPI();  // disable SERCOM so we can write registers.

  // Now reconfigure in slave mode.
  SERCOM2->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_MODE_SPI_SLAVE |
                           SERCOM_SPI_CTRLA_DOPO(SPI_TX_PAD) |
                           SERCOM_SPI_CTRLA_DIPO(SPI_RX_PAD) |
                           MSB_FIRST << SERCOM_SPI_CTRLA_DORD_Pos;

  SERCOM2->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_RXEN |
                           SERCOM_SPI_CTRLB_PLOADEN |  // Allow slave preload
                           SERCOM_SPI_CTRLB_CHSIZE(SPI_CHAR_SIZE_8_BITS);


  // cpld_spi.begin() doesn't get these right, as there are two SERCOM options
  // for these pins.
  pinPeripheral(CPLD_MOSI_PIN, PIO_SERCOM_ALT);
  pinPeripheral(CPLD_SCK_PIN, PIO_SERCOM_ALT);
  pinPeripheral(CPLD_MISO_PIN, PIO_SERCOM);  // just PIO_SERCOM as PA11 is now rx_ready
  pinPeripheral(CPLD_SS_PIN, PIO_SERCOM);    // just PIO_SERCOM as PA10 is now 2MHz

  // And start it up again!
  sercom2.enableSPI();

  // But disable interrupts
  SERCOM2->SPI.INTENCLR.reg = SERCOM_SPI_INTENCLR_ERROR |
                              SERCOM_SPI_INTENCLR_SSL |
                              SERCOM_SPI_INTENCLR_RXC |
                              SERCOM_SPI_INTENCLR_TXC |
                              SERCOM_SPI_INTENCLR_DRE;

  // Don't call cpld_spi.beginTransaction, as that will switch back to master mode.
}

// Send/receive a byte over SPI, optionally dumping details to the serial port.
uint8_t spi_transfer(uint8_t b) {
  uint8_t r = cpld_spi.transfer(b);
#ifdef SHOW_ALL_SPI_TRANSFERS
  Serial.print("[");
  Serial.print(b, HEX);
  Serial.print(" -> ");
  Serial.print(r, HEX);
  Serial.print("]");
#endif
  return r;
}

void reset() {
  // clear out any unread input
  while (Serial.available() && !Serial.dtr()) {
    Serial.read();
  }
  Serial.println("OK");
}

// returns true if we're disconnected and have been reset
bool check_disconnect() {
  if (Serial.dtr()) {
    return false; // all is well
  }

  // remote has disconnected -- reset everything / keep it reset
  reset();
  return true;
}

// System startup
void setup() {

  // Enable SPI port on SERCOM2
  select_spi();

  // Force-reset all other SERCOMs, just in case one of them is the cause of our spurious hard faults
  sercom0.resetUART();
  sercom1.resetUART();
  sercom3.resetUART();
  sercom4.resetUART();
  sercom5.resetUART();

  // Set up our GPIOs
  pinMode(RX_READY_PIN, OUTPUT);
  digitalWrite(RX_READY_PIN, HIGH);
  digitalWrite(RX_READY_PIN, LOW);

  pinMode(TX_PENDING_PIN, OUTPUT);
  digitalWrite(TX_PENDING_PIN, LOW);

  pinMode(WANT_TX_PIN, INPUT);

  pinMode(TARGET_RESET_PIN, OUTPUT);
  digitalWrite(TARGET_RESET_PIN, LOW);

  // Output 2MHz clock for CPLD
  // DIV=2 gives 24MHz, so DIV=24 is probably what we want
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(4) | GCLK_GENDIV_DIV(24);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(4) |
                      GCLK_GENCTRL_SRC_DFLL48M |  // GCK4 sourced by DFLL48M ()
                      GCLK_GENCTRL_OE |
                      GCLK_GENCTRL_IDC |
                      GCLK_GENCTRL_GENEN;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  // enable GCLK_IO[4] on PA10
  pinPeripheral(CLOCK_2MHZ_PIN, PIO_AC_CLK);

  // Set pin directions for CPLD JTAG.
  pinMode(TDO_PIN, INPUT);
  pinMode(TDI_PIN, OUTPUT);
  digitalWrite(TDI_PIN, HIGH);
  pinMode(TMS_PIN, OUTPUT);
  digitalWrite(TMS_PIN, HIGH);
  pinMode(TCK_PIN, OUTPUT);
  digitalWrite(TCK_PIN, LOW);

  // Set up USB serial port
  Serial.begin(9600);
}

// // Read a byte from the USB serial port
// uint8_t serial_get_uint8() {
//   while (!Serial.available());
//   return (uint8_t)Serial.read();
// }

// // Read a big-endian uint32 from the USB serial port
// uint32_t serial_get_uint32() {
//   uint32_t v = (uint32_t)serial_get_uint8() << 24L;
//   v |= (uint32_t)serial_get_uint8() << 16L;
//   v |= (uint32_t)serial_get_uint8() << 8L;
//   v |= (uint32_t)serial_get_uint8();
//   return v;
// }

// // Write a big-endian uint32 to the USB serial port
// void serial_put_uint32(uint32_t v) {
//   Serial.write((uint8_t)((v & 0xFF000000) >> 24));
//   Serial.write((uint8_t)((v & 0xFF0000) >> 16));
//   Serial.write((uint8_t)((v & 0xFF00) >> 8));
//   Serial.write((uint8_t)(v & 0xFF));
// }

// // Read a little-endian uint32 from the USB serial port
// uint32_t serial_get_uint32() {
//   uint32_t v = (uint32_t)serial_get_uint8();
//   v |= (uint32_t)serial_get_uint8() << 8L;
//   v |= (uint32_t)serial_get_uint8() << 16L;
//   v |= (uint32_t)serial_get_uint8() << 24L;
//   return v;
// }

// // Write a little-endian uint32 to the USB serial port
// void serial_put_uint32(uint32_t v) {
//   Serial.write((uint8_t)(v & 0xFF));
//   Serial.write((uint8_t)((v & 0xFF00) >> 8));
//   Serial.write((uint8_t)((v & 0xFF0000) >> 16));
//   Serial.write((uint8_t)((v & 0xFF000000) >> 24));
// }

bool just_sent_a_byte = false;
uint8_t last_byte_sent = 0;
bool have_buffered_rx_byte = false;
uint8_t buffered_rx_byte = 0;
bool last_transaction_was_input = false;

bool target_available() {
  // Do we have a byte already?
  if (have_buffered_rx_byte) return true;

  // If nothing is waiting for us, pulse RX_READY, to make sure the CPLD knows
  // we want data.
  if (!SERCOM2->SPI.INTFLAG.bit.RXC) {
    digitalWrite(RX_READY_PIN, HIGH);
    digitalWrite(RX_READY_PIN, LOW);
  }

  // Handle any incoming bytes.
  if (!SERCOM2->SPI.INTFLAG.bit.RXC) {
    return false;
  }

  // Read byte from SPI.
  if (SERCOM2->SPI.STATUS.bit.BUFOVF) {
    Serial.println("<input overflow!>");
    SERCOM2->SPI.STATUS.bit.BUFOVF = 1;  // Clear BUFOVF
  }
  uint8_t b = SERCOM2->SPI.DATA.bit.DATA;
  last_transaction_was_input = !!digitalRead(LAST_TRANSACTION_WAS_INPUT_PIN);

  // Indicate to CPLD that we have room for another one.
  digitalWrite(RX_READY_PIN, HIGH);
  digitalWrite(RX_READY_PIN, LOW);

  // And buffer our new byte
  have_buffered_rx_byte = true;
  buffered_rx_byte = b;
  return true;
}

uint8_t target_readbyte() {
  long start_ts = millis();
  while (!target_available()) {
    if (check_disconnect()) {
      return 0;
    }
    if (millis() - start_ts > 5000) {
      Serial.println("target_readbyte timeout");
      return 0;
    }
  }
  have_buffered_rx_byte = false;
  return buffered_rx_byte;
}

uint32_t target_input_checksum = 0;

uint32_t target_readword() {
  uint32_t w = (uint32_t)target_readbyte();
  w = (w << 8) | (uint32_t)target_readbyte();
  w = (w << 8) | (uint32_t)target_readbyte();
  w = (w << 8) | (uint32_t)target_readbyte();
  target_input_checksum += w;
  return w;
}

void target_reset_input_checksum() {
  target_input_checksum = 0;
}

void target_verify_input_checksum() {
  uint32_t cs = target_readword();
  Serial.print("read cs ");
  Serial.print(cs, HEX);
  Serial.print(" and sum is ");
  Serial.println(target_input_checksum);
  if (target_input_checksum != 0) {
    Serial.println("CHECKSUM FAIL");
  }
}

void target_sendbyte_internal(uint8_t b) {
  SERCOM2->SPI.DATA.bit.DATA = b;
  digitalWrite(TX_PENDING_PIN, HIGH);
  digitalWrite(TX_PENDING_PIN, LOW);
}

int target_sendbyte(uint8_t b) {
  // Buffer should always be empty before a sendbyte call
  if (target_available()) {
    Serial.print("(");
    Serial.print(target_readbyte(), HEX);
    Serial.print("!)");
  }

  Serial.print(">");
  Serial.print(b, HEX);

  // long start_ts = millis();
  // do {
  //   // Pick up any byte in the buffer, because we'll get another one from the
  //   // tx transaction.
  //   if (target_available() && SERCOM2->SPI.INTFLAG.bit.RXC) {
  //     uint8_t b = target_readbyte();
  //     // Serial.print("ERROR: target_sendbyte() dropped byte 0x");
  //     Serial.print(b, HEX);
  //     Serial.print("!");
  //     // Serial.println(" waiting in the buffer.");
  //   }
  //   if (millis() - start_ts > 30) {
  //     Serial.println("Timeout waiting for ")
  //   }
  //   // TODO timeout here
  // } while (just_sent_a_byte ||
  //   !digitalRead(WANT_TX_PIN));

  long start_ts = millis();
  while (!digitalRead(WANT_TX_PIN)) {
    if (millis() - start_ts > 500) {
      Serial.println("Send timeout");
      return -1;
    }
  }

  target_sendbyte_internal(b);
  // just_sent_a_byte = true;  // DEBUG: setting this may be causing hangs
  // last_byte_sent = b;
  // if (target_available()) {
  //   Serial.println("Send ack too soon?");
  // }

  // Now verify that the byte was sent
  start_ts = millis();
  while (!target_available()) {
    if (millis() - start_ts > 500) {
      Serial.println("Send ack timeout");
      return -1;
    }
  }

  // Read ack byte (should be zero)
  if (!last_transaction_was_input) {
    Serial.println("Serial mismatch: sendbyte saw output transaction");
    return -1;
  }

  uint8_t r = target_readbyte();
  if (r != 0) {
    Serial.print("Serial bad ack ");
    Serial.println(r, HEX);
    return -2;
  }

  return 0;
}

uint32_t target_checksum = 0;

void target_reset_checksum() {
  target_checksum = 0;
}

int target_sendword(uint32_t w) {
  int r = target_sendbyte((uint8_t)((w & (uint32_t)0xff000000) >> 24));
  if (r < 0) return r;
  r = target_sendbyte((uint8_t)((w & (uint32_t)0xff0000) >> 16));
  if (r < 0) return r;
  r = target_sendbyte((uint8_t)((w & (uint32_t)0xff00) >> 8));
  if (r < 0) return r;
  r = target_sendbyte((uint8_t)((w & (uint32_t)0xff)));
  if (r < 0) return r;
  target_checksum += w;
  return 0;
}

int target_send_checksum() {
  // Send two's complement of current checksum, to make the total sum to zero
  uint32_t fixup = ~target_checksum + 1;
  return target_sendword(fixup);
}

int target_wait_ready() {
  // wait for 0x90 byte
  while (1) {
    long start_ts = millis();
    bool want_tx = false;
    while (!target_available()) {
      if (check_disconnect()) return -1;
      if (digitalRead(WANT_TX_PIN) && !want_tx) {
        Serial.println("Target wants data -- maybe ready?");
        want_tx = true;
      }
      if (millis() - start_ts > 500) {
        Serial.println("Timeout in wait_ready");
        return -2;
      }
    }
    uint8_t b = target_readbyte();
    Serial.print("<");
    Serial.print(b, HEX);
    Serial.print(">");
    if (b == 0x90) return 0;
  }
}

uint8_t last_command = 0;
int target_start_command(uint8_t command) {
  last_command = command;
  int r = target_sendbyte(command);
  if (r < 0) return r;
  target_reset_checksum();
  return 0;
}

int target_finish_command() {
  int r = target_send_checksum();
  if (r < 0) return r;
  Serial.print("Response: ");
  uint8_t b = target_readbyte();
  Serial.print(b, HEX);
  if (b == 0xff) {
    Serial.println(" - Checksum error");
    return -1;
  }
  if (b != last_command) {
    Serial.println(" - Incorrect command byte");
    return -2;
  }
  Serial.println(" - OK");
  return 0;
}

#define CB(f) if ((f) < 0) break
#define CR(f) if ((f) < 0) return -1

  /* Commands:

    0 = LCD driving mode
    0x08 - 0x0f = write data
      byte: 0x08 | (new_data ? 0x04 : 0) | (increment_address ? 0x02 : 0) | (byte_mode ? 0x01 : 0)
        Most useful: 0x0e = write multiple words (CMD_WRITE | CMD_WRITE_NEW_DATA | CMD_WRITE_INCREMENT_ADDR | CMD_WRITE_WORD)
  */
#define CMD_WRITE 0x08
#define CMD_WRITE_NEW_DATA 0x04
#define CMD_WRITE_SAME_DATA 0
#define CMD_WRITE_INCREMENT_ADDR 0x02
#define CMD_WRITE_SAME_ADDR 0
#define CMD_WRITE_BYTE 0x01
#define CMD_WRITE_WORD 0
  /*
      word: count of bytes or words to write
      word: initial address
      words: data to write (just a single word if !new_data)
      word: checksum fixup so the entire packet aside from the command byte sums to zero
    0x10 - 0x17 = read data
      byte: 0x10 | (report_every_value ? 0x04 : 0) | (increment_address ? 0x02 : 0) | (byte_mode ? 0x01 : 0)
        Most useful: 0x16 = read multiple words (CMD_READ | CMD_READ_REPORT_ALL | CMD_READ_INCREMENT_ADDR | CMD_READ_WORD)
  */
#define CMD_READ 0x10
#define CMD_READ_REPORT_ALL 0x04
#define CMD_READ_REPORT_LAST 0
#define CMD_READ_INCREMENT_ADDR 0x02
#define CMD_READ_SAME_ADDR 0
#define CMD_READ_BYTE 0x01
#define CMD_READ_WORD 0
  /*
      word: count of bytes of words to write
      word: initial address
      word: checksum fixup
      response: 0xff for checksum failure, or copy of command byte
      words: data
      word: checksum fixup
    0x18 = execute
      byte: 0x18
      word: exec address
      word: checksum fixup
    0x20 - 0x27 = perform bus cycles
      byte: command
        0x20 - word read: ldr dummy, [r9]; ldr dummy, [r10]
        0x21 - byte read: ldrb dummy, [r9]; ldr dummy, [r10]
        0x22 - write and read words: str r11, [r9]; str r12, [r10]; ldr dummy, [r9]; ldr dummy, [r10]
        0x23 - write and read bytes: strb r11, [r9]; strb r12, [r10]; ldrb dummy, [r9]; ldrb dummy, [r10]
        0x24 - read multiple x 2: ldmia r9, {r1, r2}; ldmia r10, {r1, r2}
        0x25 - read multiple, read byte, read word: ldmia r9, {r1, r2}; ldrb dummy, [r10]; ldr dummy, [r9]
        0x26 - write and read multiple: stmia r9, {r11, r12}; ldmia r9, {r1, r2}; stmia r10, {r11, r12}; ldmia r10, {r11, r12}
        0x27 - write multiple, write byte, write word, then read all: stmia r9, {r11, r12}; strb r11, [r10]; str r12, [r9];
          ldmia r9, {r1, r2}; ldrb r1, [r10]; ldr r1, [r9]
      word: operation count (r8)
      word: first address (r9)
      word: second address (r10)
      word: first data (r11)
      word: second data (r12
      word: checksum fixup
    0xff = self test

  */

int read_words_from_memory(uint32_t start_addr, uint32_t n_words) {
  CR(target_start_command(CMD_READ | CMD_READ_REPORT_ALL | CMD_READ_INCREMENT_ADDR | CMD_READ_WORD));
  CR(target_sendword(n_words));
  CR(target_sendword(start_addr));
  CR(target_finish_command());
  uint32_t start_addr_confirmation = target_readword();
  target_reset_input_checksum();
  Serial.print("End addr: ");
  Serial.println(start_addr_confirmation, HEX);
  for (uint32_t i = 0; i < n_words; ++i) {
    Serial.println(target_readword(), HEX);
  }
  target_verify_input_checksum();
  return 0;
}

int write_word_to_memory(uint32_t addr, uint32_t data) {
  CR(target_start_command(CMD_WRITE | CMD_WRITE_NEW_DATA | CMD_WRITE_INCREMENT_ADDR | CMD_WRITE_WORD));  // Write words
  CR(target_sendword(1));
  CR(target_sendword(addr));
  CR(target_sendword(data));
  CR(target_finish_command());
  uint32_t cs = target_readword();
  CR(target_wait_ready());
}

bool target_started = false;
bool last_want_tx = false;
bool have_half_char = false;
uint8_t char_so_far = 0;
uint8_t last_printed = 0;

void target_reset() {
    Serial.print("Resetting target... ");
    digitalWrite(TARGET_RESET_PIN, HIGH);
    delay(100);
    digitalWrite(TARGET_RESET_PIN, LOW);
    Serial.println("done.");
    state = WAITING_FOR_READY_BYTE;
    target_started = false;
}

void loop() {

  static uint8_t serial_active = 0;
  static unsigned long serial_active_when = 0;
  if (!Serial.dtr()) {
    if (serial_active) {
      serial_active = 0;
    }
  } else if (!serial_active) {
    // USB serial connection is active
    serial_active_when = millis();
    serial_active = 1;
  } else if (serial_active == 1 && millis() - serial_active_when > 10) {
    // Serial port should have settled by now
    Serial.print("MCU at ");
    Serial.print(SystemCoreClock);
    Serial.println(" MHz");
    serial_active = 2;

    target_reset();

    Serial.println("OK");
  } else if (serial_active == 2) {
    // Serial port is actually open.
  }

  if (target_available()) {
    uint8_t b = target_readbyte();

    // Not proxying POST output; data received between commands
    Serial.print("[");
    Serial.print(b, HEX);
    Serial.print("]");
    if (b == 0x90) {
      if (!target_started) {
        Serial.println("Target is alive!");
      }
      target_started = true;
      state = TARGET_WAITING_FOR_COMMAND;
    }
  }

  bool want_tx = digitalRead(WANT_TX_PIN) ? true : false;
  if (want_tx != last_want_tx) {
    last_want_tx = want_tx;
    if (want_tx && !target_started) {
      Serial.println("Target not yet started; ignoring tx request");
    }
  }

  if (Serial.available()) {
    int c = Serial.read();
    switch (c) {
      case 'C': {
        // program CPLD
        Serial.println("SEND SVF");
        arduino_play_svf(TMS_PIN, TDI_PIN, TDO_PIN, TCK_PIN, -1);
        Serial.println("SVF DONE");
        break;
      }
      case 's': {
        // Switch machine
        machine = (machine == ARCHIMEDES) ? RISC_PC : ARCHIMEDES;
        Serial.println("Machine set to ");
        Serial.println(machine == ARCHIMEDES ? "Archimedes" : "RISC PC");
        break;
      }
      case '!': {
        target_reset();
        break;
      }
      case 'p': {
        Serial.println("LCD mode - Monitoring POST output:");
        target_sendbyte(0);
        while (1) {
          // if (digitalRead(WANT_TX_PIN)) {
          //   // In case we get spurious pulses on want_tx, just fire and forget here
          //   // (and ignore all zeros on incoming data)
          //   target_sendbyte_internal(0);
          // }
          if (target_available()) {
            uint8_t b = target_readbyte();
            // Serial.print("[");
            // Serial.print(b, HEX);
            // Serial.print("]");
            switch (b & 0x0f) {
              case 0:
                have_half_char = false;
                switch (b) {
                  case 0:
                    if (last_printed != '\n' && last_printed != ' ') {
                      Serial.println();
                      last_printed = '\n';
                    }
                    break;
                  case 0x90:
                    Serial.println("ERROR: Got 0x90 during POST");
                    break;
                  case 0x30:
                  case 0x20:
                  case 0x80:
                    break;
                  default:
                    if (last_printed != ' ' && last_printed != '\n') {
                      Serial.print(" ");
                      last_printed = ' ';
                    }
                    break;
                }
                break;
              case 8:
                if (!have_half_char) {
                  char_so_far = b & 0xf0;
                  have_half_char = true;
                } else {
                  char_so_far |= ((b & 0xf0) >> 4);
                  Serial.print((char)char_so_far);
                  last_printed = char_so_far;
                  have_half_char = false;
                }
                break;
            }
          }
          if (Serial.available()) {
            Serial.println("Exiting POST monitor loop");
            break;
          }
        }
        break;
      }
      case 'R': {
        Serial.println("Reading ROM data");
        uint32_t rom_start = (machine == ARCHIMEDES) ? 0x3800000 : 0;
        read_words_from_memory(rom_start, 16);
        break;
      }
      case 'r': {
        Serial.println("Reading from RAM");
        uint32_t ram_start = (machine == ARCHIMEDES) ? 0x2000000 : 0x18000000;  // A7000 soldered RAM
        read_words_from_memory(ram_start, 16);
        read_words_from_memory(ram_start + 1024 * 1024, 16);
        break;
      }
      case 'W': {
        Serial.println("Writing block to RAM");
        uint32_t ram_start = (machine == ARCHIMEDES) ? 0x2000000 : 0x18000000;  // A7000 soldered RAM
        for (uint32_t bank = 0; bank < 2; ++bank, ram_start += 1048576L) {
          CB(target_start_command(CMD_WRITE | CMD_WRITE_NEW_DATA | CMD_WRITE_INCREMENT_ADDR | CMD_WRITE_WORD));  // Write words
          CB(target_sendword(16));
          CB(target_sendword(ram_start));
          for (int rep = 0; rep < 16; ++rep) {
            uint32_t ram_addr = ram_start + (rep << 2);
            uint32_t random_data = ((uint32_t)random(0x10000L) << 16) | (uint32_t)random(0x10000L);
            Serial.print("Write ");
            Serial.print(random_data, HEX);
            Serial.print(" at ");
            Serial.println(ram_addr, HEX);
            CB(target_sendword(random_data));
          }
          CB(target_finish_command());
          uint32_t cs = target_readword();
          CB(target_wait_ready());
        }
        break;
      }
      case 'w': {
        Serial.println("Writing to RAM");
        uint32_t ram_start = (machine == ARCHIMEDES) ? 0x2000000 : 0x18000000;  // A7000 soldered RAM
        for (int rep = 0; rep < 32; ++rep) {
          uint32_t ram_addr = ram_start + (random(16384) << 2);
          uint32_t random_data = ((uint32_t)random(0x10000L) << 16) | (uint32_t)random(0x10000L);
          Serial.print("Write ");
          Serial.print(random_data, HEX);
          Serial.print(" at ");
          Serial.println(ram_addr, HEX);
          CB(write_word_to_memory(ram_addr, random_data));

          Serial.println("Read back...");
          read_words_from_memory(ram_addr, 1);
        }
        break;
      }
      case 'M': {
        if (machine == ARCHIMEDES) {
          Serial.println("Setup MEMC");
          CB(write_word_to_memory(0x036E050C, 0));
        } else {
          Serial.println("Setup RAM in IOMD / A7000");
        }
        break;
      }
      case 'e': {
        Serial.println("Bus exercise - writing and reading aaaaaaaa and 55555555 to 200_0000 and 200_2000");
        CB(target_start_command(0x22));
        CB(target_sendword(0xFFFFFFFF));  // op count
        CB(target_sendword(0x02000000));  // addr 1
        CB(target_sendword(0x02002000));  // addr 2
        CB(target_sendword(0xAAAAAAAA));  // data 1
        CB(target_sendword(0x55555555));  // data 2
        CB(target_finish_command());
        // Serial.print("Response: ");
        // uint8_t r = target_readbyte();
        // Serial.print(r, HEX);
        break;
      }
      case 'I': {
        if (machine == ARCHIMEDES) {
          Serial.println("IOC/IOEB check");
          Serial.println("IRQMSKA");
          Serial.println("IRQMSKB");
          Serial.println("IOEB ID reg - should be xxxxxxx5");
          // 0    3    3    5    0    0    5    0
          // 0000 0011 0011 0101 0000 0000 0101 0000
          //        ^^ ^^^^  ^^^ ^^^        ^^^ ^^
          //              T TBBB            AAA AA
          read_words_from_memory(0x03350050, 1);  // IOEB ID; D3:0 should be 5
          Serial.println("Setting TIMER2 to output a 500 kHz signal on BAUD (IOC pin 27)");
          CB(write_word_to_memory(0x03200060, 1<<16));  // latch low
          CB(write_word_to_memory(0x03200064, 1<<16));  // latch high
          CB(write_word_to_memory(0x03200068, 0));  // go
          read_words_from_memory(0x03200060, 1);  // latch low
          read_words_from_memory(0x03200064, 1);  // latch high
        } else {
          Serial.println("IOMD TODO");
        }
        break;
      }
      // case 'x': {
      //   Serial.println("Execute");
      //   target_sendbyte(0x18);
      //   target_reset_checksum();
      //   CB(target_sendword(0x3801b10));  // movs pc, r14 in RISC OS 3.11
      //   CB(target_finish_command());
      //   Serial.print("Response: ");
      //   uint8_t r = target_readbyte();
      //   Serial.println(r, HEX);
      //   if (r == 0xff) {
      //     Serial.println("checksum error");
      //   } else {
      //     for (int i = 0; i < 20; ++i) {
      //       uint8_t b = target_readbyte();
      //       Serial.print(b, HEX);
      //       if ((i % 4) == 3) Serial.println(); else Serial.print(" ");
      //     }
      //   }
      //   break;
      // }

      // }
      // case 'Y': {
      //   Serial.println("Enter bank start (MB, 0-9), then bank size (1, 2, 4).");
      //   while (!Serial.available()) {
      //     if (check_disconnect()) break;
      //   }
      //   uint8_t bank_c = Serial.read();
      //   while (!Serial.available()) {
      //     if (check_disconnect()) break;
      //   }
      //   uint8_t size_c = Serial.read();
      //   (void)size_c;

      //   uint8_t bank = bank_c - '0';
      //   if (bank > 9) bank = 0;

      //   reset();
      //   break;
      // }
    }
    Serial.println("OK");
  }

}
