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

// For libxsvf, so we can program the FPGA
#include "src/libxsvf/libxsvf.h"

// This code is for an ATSAMD21E18A connected to an acorn_postbox FPGA
// Functions provided:
// - USB serial interface for all control
// - USB SVF+JTAG interface to program the FPGA
// - FPGA clock generation (2MHz)

// Pinout:

// Pin assignments from the Adafruit Circuit Playground Express (ATSAMD21G18A)
// are used here.  Be careful mapping these to actual pins, because the G18A and
// our E18A have different pinouts.

// Mapping: A(x) = D(x+14), e.g. A0 = D14 and A10 = D24

// FPGA JTAG port

// PA04 - D24 / A10
#define TDO_PIN 24
// PA06 - D16 / A2
#define TMS_PIN 16
// PA05 - D15 / A1
#define TCK_PIN 15
// PA07 - D17 / A3
#define TDI_PIN 17

// FPGA comms

// PA08 / SERCOM2.0 (SERCOM-ALT) - D35 - FPGA_MOSI
#define FPGA_MOSI_PIN 35
// In master mode, DO is MOSI
#define SPI_TX_PAD SPI_PAD_0_SCK_1
// PA09 / SERCOM2.1 (SERCOM-ALT) - D23 - FPGA_SCK
#define FPGA_SCK_PIN 23
// PA14 / SERCOM2.2 (just SERCOM, not ALT) - D5 - FPGA_SS
#define FPGA_SS_PIN 5
// PA15 / SERCOM2.3 (just SERCOM, not ALT) - D7 - FPGA_MISO
#define FPGA_MISO_PIN 7
// In master mode, DI is MISO
#define SPI_RX_PAD SERCOM_RX_PAD_3

// PA10 - D34 - 48MHz clock out (output)
#define CLOCK_48MHZ_PIN 34
// PA11 - D22 - target powered (input)
#define TARGET_POWER_PIN 22
// PA28 (MCU pin 27) - reset_in (output)
#define TARGET_RESET_PIN 4

// Uncomment this to show every byte sent and received over SPI
// #define SHOW_ALL_SPI_TRANSFERS

// PA22 (MCU pin 21) - LINK/ACT LED on when driven low
#define LINK_ACT_LED_PIN 36


// libxsvf (xsvftool-arduino) entry point
extern void arduino_play_svf(int tms_pin, int tdi_pin, int tdo_pin, int tck_pin, int trst_pin);

// SPI on SERCOM2 to talk to the FPGA at 8MHz
SPIClass fpga_spi(&sercom2, FPGA_MISO_PIN, FPGA_SCK_PIN, FPGA_MOSI_PIN, SPI_TX_PAD, SPI_RX_PAD);

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
// void DMAC_Handler     (void) { __BKPT(3); }
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

// Set MOSI/SCK/MISO/SS pins up for SPI comms with FPGA
void select_spi() {
  fpga_spi.begin();
  fpga_spi.beginTransaction(SPISettings(4000000L, MSBFIRST, SPI_MODE0));
  sercom2.disableSPI();  // disable SERCOM so we can write registers.

  // Now reconfigure in master mode.
  SERCOM2->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_MODE_SPI_MASTER |
                           SERCOM_SPI_CTRLA_DOPO(SPI_TX_PAD) |
                           SERCOM_SPI_CTRLA_DIPO(SPI_RX_PAD) |
                           MSB_FIRST << SERCOM_SPI_CTRLA_DORD_Pos;

  SERCOM2->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_RXEN |
                           SERCOM_SPI_CTRLB_CHSIZE(SPI_CHAR_SIZE_8_BITS);


  // fpga_spi.begin() doesn't get these right, as there are two SERCOM options
  // for these pins.
  pinPeripheral(FPGA_MOSI_PIN, PIO_SERCOM_ALT);
  pinPeripheral(FPGA_SCK_PIN, PIO_SERCOM_ALT);
  pinPeripheral(FPGA_MISO_PIN, PIO_SERCOM);
  // pinPeripheral(FPGA_SS_PIN, PIO_SERCOM);
  pinMode(FPGA_SS_PIN, OUTPUT);
  digitalWrite(FPGA_SS_PIN, HIGH);

  // And start it up again!
  sercom2.enableSPI();

  // But disable interrupts
  SERCOM2->SPI.INTENCLR.reg = SERCOM_SPI_INTENCLR_ERROR |
                              SERCOM_SPI_INTENCLR_SSL |
                              SERCOM_SPI_INTENCLR_RXC |
                              SERCOM_SPI_INTENCLR_TXC |
                              SERCOM_SPI_INTENCLR_DRE;

  // Don't call fpga_spi.beginTransaction, as that will switch back to master mode.
}

// Send/receive a byte over SPI, optionally dumping details to the serial port.
uint8_t spi_transfer(uint8_t b) {
  uint8_t r = fpga_spi.transfer(b);
#ifdef SHOW_ALL_SPI_TRANSFERS
  Serial.print("[");
  Serial.print(b, HEX);
  Serial.print(" -> ");
  Serial.print(r, HEX);
  Serial.print("]");
#endif
  return r;
}

static void set_jtag_pins_idle() {
  // JTAG disabled so we don't conflict with a plugged-in programmer
  pinMode(TDO_PIN, INPUT);
  pinMode(TDI_PIN, INPUT);
  pinMode(TMS_PIN, INPUT);
  pinMode(TCK_PIN, INPUT);
}

void reset() {
  // Turn off LED
  digitalWrite(LINK_ACT_LED_PIN, HIGH);
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

  pinMode(LINK_ACT_LED_PIN, OUTPUT);
  digitalWrite(LINK_ACT_LED_PIN, HIGH);  // LED off

  // Enable SPI port on SERCOM2
  select_spi();

  // Force-reset all other SERCOMs, just in case one of them is the cause of our spurious hard faults
  sercom0.resetUART();
  sercom1.resetUART();
  sercom3.resetUART();
  sercom4.resetUART();
  sercom5.resetUART();

  // Set up our GPIOs
  // pinMode(TARGET_POWERED_PIN, INPUT);

  pinMode(TARGET_RESET_PIN, OUTPUT);
  digitalWrite(TARGET_RESET_PIN, LOW);

  // Output 48MHz clock for FPGA
  // DIV=2 gives 24MHz, so DIV=1 is probably what we want
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(4) | GCLK_GENDIV_DIV(1);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(4) |
                      GCLK_GENCTRL_SRC_DFLL48M |  // GCK4 sourced by DFLL48M ()
                      GCLK_GENCTRL_OE |
                      GCLK_GENCTRL_IDC |
                      GCLK_GENCTRL_GENEN;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  // enable GCLK_IO[4] on PA10
  pinPeripheral(CLOCK_48MHZ_PIN, PIO_AC_CLK);

  set_jtag_pins_idle();

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

// Return true if the target has a byte for us to receive
bool target_available() {
  digitalWrite(FPGA_SS_PIN, LOW);
  uint8_t fpga_status = spi_transfer(0);  // nothing to send, no buffer space
  digitalWrite(FPGA_SS_PIN, HIGH);
  return (fpga_status & 0x80) == 0x80;  // FPGA has a byte to send
}

// Read a byte if available, returning -1 if the FPGA doesn't have a byte to send
int target_readbyte() {
  digitalWrite(FPGA_SS_PIN, LOW);
  uint8_t fpga_status = spi_transfer(0x40);  // nothing to send, but have buffer space
  int r = -1;  // FPGA has nothing to send
  if (fpga_status & 0x80) {
    r = spi_transfer(0);
    // Serial.print("<");
    // Serial.print((uint8_t)r, HEX);
    // Serial.print(">");
  }
  digitalWrite(FPGA_SS_PIN, HIGH);
  return r;
}

// Wait up to 500 ms to read a byte, returning -1 if the FPGA doesn't send anything before we time out
int target_readbyte_block() {
  long start_ts = millis();
  while (1) {
    int b = target_readbyte();
    if (b != -1) return (uint8_t)(b & 0xFF);
    if (millis() - start_ts > 500) {
      Serial.println("Timeout waiting for remote to send a byte");
      return -1;
    }
  }
}

uint32_t target_input_checksum = 0;

// Read four bytes from the target.
// TODO make this return something more useful than (uint32_t)(-1) on failure
uint32_t target_readword(int* status) {
  if (status) *status = 0;

  int b = target_readbyte_block();
  if (b < 0) { if (status) *status = -1; return b; }
  uint32_t w = (uint32_t)b;

  b = target_readbyte_block();
  if (b < 0) { if (status) *status = -1; return b; }
  w = (w << 8) | (uint32_t)b;

  b = target_readbyte_block();
  if (b < 0) { if (status) *status = -1; return b; }
  w = (w << 8) | (uint32_t)b;

  b = target_readbyte_block();
  if (b < 0) { if (status) *status = -1; return b; }
  w = (w << 8) | (uint32_t)b;

  target_input_checksum += w;
  return w;
}

void target_reset_input_checksum() {
  target_input_checksum = 0;
}

void target_verify_input_checksum() {
  int status = 0;
  uint32_t cs = target_readword(&status);
  if (status < 0) {
    Serial.print("ERROR reading checksum word");
    return;
  }
  Serial.print("read cs ");
  Serial.print(cs, HEX);
  Serial.print(" and sum is ");
  Serial.println(target_input_checksum);
  if (target_input_checksum != 0) {
    Serial.println("CHECKSUM FAIL");
  }
}

int target_sendbyte(uint8_t b) {
  digitalWrite(FPGA_SS_PIN, LOW);
  uint8_t fpga_status = spi_transfer(0x80);  // We have a byte to write, but no buffer space
  bool sent = false;
  if (fpga_status & 0x40) {
    // Now send the byte
    (void)spi_transfer(b);
    sent = true;
  }
  digitalWrite(FPGA_SS_PIN, HIGH);

  return sent ? 0 : -1;
}

int target_sendbyte_block(uint8_t b) {
  // Buffer should always be empty before a sendbyte call
  int rx_byte = target_readbyte();
  if (rx_byte != -1) {
    Serial.print("(");
    Serial.print((uint8_t)rx_byte, HEX);
    Serial.print("!)");
  }

  Serial.print(">");
  Serial.print(b, HEX);

  long start_ts = millis();
  while (1) {
    digitalWrite(FPGA_SS_PIN, LOW);
    uint8_t fpga_status = spi_transfer(0x80);  // We have a byte to write, but no buffer space
    if (fpga_status & 0x40) break;  // FPGA is ready for our byte
    if (millis() - start_ts > 500) {
      digitalWrite(FPGA_SS_PIN, HIGH);
      Serial.println("Timeout waiting for remote to accept a byte");
      return -1;
    }
  }

  // Now send the byte
  (void)spi_transfer(b);
  digitalWrite(FPGA_SS_PIN, HIGH);

  return 0;
}

uint32_t target_checksum = 0;

void target_reset_checksum() {
  target_checksum = 0;
}

int target_sendword(uint32_t w) {
  int r = target_sendbyte_block((uint8_t)((w & (uint32_t)0xff000000) >> 24));
  if (r < 0) return r;
  r = target_sendbyte_block((uint8_t)((w & (uint32_t)0xff0000) >> 16));
  if (r < 0) return r;
  r = target_sendbyte_block((uint8_t)((w & (uint32_t)0xff00) >> 8));
  if (r < 0) return r;
  r = target_sendbyte_block((uint8_t)((w & (uint32_t)0xff)));
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
    bool want_tx = false;
    int b = target_readbyte_block();
    if (b < 0) {
      Serial.println("Timeout in wait_ready");
      return -2;
    }
    if (check_disconnect()) return -1;

    Serial.print("<");
    Serial.print(b, HEX);
    Serial.print(">");
    if (b == 0x90) return 0;
  }
}

uint8_t last_command = 0;
int target_start_command(uint8_t command) {
  last_command = command;
  int r = target_sendbyte_block(command);
  if (r < 0) return r;
  target_reset_checksum();
  return 0;
}

int target_finish_command() {
  int r = target_send_checksum();
  if (r < 0) return r;
  Serial.print("Response: ");
  r = target_readbyte_block();
  if (r < 0) return r;
  uint8_t b = (uint8_t)r;
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
  uint32_t start_addr_confirmation = target_readword(NULL);
  target_reset_input_checksum();
  Serial.print("End addr: ");
  Serial.println(start_addr_confirmation, HEX);
  Serial.println("START DATA");
  for (uint32_t i = 0; i < n_words; ++i) {
    int status;
    Serial.println(target_readword(&status), HEX);
    if (status < 0) return status;
    if (check_disconnect()) return -1;
  }
  Serial.println("END DATA");
  target_verify_input_checksum();
  return 0;
}

int write_word_to_memory(uint32_t addr, uint32_t data) {
  CR(target_start_command(CMD_WRITE | CMD_WRITE_NEW_DATA | CMD_WRITE_INCREMENT_ADDR | CMD_WRITE_WORD));  // Write words
  CR(target_sendword(1));
  CR(target_sendword(addr));
  CR(target_sendword(data));
  CR(target_finish_command());
  uint32_t cs = target_readword(NULL);
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
      // Turn off LED
      digitalWrite(LINK_ACT_LED_PIN, HIGH);
    }
  } else if (!serial_active) {
    // USB serial connection is active
    serial_active_when = millis();
    serial_active = 1;
  } else if (serial_active == 1 && millis() - serial_active_when > 10) {
    // Turn on LED
    digitalWrite(LINK_ACT_LED_PIN, LOW);
    // Serial port should have settled by now
    Serial.print("MCU at ");
    Serial.print(SystemCoreClock);
    Serial.println(" MHz");
    Serial.print("Uptime: ");
    Serial.print(millis());
    Serial.println(" ms");
    serial_active = 2;
    machine = ARCHIMEDES;

    target_reset();

    Serial.println("OK");
  } else if (serial_active == 2) {
    // Serial port is actually open.
  }

  int i = target_readbyte();
  if (i >= 0) {
    uint8_t b = (uint8_t)i;
    // Not proxying POST output; data received between commands
    static int ff_count = 0;
    if (b == 0xff) ++ff_count; else ff_count = 0;
    if (ff_count < 100) {
      Serial.print("[");
      Serial.print(b, HEX);
      Serial.print("]");
    }
    if (b == 0x90) {
      if (!target_started) {
        Serial.println("Target is alive!");
      }
      target_started = true;
      state = TARGET_WAITING_FOR_COMMAND;
    }
  }

  if (Serial.available()) {
    int c = Serial.read();
    switch (c) {
      case 'C': {
        // program FPGA
        Serial.println("SEND SVF");
        arduino_play_svf(TMS_PIN, TDI_PIN, TDO_PIN, TCK_PIN, -1);
        set_jtag_pins_idle();
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
      case '*': {
        // Passthrough mode
        target_reset();
        Serial.println("Enter passthrough mode");
        int byte_to_send = -1;
        while (1) {
          if (check_disconnect()) break;
          if (Serial.availableForWrite()) {
            int i = target_readbyte();
            if (i >= 0) {
              // Serial.println(i, HEX);
              Serial.write((uint8_t)i);
            }
          }
          if (byte_to_send != -1) {
            int r = target_sendbyte((uint8_t)byte_to_send);
            if (r == 0) byte_to_send = -1;
          }
          if (byte_to_send == -1 && Serial.available()) {
            byte_to_send = Serial.read();
          }
        }
        break;
      }
      case 'p': {
        Serial.println("LCD mode - Monitoring POST output:");
        target_sendbyte_block(0);
        while (!check_disconnect()) {
          int i = target_readbyte();
          if (i >= 0) {
            uint8_t b = (uint8_t)i;
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
        read_words_from_memory(rom_start, 8 * 1024 * 1024 / 4);
        break;
      }
      case 'r': {
        Serial.println("Reading from RAM");
        uint32_t ram_start = (machine == ARCHIMEDES) ? 0x2000000 : 0x18000000;  // A7000 soldered RAM
        read_words_from_memory(ram_start, 16);
        if (target_readbyte_block() != 0x90) break;
        read_words_from_memory(ram_start + 1024 * 1024, 16);
        if (target_readbyte_block() != 0x90) break;
        Serial.println("RAM read successful");
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
          uint32_t cs = target_readword(NULL);
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
      //   target_sendbyte_block(0x18);
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
