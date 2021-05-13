/*

This file was obtained from https://github.com/microsoft/uf2-samdx1/blob/master/inc/neopixel.h

It originally contained no license comment, so here is the repository LICENSE file:

 > The MIT License (MIT)
 >
 > Copyright (c) Microsoft
 >
 > Permission is hereby granted, free of charge, to any person obtaining a copy
 > of this software and associated documentation files (the "Software"), to deal
 > in the Software without restriction, including without limitation the rights
 > to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 > copies of the Software, and to permit persons to whom the Software is
 > furnished to do so, subject to the following conditions:
 >
 > The above copyright notice and this permission notice shall be included in all
 > copies or substantial portions of the Software.
 >
 > THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 > IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 > FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 > AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 > LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 > OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 > SOFTWARE.
 >
 > Third Party Programs: The software may include third party programs that
 > Microsoft, not the third party, licenses to you under this agreement.
 > Notices, if any, for the third party programs are included for your
 > information only.
 >
 >
 >
 > Otherwise, where noted:
 >
 >  * ----------------------------------------------------------------------------
 >  *         SAM Software Package License
 >  * ----------------------------------------------------------------------------
 >  * Copyright (c) 2011-2014, Atmel Corporation
 >  *
 >  * All rights reserved.
 >  *
 >  * Redistribution and use in source and binary forms, with or without
 >  * modification, are permitted provided that the following condition is met:
 >  *
 >  * Redistributions of source code must retain the above copyright notice,
 >  * this list of conditions and the disclaimer below.
 >  *
 >  * Atmel's name may not be used to endorse or promote products derived from
 >  * this software without specific prior written permission.
 >  *
 >  * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 >  * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 >  * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 >  * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 >  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 >  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 >  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 >  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 >  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 >  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 >  * ----------------------------------------------------------------------------

*/

#ifndef DEVICE_NEOPIXEL_H
#define DEVICE_NEOPIXEL_H

#ifdef BOARD_NEOPIXEL_PIN

#ifndef SAMD21
#error "SAMD21 expected for POST box code"
#endif

// The timings are taken from Adafruit's NeoPixel library

static void neopixel_send_buffer_core(volatile uint32_t *clraddr, uint32_t pinMask,
                                      const uint8_t *ptr, int numBytes) __attribute__((naked));

static void neopixel_send_buffer_core(volatile uint32_t *clraddr, uint32_t pinMask,
                                      const uint8_t *ptr, int numBytes) {
    asm volatile("        push    {r4, r5, r6, lr};"
                 "        add     r3, r2, r3;"
                 "loopLoad:"
                 "        ldrb r5, [r2, #0];" // r5 := *ptr
                 "        add  r2, #1;"       // ptr++
                 "        movs    r4, #128;"  // r4-mask, 0x80
                 "loopBit:"
                 "        str r1, [r0, #4];"                    // set
                 #ifdef SAMD21
                 "        movs r6, #3; d2: sub r6, #1; bne d2;" // delay 3
                 #endif
                 #ifdef SAMD51
                 "        movs r6, #3; d2: subs r6, #1; bne d2;" // delay 3
                 #endif
                 "        tst r4, r5;"                          // mask&r5
                 "        bne skipclr;"
                 "        str r1, [r0, #0];" // clr
                 "skipclr:"
                 #ifdef SAMD21
                 "        movs r6, #6; d0: sub r6, #1; bne d0;" // delay 6
                 #endif
                 #ifdef SAMD51
                 "        movs r6, #6; d0: subs r6, #1; bne d0;" // delay 6
                 #endif
                 "        str r1, [r0, #0];"   // clr (possibly again, doesn't matter)
                 #ifdef SAMD21
                 "        asr     r4, r4, #1;" // mask >>= 1
                 #endif
                 #ifdef SAMD51
                 "        asrs     r4, r4, #1;" // mask >>= 1
                 #endif
                 "        beq     nextbyte;"
                 "        uxtb    r4, r4;"
                 #ifdef SAMD21
                 "        movs r6, #2; d1: sub r6, #1; bne d1;" // delay 2
                 #endif
                 #ifdef SAMD51
                 "        movs r6, #2; d1: subs r6, #1; bne d1;" // delay 2
                 #endif
                 "        b       loopBit;"
                 "nextbyte:"
                 "        cmp r2, r3;"
                 "        bcs stop;"
                 "        b loopLoad;"
                 "stop:"
                 "        pop {r4, r5, r6, pc};"
                 "");
}

// this assumes the pin has been configured correctly
static inline void neopixel_send_buffer(const uint8_t *ptr, int numBytes) {
    uint8_t portNum = BOARD_NEOPIXEL_PIN / 32;
    uint32_t pinMask = 1ul << (BOARD_NEOPIXEL_PIN % 32);

    PINOP(BOARD_NEOPIXEL_PIN, DIRSET);

#if (USB_VID == 0x239a) && (USB_PID == 0x0013)  // Adafruit Metro M0
    // turn off mux too, needed for metro m0
    PORT->Group[BOARD_NEOPIXEL_PIN / 32].PINCFG[BOARD_NEOPIXEL_PIN % 32].reg =
        (uint8_t)(PORT_PINCFG_INEN);
#endif

    PINOP(BOARD_NEOPIXEL_PIN, OUTCLR);
    delay(1);

    volatile uint32_t *clraddr = &PORT->Group[portNum].OUTCLR.reg;

    // equivalent to cpu_irq_is_enabled()
    if (__get_PRIMASK() == 0) {
        __disable_irq();
        neopixel_send_buffer_core(clraddr, pinMask, ptr, numBytes);
        __enable_irq();
    } else {
        neopixel_send_buffer_core(clraddr, pinMask, ptr, numBytes);
    }
}

#endif

#endif
