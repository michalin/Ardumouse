/* Copyright (C) 2020  Doctor Volt
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <https://www.gnu.org/licenses>.
*/
#include <Arduino.h>
#include "ps2mouse.h"

uint8_t ucsr0b;
uint8_t ucsr0c;
byte received = 0;
byte ps2buf[3];
volatile uint16_t to_send;
uint8_t state = OFFLINE;
uint8_t mousetype = BASIC;

struct
{
    uint8_t ack;     // Acknowle of command
    uint8_t btn : 3; // Bit0: Left button, Bit 1: right button, Bit2: middle button
    uint8_t one : 1; // Always 1
    uint8_t xs : 1;  // X-Axis Sign Bit (9-Bit X-Axis Relative Offset)
    uint8_t ys : 1;  // Y-Axis Sign Bit (9-Bit Y-Axis Relative Offset)
    uint8_t ovf : 2; // X and Y-Axis Overflow
    int8_t xmov;    // X-Axis Movement Value
    int8_t ymov;    // Y-Axis Movement Value
    uint8_t wheel;
    // uint8_t btn2 : 4;
} ps2mouse;

#define CLK PD4 // USART 0 external clock pin (must not be changed)
#define RX PD0  // USART0 Receive Pin

//#define DEBUGPIN 10

#if defined DEBUGPIN
#define DEBUG                  \
    digitalWrite(DEBUGPIN, 1); \
    /*delayMicroseconds(4);*/  \
    digitalWrite(DEBUGPIN, 0);
#endif
void on_mouse_ready(byte) __attribute__((weak));
void on_mouse_disconnected() __attribute__((weak));

void ps2_init()
{
    // Save affected UART Control and status registers
    ucsr0b = UCSR0B;
    ucsr0c = UCSR0C;
    bitSet(PCICR, PCIE2);
    //Serial.setTimeout(100);
#if defined DEBUGPIN
    pinMode(DEBUGPIN, OUTPUT);
#endif
}

void ps2_begin()
{
    Serial.flush();
    bitClear(UCSR0B, TXEN0);
    // Synchronous mode, odd parity, 8 data bits, external clock 
    UCSR0C = (1 << UMSEL00) + (1 << UPM01) + (1 << UPM00) + (1 << UCSZ01) + (1 << UCSZ00);
    while (Serial.read() != -1)
        ; // Flush Rx buffer
}

void ps2_end()
{
    Serial.flush();
    UCSR0B = ucsr0b; // Restore register content
    UCSR0C = ucsr0c;
    while (Serial.read() != -1)
        ; // Flush Rx buffer
}

/*ISR that sends PS/2 commands over the RX pin*/
ISR(PCINT2_vect)
{
    static uint8_t bitcnt = 0;
    static bool ack = 0;
    if (digitalRead(CLK))
    {
        if (bitcnt <= 9)
        {
            digitalWrite(RX, (to_send >> bitcnt) & 1);
        }
        else
        {                              // All shifted out, now wait for ack
            pinMode(RX, INPUT_PULLUP); // Give control of data line back to device
            ack = true;
        }
        bitcnt++;
    }
    else
    {
        if (ack && digitalRead)
        {
            bitcnt = 0;
            to_send = 0xFFFF;
            ack = false;
        }
    }
}

bool ps2_command(uint8_t *response, uint8_t len, uint8_t cmd, uint8_t par = 0)
{
    // Assemble the frame to be sent to the device
    to_send = !__builtin_parity(cmd); // Parity odd
    to_send = (to_send << 8) + cmd;   // Command
    to_send <<= 1;                    // Stop bit
    // Initiate command mode of device
    bitClear(UCSR0B, RXEN0); // Use Rx pin as I/O
    bitSet(PCMSK2, PCINT20);
    pinMode(CLK, OUTPUT);
    delayMicroseconds(100);
    pinMode(RX, OUTPUT);
    digitalWrite(RX, 0);
    pinMode(CLK, INPUT);

    uint32_t msecs = millis();
    while (to_send < 0xFF00 && millis() - msecs < 100)
        ; // Wait until device ack or timeout
    bitClear(PCMSK2, PCINT20);
    delayMicroseconds(100);
    bitSet(UCSR0B, RXEN0); // Now set Rx pin to receive the response
    ps2_begin();
    if (par) // If there is a parameter
    {
        Serial.readBytes(response, 1); // Command acknowledged
        if (response[0] == 0xFA)
        {
            ps2_command(response, len, par); // Call function recursively to send parameter
        }
        else
        {
            ps2_end();
            return false;
        }
    }

    size_t retval = Serial.readBytes(response, len);
    ps2_end();
    if (response[0] == 0xFA)
        return true;
    return false;
}

void mouse_init()
{
    ps2_init();
    Serial.setTimeout(10);
}

bool mouse_start(uint8_t &mtype)
{
    uint8_t init[2];
    memset(init, 0, 2);
    //if (!ps2_command(init, 1, 0xFF)) // Reset mouse
    //    return false;
    if (!ps2_command(init, 1, 0xF4)) // Enable mouse
        return false;

    if (!ps2_command(init, 1, 0xF0)) // Set remote mode (Read data with 0xEB command)
        return false;

    ps2_command(init, 1, 0xF3, 200);
    ps2_command(init, 1, 0xF3, 100);
    ps2_command(init, 1, 0xF3, 80); // Check if it's an intellimouse, then device-id will be 3
    if (!ps2_command(init, 2, 0xF2)) // Get device Id
        return false;

    mtype = init[1];                 // 0: standard PS/2 mouse 3: Mouse has scroll wheel
    return true;
}

bool mouse_update()
{
    if (state == OFFLINE)
        if (mouse_start(mousetype))
        {
            state = READY;
            on_mouse_ready(mousetype);
        }
        else
            return false;
    // Poll position and buttons of mouse
    ps2mouse.ack = 0;
    uint8_t resplen = mousetype == BASIC ? 4 : 5;
    ps2_command((byte *)&ps2mouse, resplen, 0xEB); // Request status
    if (!ps2mouse.ack)
    {
        state = OFFLINE;
        on_mouse_disconnected();
        return false;
    }

    if (ps2mouse.btn || ps2mouse.xmov || ps2mouse.ymov || ps2mouse.wheel)
        on_mouse(ps2mouse.btn,
                 ps2mouse.xmov,
                 ps2mouse.ymov,
                 ps2mouse.wheel);
    return true;
}
