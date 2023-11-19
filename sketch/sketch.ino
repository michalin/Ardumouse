#include "ps2mouse.h"

void on_mouse_ready(uint8_t t)
{
    if(t == BASIC)
        Serial.print("Basic ");
    else
        Serial.print("Wheel ");

    Serial.println("mouse connected");
}

void on_mouse_disconnected()
{
    for (int i = 5; i<=10; i++)
        digitalWrite(i, 0);
    Serial.println("Mouse disconnected");
}

void on_mouse(uint8_t btn, int8_t xmov, int8_t ymov, int8_t wheel)
{
    char status[64];
    static int16_t x=128, y=128, w=240;
    sprintf(status, "Button: %i, xmov: %i, ymov: %i, wheel: %i\n", btn, xmov, ymov, wheel);
    y -= ymov/4;
    if(y < 0) y = 0;
    if(y > 255) y = 255;
    x -= xmov/4;
    if(x < 0) x = 0;
    if(x > 255) x = 255;
    w += wheel * 4; 
    if(w < 0) w = 0;
    if(w > 255) w = 255;
    analogWrite(5, x);
    analogWrite(6, y);
    analogWrite(9, w);
    if(btn)
    {
        digitalWrite(7, btn & 1);
        digitalWrite(8, btn & 2);
        digitalWrite(10, btn & 4);
    }
    //Serial.println(status);
}

void setup()
{
    Serial.begin(115200);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    mouse_init();
}

void loop()
{
    mouse_update();
}
