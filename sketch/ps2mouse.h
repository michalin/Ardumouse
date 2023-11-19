#ifndef PS2MOUSE_H
#define PS2MOUSE_H

enum type
{
    BASIC = 0,
    INTELLIMOUSE = 3
};

enum status
{
    OFFLINE,
    READY
};

void mouse_init();
/**
 * @brief Asynchronously queries the last mouse movement.
 * @return true on success or false on error or if no mouse connected
 */
bool mouse_update();

/**
 * @brief Called when data from the mouse has been received.
 * 
 * @param button Left: 1, middle: 2, right: 4
 * @param xmov X-Movement
 * @param ymov Y-Movement
 * @param wheel Wheel
 */
void on_mouse(uint8_t button, int8_t xmov, int8_t ymov, int8_t wheel);

#endif