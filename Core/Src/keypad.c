/*
 * keypad.c
 *
 *  Created on: 12-Feb-2026
 *      Author: rajes
 */
#include "keypad.h"
uint8_t row;

// Keypad layout
char keymap[4][4] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};


GPIO_TypeDef* rowPorts[4] = {GPIOC, GPIOC, GPIOC, GPIOC};
uint16_t rowPins[4] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3};

GPIO_TypeDef* colPorts[4] = {GPIOC, GPIOC, GPIOC, GPIOC};
uint16_t colPins[4] = {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7};

char Keypad_GetKey(void)
{
    for(uint8_t row = 0; row < 4; row++)
    {
        // Set all rows LOW
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

        // Set current row HIGH
        HAL_GPIO_WritePin(GPIOC, (GPIO_PIN_0 << row), GPIO_PIN_SET);

        HAL_Delay(1);

        if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET) return keymap[row][0];
        if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_SET) return keymap[row][1];
        if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == GPIO_PIN_SET) return keymap[row][2];
        if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_SET) return keymap[row][3];
    }

    return 0;
}



