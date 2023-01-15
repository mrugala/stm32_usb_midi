/**
 * @file main.c
 * @author Philip Karlsson
 * @version V1.0.0
 * @date 29-November-2014
 * @brief This file contains a simple example usage of the new midi device class.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "main.h"

// Midi
#include "usbd_usr.h"
#include "usbd_desc.h"

#include "usbd_midi_core.h"
#include "usbd_midi_user.h"

#define BUTTON(GPIO, GPIO_Pin_N)      (GPIO->IDR & GPIO_Pin_N)
#define ever ;;
#define BUTTON_MODE_MONOSTABLE 0
#define BUTTON_MODE_BISTABLE   1

    // Private variables
    volatile uint32_t time_var1, time_var2;
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

// Private function prototypes
void Delay(volatile uint32_t nCount);
void init();
void calculation_test();

typedef struct {
    GPIO_TypeDef* button_gpio_bank;
    int32_t button_gpio_pin;
    int8_t button_mode;
    GPIO_TypeDef *led_gpio_bank;
    int32_t led_gpio_pin;
    uint8_t noteOn[4];
    uint8_t noteOff[4];
    uint8_t active;
} midi_button;

typedef struct {
    uint8_t message[256];
    uint8_t length;
} midi_message;

midi_button midi_buttons[] = {
    { //User button, green LED
        .button_gpio_bank = GPIOA, .button_gpio_pin = GPIO_Pin_0, .button_mode = BUTTON_MODE_MONOSTABLE,
        .led_gpio_bank = GPIOD, .led_gpio_pin = GPIO_Pin_12, 
        .noteOn = {0x08, 0x90, 0x45, 0x40},
        .noteOff = {0x08, 0x80, 0x45, 0x40}
    },
    { //blue button, blue LED
        .button_gpio_bank = GPIOE, .button_gpio_pin = GPIO_Pin_2, .button_mode = BUTTON_MODE_BISTABLE,
        .led_gpio_bank = GPIOD, .led_gpio_pin = GPIO_Pin_15,
        .noteOn = {0x08, 0x90, 0x46, 0x40},
        .noteOff = {0x08, 0x80, 0x46, 0x40}
    },
    { //red button, red LED
        .button_gpio_bank = GPIOE, .button_gpio_pin = GPIO_Pin_4, .button_mode = BUTTON_MODE_BISTABLE,
        .led_gpio_bank = GPIOD, .led_gpio_pin = GPIO_Pin_14,
        .noteOn = {0x08, 0x90, 0x47, 0x40},
        .noteOff = {0x08, 0x80, 0x47, 0x40}
    },
    { //white LED button
        .button_gpio_bank = GPIOE, .button_gpio_pin = GPIO_Pin_3, .button_mode = BUTTON_MODE_MONOSTABLE,
        .led_gpio_bank = GPIOD, .led_gpio_pin = GPIO_Pin_1, 
        .noteOn = {0x08, 0x90, 0x48, 0x40},
        .noteOff = {0x08, 0x80, 0x48, 0x40}
    },
};

midi_message midi_msg = {.message = {0}, .length = 0};

void midi_disable(midi_button* button)
{
    if (!button->active)
        return;
    button->active = 0;
    GPIO_ResetBits(button->led_gpio_bank, button->led_gpio_pin);
    memcpy((void*)&(midi_msg.message[midi_msg.length]), (void*)&button->noteOff, 4);
    midi_msg.length += 4;
}

void midi_enable(midi_button* button)
{
    if (button->active)
        return;
    button->active = 1;
    GPIO_SetBits(button->led_gpio_bank, button->led_gpio_pin);
    memcpy((void*)&(midi_msg.message[midi_msg.length]), (void*)&button->noteOn, 4);
    midi_msg.length += 4;
}

void apply_midi()
{
    send_MIDI_msg(midi_msg.message, midi_msg.length);
    midi_msg.length = 0;
}

void act_button(midi_button* button)
{
    if (BUTTON_MODE_MONOSTABLE == button->button_mode) {
        if (BUTTON(button->button_gpio_bank, button->button_gpio_pin)) {
            Delay(100);
            if (!BUTTON(button->button_gpio_bank, button->button_gpio_pin))
            {
                if (!button->active){
                    midi_enable(button);
                }
                else {
                    midi_disable(button);
                }
            }
        }
    }
    else if (BUTTON_MODE_BISTABLE == button->button_mode) {
        uint8_t button_state = BUTTON(button->button_gpio_bank, button->button_gpio_pin);
        if (button_state != button->active)
        {
            if (button_state)
            {
                midi_enable(button);
            }
            else
            {
                midi_disable(button);
            }
        }
    }
}

int main(void) {
	init();
	for(ever) {
        for (int i = 0; i < (sizeof(midi_buttons) / sizeof(midi_buttons[0])); ++i)
        {
            act_button(&midi_buttons[i]);
        }
        apply_midi();
        Delay(10);
    }
	return 0;
}


void init() {
	GPIO_InitTypeDef  GPIO_InitStructure;

	// ---------- SysTick timer -------- //
	if (SysTick_Config(SystemCoreClock / 1000)) {
		// Capture error
		while (1){};
	}

	// ---------- GPIO -------- //
	// GPIOD Periph clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// Configure PD12, PD13, PD14 and PD15 in output pushpull mode
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// GPIOE Periph clock enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

	// ------------- USB -------------- //
    /* Initialize the midi driver */
	usbd_midi_init_driver(&MIDI_fops);
    /* Make the connection and initialize to USB_OTG/usbdc_core */
	USBD_Init(&USB_OTG_dev,
	            USB_OTG_FS_CORE_ID,
	            &USR_desc,
				&USBD_MIDI_cb,
	            &USR_cb);
}

/*
 * Called from systick handler
 */
void timing_handler() {
	if (time_var1) {
		time_var1--;
	}

	time_var2++;
}

/*
 * Delay a number of systick cycles (1ms)
 */
void Delay(volatile uint32_t nCount) {
	time_var1 = nCount;
	while(time_var1){};
}

/*
 * Dummy function to avoid compiler error
 */
void _init() {

}

