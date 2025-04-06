/*
 * led_task
 *
 * Copyright (C) 2022 Texas Instruments Incorporated
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/******************************************************************************
 *
 * vLEDTask turns on the initial LED, configurations the buttons, and creates
 * prvProcessSwitchInputTask which handles processing the ISR result to adjust
 * the LED per the button inputs.
 *
 * prvProcessSwitchInputTask uses semaphore take to wait until it receives a
 * semaphore from the button ISR.  Once it does, then it processes the button
 * pressed.  Each time the SW1 or SW2 button is pressed, the LED index is
 * updated based on which button has been pressed and the corresponding LED
 * lights up.
 *
 * When either user switch SW1 or SW2 on the EK-TM4C1294XL is pressed, an
 * interrupt is generated and the switch pressed is logged in the global
 * variable g_pui32ButtonPressed.  Then the binary semaphore is given to
 * prvProcessSwitchInputTask before yielding to it.  This is an example of
 * using binary semaphores to defer ISR processing to a task.
 *
 */

/* Standard includes. */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/* Hardware includes. */
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "drivers/rtos_hw_drivers.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

/* Display includes. */
#include "grlib.h"
#include "widget.h"
#include "canvas.h"

/* Include Images */
#include "assets.h"


#include "drivers/Kentec320x240x16_ssd2119_spi.h"
#include "drivers/touch.h"

#define MAX_ITEMS 10
#define collectionLevel 225
#define deductionLevel 230
#define MAX_LIVES 5
#define rand_a 1103515245
#define rand_c 12345


volatile uint32_t seed = 1;

// Linear Congruential Generator (LCG)
int randInt(uint32_t modulus){
    seed = rand_a * seed + rand_c;
    return seed % modulus;
}

/*-----------------------------------------------------------*/
/*
 * Time stamp global variable.
 */
volatile uint32_t g_ui32TimeStamp = 0;

extern volatile uint32_t g_ui32SysClock;

/*
 * Global variable to log the last GPIO button pressed.
 */
volatile static uint32_t g_pui32ButtonPressed;

/*
 * The binary semaphore used by the switch ISR & task.
 */
extern SemaphoreHandle_t xButtonSemaphore;

/*
 * Location Struct
 */
struct location {
    int x;
    int y;
};

/*
 * Basket Struct
 */
typedef struct basket_t {
    struct location currentLocation; // location x and y (y should be set as deductionLevel) 
    uint8_t size; // size of the basket
    uint8_t lives; // score of the current game
    uint8_t level; // difficuity of the game
} basket_t;

// Share Bucket Struct
basket_t basket;

/*
 * Item Variables
 */
enum STATUS {
    INACTIVE,
    ACTIVE
};

// Enum of Types POWERUP - 0, FRUIT - 1
// Note: Powerup should show up at every x level up. (x could be set as a marco)
enum itemType {
    POWERUP, // Increases the Size of the basket, but doesen't deduct the score when it reaches it final y value
    BANANA,
    APPLE,
    PEAR,
    WATERMELON 
};

typedef struct item_t {
    enum STATUS status;
    enum itemType type; // Falling items can be a powerup, or fruit
    struct location currentlocation; // Current location of the item x, y
    uint8_t level; // Level (aka speed) of the item.
} item_t;

// Share Item List;
volatile item_t itemsList[MAX_ITEMS];
/*
 * data structures for the graphics library
 */
tContext sContext;
tRectangle sRect;

/*
 * Clear Item List
 * Make sure items are all set to inactive
 */
void ClearItems(){
    for (int i=0;i<MAX_ITEMS;i++){
        itemsList[i].status = INACTIVE;
    }
}


/*
 * Item Creator
 * @waring MUST BE PROTECTED
 */
int NewItem(item_t *newItem, uint32_t screenWidth){
    int i;
    for (i=0;i <= MAX_ITEMS;i++){
        if (i == MAX_ITEMS){
            return -1;
        }
        if (itemsList[i].status == INACTIVE){
            newItem = &itemsList[i];
            break;
        }
    }
    itemsList[i].status = ACTIVE;
    itemsList[i].type = randInt(3)+1;
    itemsList[i].currentlocation.y = 24;
    itemsList[i].currentlocation.x = randInt(screenWidth-1);
    return 0;
}
/*
 *
 * @arg items Expecting a copy of items list
 */
int drawFruit(tContext *context, item_t items[MAX_ITEMS]){
    for (int i=0; i < MAX_ITEMS; i++){
        uint8_t *asset;
        switch (items[i].type) {
            case BANANA:      asset = assetBanana; break;
            case APPLE:       asset = assetApple; break;
            case PEAR:        asset = assetPear; break;
            case WATERMELON:  asset = assetWatermelon; break;
            default: return 1;
        }
        GrTransparentImageDraw(&sContext, asset, items[i].currentlocation.x,items[i].currentlocation.y, 0x00);
    }
    return 0;
}

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvDisplayTask(void *pvParameters);

/*
 * Called by main() to do example specific hardware configurations and to
 * create the Process Switch task.
 */
void vCreateTask(void);

/*
 * Hardware configuration for the LEDs.
 */
static void prvConfigureLED(void);

/*
 * Timer configuration
 */
static void prvConfigureHWTimer(void);

/*
 * Hardware configuration for the buttons SW1 and SW2 to generate interrupts.
 */
static void prvConfigureButton(void);
/*-----------------------------------------------------------*/

void vCreateDisplayTask(void) {

    /* Light the initial LED. */
    prvConfigureLED();

    /* Configure the button to generate interrupts. */
    prvConfigureButton();

    /* Configure the hardware timer to run in periodic mode. */
    prvConfigureHWTimer();

    /* Create the task as described in the comments at the top of this file.
     *
     * The xTaskCreate parameters in order are:
     *  - The function that implements the task.
     *  - The text name for the LED Task - for debug only as it is not used by
     *    the kernel.
     *  - The size of the stack to allocate to the task.
     *  - The parameter passed to the task - just to check the functionality.
     *  - The priority assigned to the task.
     *  - The task handle is not required, so NULL is passed. */
    xTaskCreate(prvDisplayTask, "LED", configMINIMAL_STACK_SIZE, NULL,
            tskIDLE_PRIORITY + 1, NULL);
}

static void prvConfigureLED(void) {
    /* Configure initial LED state.  PinoutSet() has already configured
     * LED I/O. */
    LEDWrite(LED_D1, LED_D1);
}
/*-----------------------------------------------------------*/

static void prvConfigureButton(void) {
    /* Initialize the LaunchPad Buttons. */
    ButtonsInit();

    /* Configure both switches to trigger an interrupt on a falling edge. */
    GPIOIntTypeSet(BUTTONS_GPIO_BASE, ALL_BUTTONS, GPIO_FALLING_EDGE);

    /* Enable the interrupt for LaunchPad GPIO Port in the GPIO peripheral. */
    GPIOIntEnable(BUTTONS_GPIO_BASE, ALL_BUTTONS);

    /* Enable the Port F interrupt in the NVIC. */
    IntEnable(INT_GPIOJ);

    /* Enable global interrupts in the NVIC. */
    IntMasterEnable();
}

/*-----------------------------------------------------------*/

static void prvConfigureHWTimer(void) {
    /* The Timer 0 peripheral must be enabled for use. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    /* Configure Timer 0 in full-width periodic mode. */
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    /* Set the Timer 0A load value to run at 5 Hz. */
    TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32SysClock / 5);

    /* Configure the Timer 0A interrupt for timeout. */
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    /* Enable the Timer 0A interrupt in the NVIC. */
    IntEnable(INT_TIMER0A);

    /* Enable global interrupts in the NVIC. */
    IntMasterEnable();

    //
    // Start the timer used in this example Task
    // You may need change where this timer is enabled
    //
    TimerEnable(TIMER0_BASE, TIMER_A);
}

/*-----------------------------------------------------------*/

void xTimerHandler(void) {

    /* Clear the hardware interrupt flag for Timer 0A. */
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    /* Update only time based game variables here*/
    // e.g. fruit location
    for (int i = 0; i < MAX_ITEMS; i++){
        if ((itemsList[i].status == INACTIVE) ||(deductionLevel == (itemsList[i].currentlocation.y + 16))){
            continue;
        }
        itemsList[i].currentlocation.y += 10;
    }
}

void xButtonsHandler(void) {
    BaseType_t xLEDTaskWoken;
    uint32_t ui32Status;

    /* Initialize the xLEDTaskWoken as pdFALSE.  This is required as the
     * FreeRTOS interrupt safe API will change it if needed should a
     * context switch be required. */
    xLEDTaskWoken = pdFALSE;

    /* Read the buttons interrupt status to find the cause of the interrupt. */
    ui32Status = GPIOIntStatus(BUTTONS_GPIO_BASE, true);

    /* Clear the interrupt. */
    GPIOIntClear(BUTTONS_GPIO_BASE, ui32Status);

    /* Debounce the input with 100ms filter */
    // Can reduce this value to increase response time of button
    // but if too small can lead to debouncing issues
    if ((xTaskGetTickCount() - g_ui32TimeStamp) > 100) {
        /* Log which button was pressed to trigger the ISR. */
        if ((ui32Status & USR_SW1) == USR_SW1) {
            g_pui32ButtonPressed = USR_SW1;
        } else if ((ui32Status & USR_SW2) == USR_SW2) {
            g_pui32ButtonPressed = USR_SW2;
        }

        /* Give the semaphore to unblock prvProcessSwitchInputTask.  */
        xSemaphoreGiveFromISR(xButtonSemaphore, &xLEDTaskWoken);

        /* This FreeRTOS API call will handle the context switch if it is
         * required or have no effect if that is not needed. */
        portYIELD_FROM_ISR(xLEDTaskWoken);
    }

    /* Update the time stamp. */
    g_ui32TimeStamp = xTaskGetTickCount();
}
/*-----------------------------------------------------------*/

static void prvButtonTask(void *_){

}
tRectangle backdrop; 
tRectangle lava;
tRectangle statusbar;

/**
 * DrawGame
 */
void DrawGame(tContext sContent){
    GrContextForegroundSet(&sContext, ClrLightBlue);
    GrRectFill(&sContext,&backdrop);
    GrContextForegroundSet(&sContext, ClrRed);
    GrRectFill(&sContext,&lava);
}

/**
 * Draw Hearts
 */
void DrawStatusBar(tContext sContext){
    GrContextForegroundSet(&sContext, ClrDarkBlue);
    GrRectFill(&sContext, &statusbar);
    for (int i=0; i < MAX_LIVES; i++){
        if (0 <= (i - basket.lives)){
            GrTransparentImageDraw(&sContext, assetHeart, GrContextDpyWidthGet(&sContext) - (MAX_LIVES - i)*17 - 1,5, ClrBlack);
        } else {
            GrTransparentImageDraw(&sContext, assetHeartlost, GrContextDpyWidthGet(&sContext) - (MAX_LIVES - i)*17 - 1,5, ClrBlack);
        }
    }
}
/*--------------------------TASK FUNCTIONS ---------------------------------*/
// You could either add a second game logic task function and setup in this file
// or create a second src (.c) file and define a game logic task function there

static void prvDisplayTask(void *pvParameters) {
    //
    // Initialize the display driver.
    //
    Kentec320x240x16_SSD2119Init(g_ui32SysClock);

    //
    // Initialize the graphics context.
    //
    GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);
    backdrop.i16XMin = 0;
    backdrop.i16YMin = 24;
    backdrop.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    backdrop.i16YMax = deductionLevel - 1;
    lava.i16XMin = 0;
    lava.i16YMin = deductionLevel;
    lava.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    lava.i16YMax = deductionLevel + 10;
    statusbar.i16XMin = 0;
    statusbar.i16YMin = 0;
    statusbar.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    statusbar.i16YMax = 23;
    DrawGame(sContext);
    DrawStatusBar(sContext);
    //
    // Main loop for display task
    //
    for (;;) {
        /* Block until the Push Button ISR gives the semaphore. */
        if (xSemaphoreTake(xButtonSemaphore, portMAX_DELAY) == pdPASS) {
            /* If the right button is hit, either increment by 1 or reset the
             * index to 0 if it is at 3. */
            DrawStatusBar(sContext);
            DrawGame(sContext);
            item_t *newitem;
            basket.lives+=1;
            NewItem(newitem, GrContextDpyWidthGet(&sContext));
            drawFruit(&sContext, itemsList);
        }
    }
}
/*-----------------------------------------------------------*/
