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
#include "portmacro.h"
#include "projdefs.h"
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

#define MAX_ITEMS 25
#define MAX_LIVES 10
#define collectionLevel 5
#define deductionLevel 10
#define rand_a 1103515245
#define rand_c 12345
#define MAX_LEVELS 3
tRectangle backdrop;
tRectangle lava;
tRectangle statusbar;
tRectangle bottomBackdrop;

const uint8_t LEVELS[MAX_LEVELS] = {5, 10, 15};

volatile uint32_t seed = 1;
volatile bool g_bMoveRight = false;
volatile bool g_bMoveLeft = false;

// Linear Congruential Generator (LCG)
int randInt(uint32_t modulus) {
  seed = rand_a * seed + rand_c;
  return seed % modulus;
}

enum GAMESTATES { GAMESTART, GAME, GAMEOVER };

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
extern SemaphoreHandle_t xItemsSemaphore;
extern SemaphoreHandle_t xBasketSemaphore;
extern SemaphoreHandle_t xDisplaySemaphore;
extern SemaphoreHandle_t xDisplayRestartSemaphore;
/*
 * Location Struct
 */
typedef struct location {
  int x;
  int y;
} location;

/*
 * Basket Struct
 */
typedef struct basket_t {
  location
      currentLocation; // location x and y (y should be set as deductionLevel)
  uint8_t size;        // size of the basket
  uint8_t lives;       // lives lost of the current game
  uint8_t score;       // Score
  uint8_t level;       // difficuity of the game
  enum GAMESTATES state;
} basket_t;

// Share Bucket Struct
basket_t basket;
tRectangle basketLine;

void initBasket(tContext sContext) {
  basket.size = 100;
  basket.currentLocation.x = GrContextDpyWidthGet(&sContext) / 2;
  basket.lives = 0;
  basket.score = 0;
}

void drawBasket(tContext sContext, basket_t tempBasket) {
  GrContextForegroundSet(&sContext, ClrBrown);
  int delta = tempBasket.size / 2 - 24;
  basketLine.i16XMin = tempBasket.currentLocation.x - delta;
  basketLine.i16XMax = tempBasket.currentLocation.x + delta;
  basketLine.i16YMin = lava.i16YMin - 5;
  basketLine.i16YMax = lava.i16YMin - 1;
  GrRectFill(&sContext, &basketLine);
  GrTransparentImageDraw(&sContext, assetBasketlhs,
                         tempBasket.currentLocation.x - delta - 16,
                         lava.i16YMin - 17, 0x00);
  GrTransparentImageDraw(&sContext, assetBasketrhs,
                         tempBasket.currentLocation.x + delta,
                          lava.i16YMin- 17, 0x00);
}

/*
 * Item Variables
 */
enum STATUS { INACTIVE, ACTIVE };

// Enum of Types POWERUP - 0, FRUIT - 1
// Note: Powerup should show up at every x level up. (x could be set as a marco)
enum itemType {
  BANANA,
  APPLE,
  PEAR,
  WATERMELON,
  POWERUP, // Increases the Size of the basket, but doesen't deduct the score
  HEART
};

typedef struct item_t {
  enum STATUS status;
  enum itemType type;       // Falling items can be a powerup, or fruit
  location currentlocation; // Current location of the item x, y
  uint8_t level;            // Level (aka speed) of the item.
  struct location previouslocation;
} item_t;

// Share Item List;
volatile item_t itemsList[MAX_ITEMS];
/*
 * data structures for the graphics library
 */
tContext sContext;
tRectangle sRect;
BaseType_t xHigherPriorityTaskWoken = pdFALSE;

/*
 * Clear Item List
 * Make sure items are all set to inactive
 */
void ClearItems() {
  for (int i = 0; i < MAX_ITEMS; i++) {
    itemsList[i].status = INACTIVE;
  }
}

/*
 * Item Creator
 * @waring MUST BE PROTECTED
 */
int NewItem(uint32_t screenWidth, int level, int type) {
  int i;
  if (xSemaphoreTake(xItemsSemaphore, portMAX_DELAY) != pdPASS) {
    return -2;
  }
  for (i = 0; i <= MAX_ITEMS; i++) {
    if (i == MAX_ITEMS) {
      return -1;
    }
    if (itemsList[i].status == INACTIVE) {
      break;
    }
  }
  itemsList[i].status = ACTIVE;
  if (type == -1) {
    itemsList[i].type = randInt(4);
  } else {
    itemsList[i].type = type;
  }
  itemsList[i].currentlocation.y = 24;
  if (level - 1 > MAX_LEVELS) {
    itemsList[i].level = 2;
  } else if (level < 0) {
    itemsList[i].level = 0;
  } else {
    itemsList[i].level = level;
  }
  itemsList[i].currentlocation.x = randInt(screenWidth - 33) + 16;
  itemsList[i].previouslocation.y = 24;
  itemsList[i].previouslocation.x = randInt(screenWidth - 33) + 16;
  xSemaphoreGive(xItemsSemaphore);
  return 0;
}
/*
 *
 * @arg items Expecting a copy of items list
 */
int drawFruit(tContext *context, item_t items[MAX_ITEMS]) {
  for (int i = 0; i < MAX_ITEMS; i++) {
    uint8_t *asset;
    uint8_t *assetinv;
    GrContextForegroundSet(&sContext, ClrLightBlue);
    switch (items[i].type) {
        case POWERUP:
            asset = assetLevelup;
            assetinv = assetLevelup_inv;
            break;
        case HEART:
            asset = assetHeart;
            break;
        case BANANA:
            asset = assetBanana;
            assetinv = assetBanana_inv;
            break;
        case APPLE:
            asset = assetApple;
            assetinv = assetApple_inv;
            break;
        case PEAR:
            asset = assetPear;
            assetinv = assetPear_inv;
            break;
        case WATERMELON:
            asset = assetWatermelon;
            assetinv = assetWatermelon_inv;
            break;
        default:
            return 1;
        }
    if (items[i].status == INACTIVE) {

      if (items[i].previouslocation.x != 0 ||
          items[i].previouslocation.y != 0) {

        GrImageDraw(&sContext, assetinv, items[i].previouslocation.x,
            items[i].previouslocation.y);
        if (xSemaphoreTake(xBasketSemaphore, portMAX_DELAY) == pdPASS) {
          itemsList[i].previouslocation.x = 0;
          itemsList[i].previouslocation.y = 0;
          xSemaphoreGive(xBasketSemaphore);
        }
      }
      continue;
    }

    if (items[i].previouslocation.x != items[i].currentlocation.x ||
        items[i].previouslocation.y != items[i].currentlocation.y) {
            

      tRectangle clearRect;
      clearRect.i16XMin = items[i].previouslocation.x;
      clearRect.i16YMin = items[i].previouslocation.y;
      clearRect.i16XMax = items[i].previouslocation.x + 16;
      clearRect.i16YMax = items[i].previouslocation.y + 16;

      GrImageDraw(&sContext, assetinv, items[i].previouslocation.x-16,
        items[i].previouslocation.y-16);

      if (xSemaphoreTake(xBasketSemaphore, portMAX_DELAY) == pdPASS) {
        itemsList[i].previouslocation.x = items[i].currentlocation.x;
        itemsList[i].previouslocation.y = items[i].currentlocation.y;
        xSemaphoreGive(xBasketSemaphore);
      }
    }
    GrTransparentImageDraw(&sContext, asset, items[i].currentlocation.x,
                           items[i].currentlocation.y, 0x00);
  }
  return 0;
}

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvDisplayTask(void *pvParameters);
static void prvGameLogicTask(void *pvParameters);

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
  xTaskCreate(prvDisplayTask, "Display", configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 1, NULL);
  xTaskCreate(prvGameLogicTask, "GameLogicTask", configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 2, NULL);
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

void xTimerHandlerA(void) {
  TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

  for (int i = 0; i < MAX_ITEMS; i++) {
    if (itemsList[i].status == INACTIVE) {
      continue;
    }
    itemsList[i].previouslocation.y = itemsList[i].currentlocation.y;
    itemsList[i].currentlocation.y += LEVELS[itemsList[i].level];
  }

  xSemaphoreGiveFromISR(xDisplaySemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void xTimerHandlerB(void) { TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT); }

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
    if ((ui32Status & USR_SW1)) {
      g_bMoveRight = true;
    } else if ((ui32Status & USR_SW2)) {
      g_bMoveLeft = true;
    }

    /* Give the semaphore to unblock prvProcessSwitchInputTask.  */
    xSemaphoreGiveFromISR(xButtonSemaphore, &xLEDTaskWoken);

    /* This FreeRTOS API call will handle the context switch if it is
     * required or have no effect if that is not needed. */
    portYIELD_FROM_ISR(xLEDTaskWoken);
  }

  /* Update the time stamp. */
  g_ui32TimeStamp = xTaskGetTickCount();
  xSemaphoreGiveFromISR(xDisplaySemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/*-----------------------------------------------------------*/

void checkItems() {
  for (int i = 0; i < MAX_ITEMS; i++) {
    if (xSemaphoreTake(xItemsSemaphore, portMAX_DELAY) != pdPASS) {
      continue;
    }
    if (itemsList[i].status == INACTIVE) {
      xSemaphoreGive(xItemsSemaphore);
      continue;
    }

    int itemBottomY = itemsList[i].currentlocation.y + 10;
    if (itemBottomY >= lava.i16YMin - collectionLevel) {
      if (xSemaphoreTake(xBasketSemaphore, portMAX_DELAY) == pdPASS) {
        int delta = basket.size / 2 - 16;
        int basketLeftEdge = basket.currentLocation.x - delta - 16;
        int basketRightEdge = basket.currentLocation.x + delta + 16;

        int itemLeftEdge = itemsList[i].currentlocation.x;
        int itemRightEdge = itemsList[i].currentlocation.x + 16;

        if ((itemRightEdge >= basketLeftEdge) &&
            (itemLeftEdge <= basketRightEdge)) {
          itemsList[i].status = INACTIVE;
          if (itemsList[i].type > 3) {
              if (itemsList[i].type == POWERUP){
                basket.size += 20;
              } else if (itemsList[i].type == HEART){
                if( basket.lives > 0) basket.lives -= 1;
              }
          } else {
            basket.score += 1;
          }
          xSemaphoreGive(xBasketSemaphore);
          xSemaphoreGive(xItemsSemaphore);
          xSemaphoreGive(xDisplaySemaphore);
          continue;
        }
        xSemaphoreGive(xBasketSemaphore);
      }
    }

    if (itemBottomY >= lava.i16YMin ) {
      if (xSemaphoreTake(xBasketSemaphore, portMAX_DELAY) == pdPASS) {
        if (itemsList[i].type < 4) {
          if (basket.lives + 1 < MAX_LIVES) {
            basket.lives += 1;
          } else {
            basket.lives += 1;
            basket.state = GAMEOVER;
            xSemaphoreGive(xItemsSemaphore);
            xSemaphoreGive(xBasketSemaphore);
            xSemaphoreGive(xDisplaySemaphore);
            continue;
          }
        }
        itemsList[i].status = INACTIVE;
        xSemaphoreGive(xBasketSemaphore);
      }
    }
    xSemaphoreGive(xItemsSemaphore);
  }
}

static void prvButtonTask(void *_) {}


/**
 * DrawGame
 */
void DrawGame(tContext sContent) {
  GrContextForegroundSet(&sContext, ClrLightBlue);
  GrRectFill(&sContext, &backdrop);
  GrContextForegroundSet(&sContext, ClrRed);
  GrRectFill(&sContext, &lava);
}

void DrawBottom(tContext sContent) {
  GrContextForegroundSet(&sContext, ClrLightBlue);
  GrRectFill(&sContext, &bottomBackdrop);

  GrContextForegroundSet(&sContext, ClrRed);
  GrRectFill(&sContext, &lava);
}
/**
 * Draw Hearts
 */
void DrawStatusBar(tContext sContext, basket_t tempBasket) {
  GrContextForegroundSet(&sContext, ClrDarkBlue);
  GrRectFill(&sContext, &statusbar);

  GrContextFontSet(&sContext, &g_sFontCm20);

  GrContextForegroundSet(&sContext, ClrWhite);

  char scoreText[16];
  usnprintf(scoreText, sizeof(scoreText), "Score: %u", tempBasket.score);

  GrStringDraw(&sContext, scoreText, -1, 5, 5, false);

  GrContextForegroundSet(&sContext, ClrBlack);

  for (int i = 0; i < MAX_LIVES; i++) {
    if (0 <= (i - tempBasket.lives)) {
      GrTransparentImageDraw(&sContext, assetHeart,
                             GrContextDpyWidthGet(&sContext) -
                                 (MAX_LIVES - i) * 17 - 1,
                             5, ClrBlack);
    } else {
      GrTransparentImageDraw(&sContext, assetHeartlost,
                             GrContextDpyWidthGet(&sContext) -
                                 (MAX_LIVES - i) * 17 - 1,
                             5, ClrBlack);
    }
  }
}

void splashScreen(tContext sContext, basket_t temp_basket) {
    if (temp_basket.state == GAMESTART){
        GrTransparentImageDraw(&sContext, assetIntructionsv3, 0, 0, 0x00);
    } else {
        GrImageDraw(&sContext, assetGameover, 80, 60);
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
  initBasket(sContext);
  backdrop.i16XMin = 0;
  backdrop.i16YMin = 24;
  backdrop.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
  backdrop.i16YMax = GrContextDpyHeightGet(&sContext) - deductionLevel -1;
  lava.i16XMin = 0;
  lava.i16YMin = GrContextDpyHeightGet(&sContext) - deductionLevel;
  lava.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
  lava.i16YMax = GrContextDpyHeightGet(&sContext);
  statusbar.i16XMin = 0;
  statusbar.i16YMin = 0;
  statusbar.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
  statusbar.i16YMax = 23;
  bottomBackdrop.i16XMin = 0;
  bottomBackdrop.i16YMin =  GrContextDpyHeightGet(&sContext) - deductionLevel -20;
  bottomBackdrop.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
  bottomBackdrop.i16YMax =  GrContextDpyHeightGet(&sContext) - deductionLevel -1;
  basket.state = GAMESTART;
  DrawGame(sContext);
  DrawBottom(sContext);
  xSemaphoreGive(xItemsSemaphore);
  xSemaphoreGive(xBasketSemaphore);
  xSemaphoreGive(xDisplaySemaphore);
  //
  // Main loop for display task
  //
  for (;;) {
    if (xSemaphoreTake(xDisplaySemaphore, portMAX_DELAY) == pdPASS) {
      DrawBottom(sContext);
      // Give
      if (xSemaphoreTake(xBasketSemaphore, portMAX_DELAY) == pdPASS) {
        basket_t temp_basket = basket;
        xSemaphoreGive(xBasketSemaphore);
        if (basket.state != GAME) {
          DrawStatusBar(sContext, temp_basket);
          splashScreen(sContext,temp_basket);
          if (xSemaphoreTake(xDisplayRestartSemaphore, portMAX_DELAY) !=
              pdPASS) {
            xSemaphoreGive(xDisplaySemaphore);
            continue;
          }
          if (xSemaphoreTake(xBasketSemaphore,portMAX_DELAY) == pdPASS){
              initBasket(sContext);
              basket.state = GAME;
              temp_basket = basket;
              xSemaphoreGive(xBasketSemaphore);
          }
          if (xSemaphoreTake(xItemsSemaphore,portMAX_DELAY) == pdPASS){
            ClearItems();
            xSemaphoreGive(xItemsSemaphore);
          }
          DrawGame(sContext);
        }
        drawBasket(sContext, temp_basket);
        DrawStatusBar(sContext, temp_basket);
      }

      // Get Item and Basket List for display
      if (xSemaphoreTake(xItemsSemaphore, portMAX_DELAY) == pdPASS) {
        item_t temp_itemList[MAX_ITEMS];
        for (int i = 0; i < MAX_ITEMS; i++) {
          temp_itemList[i] = itemsList[i];
        }
        xSemaphoreGive(xItemsSemaphore);
        drawFruit(&sContext, temp_itemList);
      }
    }
  }
}
/*-----------------------------------------------------------*/

static void prvGameLogicTask(void *pvParameters) {
  TickType_t xLastUpdateTime = xTaskGetTickCount();
  TickType_t xLastMoveTime = xLastUpdateTime;
  TickType_t xLastLevelTime = xLastUpdateTime;

  const TickType_t xItemInterval = pdMS_TO_TICKS(2e3);
  const TickType_t xLevelInterval = pdMS_TO_TICKS(1e4);
  const TickType_t xMoveInterval = 10;
  const int moveStep = 2;
  int level = 0;

  for (;;) {
    TickType_t xCurrentTime = xTaskGetTickCount();

    if ((xCurrentTime - xLastMoveTime) >= xMoveInterval) {
      if (xSemaphoreTake(xBasketSemaphore, portMAX_DELAY) == pdPASS) {
        if (basket.state != GAME) {
          xSemaphoreGive(xBasketSemaphore);
          if (g_bMoveRight || g_bMoveLeft) {
            level = 0;
            xSemaphoreGive(xDisplaySemaphore);
            xSemaphoreGive(xDisplayRestartSemaphore);
          };
          seed = xCurrentTime;
          xLastLevelTime = xCurrentTime;
          xLastUpdateTime = xCurrentTime;
          xLastMoveTime = xCurrentTime;
          continue;
        }
        uint8_t delta = basket.size/2 - 16;
        if (g_bMoveRight && lava.i16XMax > basket.currentLocation.x + delta) {
            basket.currentLocation.x += moveStep;
        }
        if (g_bMoveLeft && lava.i16XMin < basket.currentLocation.x - delta) {
          basket.currentLocation.x -= moveStep;
        }
        xSemaphoreGive(xBasketSemaphore);
        xLastMoveTime = xCurrentTime;
      };
    }

    if (GPIOPinRead(BUTTONS_GPIO_BASE, USR_SW1) != 0) {
      g_bMoveRight = false;
    }
    if (GPIOPinRead(BUTTONS_GPIO_BASE, USR_SW2) != 0) {
      g_bMoveLeft = false;
    }
    if ((xCurrentTime - xLastLevelTime) >= xLevelInterval) {
        if ( level - 1 < MAX_LEVELS) {
        level += 1;
        }
      if (randInt(100) < 50){
        NewItem(GrContextDpyWidthGet(&sContext), 2, POWERUP);
      } else {
        NewItem(GrContextDpyWidthGet(&sContext), 2, HEART);
      }
      xLastLevelTime = xCurrentTime;
    }
    if ((xCurrentTime - xLastUpdateTime) >= xItemInterval) {
      for (int i = 0; i <= level; i++) {
        NewItem(GrContextDpyWidthGet(&sContext), i, -1);
      }
      xLastUpdateTime = xCurrentTime;
    }

    checkItems();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
