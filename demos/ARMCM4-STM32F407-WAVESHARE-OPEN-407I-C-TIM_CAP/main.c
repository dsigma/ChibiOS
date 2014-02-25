/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012,2013 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "ch.h"
#include "hal.h"

#include "chprintf.h"
#include "timcap.h"


#if STM32_SERIAL_USE_USART2
  BaseSequentialStream *chp = (BaseSequentialStream *)&SD2;
#endif
#if STM32_SERIAL_USE_USART3
  BaseSequentialStream *chp = (BaseSequentialStream *)&SD3;
#endif


volatile uint32_t ccr_values[TIMCAP_CHANNEL_4 + 1];
volatile uint32_t callback_count[TIMCAP_CHANNEL_4 + 1];
volatile uint32_t of_count = 0;

void capture1_cb(TIMCAPDriver *timcapp) {
  ccr_values[TIMCAP_CHANNEL_1] = timcap_lld_get_ccr(timcapp, TIMCAP_CHANNEL_1);
  callback_count[TIMCAP_CHANNEL_1]++;
}

void capture2_cb(TIMCAPDriver *timcapp) {
  ccr_values[TIMCAP_CHANNEL_2] = timcap_lld_get_ccr(timcapp, TIMCAP_CHANNEL_2);
  callback_count[TIMCAP_CHANNEL_2]++;
}

void capture3_cb(TIMCAPDriver *timcapp) {
  ccr_values[TIMCAP_CHANNEL_3] = timcap_lld_get_ccr(timcapp, TIMCAP_CHANNEL_3);
  callback_count[TIMCAP_CHANNEL_3]++;
}

void capture4_cb(TIMCAPDriver *timcapp) {
  ccr_values[TIMCAP_CHANNEL_4] = timcap_lld_get_ccr(timcapp, TIMCAP_CHANNEL_4);
  callback_count[TIMCAP_CHANNEL_4]++;
}

void of_cb(TIMCAPDriver *timcapp) {
  (void)timcapp;
  of_count++;
}

TIMCAPDriver *timcapp = &TIMCAPD4;
TIMCAPConfig tc_conf = {
   {TIMCAP_INPUT_ACTIVE_HIGH, TIMCAP_INPUT_ACTIVE_HIGH, TIMCAP_INPUT_ACTIVE_HIGH, TIMCAP_INPUT_ACTIVE_HIGH},
   10000,
   /*Pins PD12, PD13, PD14, PD15*/
   {capture1_cb,capture2_cb, capture3_cb, capture4_cb},
   of_cb,
   0
};

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

#define TEST_WA_SIZE    THD_WA_SIZE(256)

/*
 * Debug output and heartbeat thread.
 */
static WORKING_AREA(waThread1, 512);
static msg_t Thread1(void *arg) {
  (void)arg;
  chRegSetThreadName("reader");

  /* Blinky Light Loop */
  palTogglePad(GPIOH, GPIOH_LED1);
  palTogglePad(GPIOH, GPIOH_LED2);

  while (TRUE) {
    chprintf(chp, "-----------------------------------------------------\r\n");
    chprintf(chp, "timcapp->tim->CR1  = 0x%X\r\n", timcapp->tim->CR1);
    chprintf(chp, "timcapp->tim->CR2  = 0x%X\r\n", timcapp->tim->CR2);
    chprintf(chp, "timcapp->tim->CCMR1  = 0x%X\r\n", timcapp->tim->CCMR1);
    chprintf(chp, "timcapp->tim->CCMR2  = 0x%X\r\n", timcapp->tim->CCMR2);
    chprintf(chp, "timcapp->tim->CCMR3  = 0x%X\r\n", timcapp->tim->CCMR3);
    chprintf(chp, "timcapp->tim->CCER   = 0x%X\r\n", timcapp->tim->CCER);
    chprintf(chp, "timcapp->tim->DIER   = 0x%X\r\n", timcapp->tim->DIER);
    chprintf(chp, "timcapp->tim->PSC    = 0x%X\r\n", timcapp->tim->PSC);
    chprintf(chp, "timcapp->tim->SMCR   = 0x%X\r\n", timcapp->tim->SMCR);
    chprintf(chp, "timcapp->tim->SR     = 0x%X\r\n", timcapp->tim->SR);
    chprintf(chp, "timcapp->tim->CNT     = 0x%X\r\n", timcapp->tim->CNT);
    chprintf(chp, "timcapp->tim->ARR     = 0x%X\r\n", timcapp->tim->ARR);
    chprintf(chp, "timcapp->tim->CCR[0] = 0x%X\r\n", timcapp->tim->CCR[0]);
    chprintf(chp, "timcapp->tim->CCR[1] = 0x%X\r\n", timcapp->tim->CCR[1]);
    chprintf(chp, "timcapp->tim->CCR[2] = 0x%X\r\n", timcapp->tim->CCR[2]);
    chprintf(chp, "timcapp->tim->CCR[3] = 0x%X\r\n", timcapp->tim->CCR[3]);

    int i = 0;
    for(i = TIMCAP_CHANNEL_1; i <= TIMCAP_CHANNEL_4; i++ ) {
      chprintf(chp, "  ccr_value[%u]=%u,   cb_count=%u\r\n", i, ccr_values[i], callback_count[i]);
    }
    chprintf(chp, "  overflow count=%u\r\n", i, of_count);


    palTogglePad(GPIOH, GPIOH_LED1);
    palTogglePad(GPIOH, GPIOH_LED2);
    palTogglePad(GPIOI, GPIOI_LED3);
    palTogglePad(GPIOI, GPIOI_LED4);

    chThdSleep(MS2ST(1000));
  }

  return(0);
}

/*===========================================================================*/
/* Initialization and main thread.                                           */
/*===========================================================================*/

/*
 * Application entry point.
 */
int main(void) {
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  timcapInit();


  //Configure pin inputs for TIMCAP
  palSetPadMode(GPIOD, 12, PAL_MODE_ALTERNATE(2));
  palSetPadMode(GPIOD, 13, PAL_MODE_ALTERNATE(2));
  palSetPadMode(GPIOD, 14, PAL_MODE_ALTERNATE(2));
  palSetPadMode(GPIOD, 15, PAL_MODE_ALTERNATE(2));

  timcapStart(timcapp, &tc_conf);
  timcapEnable(timcapp);





  /*
   * Activates the serial driver 2 using the driver default configuration.
   * PA2(TX) and PA3(RX) are routed to USART2.
   */
#if STM32_SERIAL_USE_USART2
  sdStart(&SD2, NULL);
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7)); palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));
#endif

#if STM32_SERIAL_USE_USART3
  sdStart(&SD3, NULL);
  palSetPadMode(GPIOC, 10, PAL_MODE_ALTERNATE(7));//TX
  palSetPadMode(GPIOC, 11, PAL_MODE_ALTERNATE(7));//RX
#endif


  /*
   * Creates the example thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 10, Thread1,
                    NULL);

  while (TRUE) {
    chThdSleepMilliseconds(500);
  }
}
