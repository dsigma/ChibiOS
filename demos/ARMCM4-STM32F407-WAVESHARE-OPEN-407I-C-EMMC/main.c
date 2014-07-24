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
#include "string.h"

#if STM32_SERIAL_USE_USART2
  BaseSequentialStream *chp = (BaseSequentialStream *)&SD2;
#endif

#define BUFFER_BLOCK_COUNT  128

#define TEST_BLOCK_COUNT    (BUFFER_BLOCK_COUNT * 100)

uint8_t block_buffer[512 * BUFFER_BLOCK_COUNT];

#define KBPS(usec) (((uint64_t)TEST_BLOCK_COUNT * (uint64_t)500000) / (uint64_t)usec)


void print_sdc_error(void)
{
  chprintf(chp, "SDCD1.errors = %04x\r\n", SDCD1.errors);
  if( SDCD1.errors & SDC_CMD_CRC_ERROR )
    chprintf(chp, "SDC_CMD_CRC_ERROR\r\n");
  if( SDCD1.errors & SDC_DATA_CRC_ERROR )
    chprintf(chp, "SDC_DATA_CRC_ERROR\r\n");
  if( SDCD1.errors & SDC_COMMAND_TIMEOUT )
    chprintf(chp, "SDC_COMMAND_TIMEOUT\r\n");
  if( SDCD1.errors & SDC_DATA_TIMEOUT )
    chprintf(chp, "SDC_DATA_TIMEOUT\r\n");
  if( SDCD1.errors & SDC_TX_UNDERRUN )
    chprintf(chp, "SDC_TX_UNDERRUN\r\n");
  if( SDCD1.errors & SDC_RX_OVERRUN )
    chprintf(chp, "SDC_RX_OVERRUN\r\n");
  if( SDCD1.errors & SDC_STARTBIT_ERROR )
    chprintf(chp, "SDC_STARTBIT_ERROR\r\n");
  if( SDCD1.errors & SDC_OVERFLOW_ERROR )
    chprintf(chp, "SDC_OVERFLOW_ERROR\r\n");

  SDCD1.errors = 0;
}

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

/*
 * Debug output and heartbeat thread.
 */
static WORKING_AREA(waThreadLED, 512);
static msg_t ThreadLED(void *arg) {
  (void)arg;
  chRegSetThreadName("LED");

  /* Blinky Light Loop */
  palClearPad(GPIOH, GPIOH_LED1);
  palClearPad(GPIOH, GPIOH_LED2);
  palClearPad(GPIOI, GPIOI_LED3);
  palClearPad(GPIOI, GPIOI_LED4);

  while (TRUE) {
    palSetPad(GPIOH, GPIOH_LED1);
    chThdSleep(MS2ST(500));
    palClearPad(GPIOH, GPIOH_LED1);
    palSetPad(GPIOH, GPIOH_LED2);
    chThdSleep(MS2ST(500));
    palClearPad(GPIOH, GPIOH_LED2);
    palSetPad(GPIOI, GPIOI_LED3);
    chThdSleep(MS2ST(500));
    palClearPad(GPIOI, GPIOI_LED3);
    palSetPad(GPIOI, GPIOI_LED4);
    chThdSleep(MS2ST(500));
    palClearPad(GPIOI, GPIOI_LED4);
    palSetPad(GPIOI, GPIOI_LED3);
    chThdSleep(MS2ST(500));
    palClearPad(GPIOI, GPIOI_LED3);
    palSetPad(GPIOH, GPIOH_LED2);
    chThdSleep(MS2ST(500));
    palClearPad(GPIOH, GPIOH_LED2);
  }

  return(0);
}

msg_t MMCTest( TimeMeasurement *tm_ws, TimeMeasurement *tm_rs, TimeMeasurement *tm_wm, TimeMeasurement *tm_rm ) {
  uint32_t i;
  uint32_t retry_count;
  uint32_t kbps_value;

  chprintf(chp, "eMMC SDC Test\r\n");

  chprintf(chp, "SDC_BUS_WIDTH = %d\r\n", SDC_BUS_WIDTH);

  // power cycle card
  palSetPad(GPIOC, GPIOC_SDIO_POWER); // off
  chThdSleepMilliseconds(100);
  palClearPad(GPIOC, GPIOC_SDIO_POWER); // on
  chThdSleepMilliseconds(100);


  if (sdcConnect(&SDCD1) == CH_FAILED) {
    chprintf(chp, "sdcConnect FAILED\r\n");
    print_sdc_error();
    goto sdc_failed;
  } else {
    chprintf(chp, "sdcConnect OK\r\n");
  }

  chprintf(chp, "SDCD1->capacity = %d blocks, %d GB\r\n", SDCD1.capacity,
           SDCD1.capacity / (2 * 1024 * 1024));
  
  chprintf(chp, "== Zeros write test\r\n");

  memset(block_buffer, 0, sizeof(block_buffer));

  retry_count = 0;
  while( sdcWrite(&SDCD1, 0, block_buffer, 1) == CH_FAILED )
  {
    chprintf(chp, "sdcWrite FAILED (%d}\r\n", retry_count);
    print_sdc_error();
    
    if( ++retry_count > 2 )
      goto sdc_failed;
    chprintf(chp, "- Retry -\r\n");
  }
  chprintf(chp, "sdcWrite OK\r\n");

  memset(block_buffer, 0xaa, sizeof(block_buffer)); // clobber buffer

  retry_count = 0;
  while( sdcRead(&SDCD1, 0, block_buffer, 1) == CH_FAILED )
  {
    chprintf(chp, "sdcRead FAILED (%d)\r\n", retry_count);
    print_sdc_error();

    if( ++retry_count > 2 )
      goto sdc_failed;
    chprintf(chp, "- Retry -\r\n");
  }
  chprintf(chp, "sdcRead OK\r\n");
  
  for( i = 0; i < 512; i++ )
  {
    if( block_buffer[i] != 0 )
    {
      chprintf(chp, "Test failed\r\noffset = %d, value = %02x\r\n", i, block_buffer[i]);
      goto sdc_failed;
    }
  }

  chprintf(chp, "Test Passed\r\n");

  chprintf(chp, "== Ones write test\r\n");

  memset(block_buffer, 0xff, sizeof(block_buffer));

  retry_count = 0;
  while( sdcWrite(&SDCD1, 0, block_buffer, 1) == CH_FAILED )
  {
    chprintf(chp, "sdcWrite FAILED (%d)\r\n", retry_count);
    print_sdc_error();
    
    if( ++retry_count > 2 )
      goto sdc_failed;
    chprintf(chp, "- Retry -\r\n");
  }
  chprintf(chp, "sdcWrite OK\r\n");

  memset(block_buffer, 0xaa, sizeof(block_buffer)); // clobber buffer

  retry_count = 0;
  while( sdcRead(&SDCD1, 0, block_buffer, 1) == CH_FAILED )
  {
    chprintf(chp, "sdcRead FAILED (%d)\r\n", retry_count);
    print_sdc_error();

    if( ++retry_count > 2 )
      goto sdc_failed;
    chprintf(chp, "- Retry -\r\n");
  }
  chprintf(chp, "sdcRead OK\r\n");

  for( i = 0; i < 512; i++ )
  {
    if( block_buffer[i] != 0xff )
    {
      chprintf(chp, "Test failed\r\noffset = %d, value = %02x\r\n", i, block_buffer[i]);
      goto sdc_failed;
    }
  }

  chprintf(chp, "Test Passed\r\n");

  
  chprintf(chp, "== Sequence write test\r\n");

  for( i = 0; i < sizeof(block_buffer); i++ )
  {
    block_buffer[i] = (uint8_t)i;
  }

  retry_count = 0;
  while( sdcWrite(&SDCD1, 0, block_buffer, 1) == CH_FAILED )
  {
    chprintf(chp, "sdcWrite FAILED (%d)\r\n", retry_count);
    print_sdc_error();

    if( ++retry_count > 2 )
      goto sdc_failed;
    chprintf(chp, "- Retry -\r\n");
  }
  chprintf(chp, "sdcWrite OK\r\n");
  
  memset(block_buffer, 0xaa, sizeof(block_buffer)); // clobber buffer

  retry_count = 0;
  while( sdcRead(&SDCD1, 0, block_buffer, 1) == CH_FAILED )
  {
    chprintf(chp, "sdcRead FAILED (%d)\r\n", retry_count);
    print_sdc_error();
    
    if( ++retry_count > 2 )
      goto sdc_failed;
    chprintf(chp, "- Retry -\r\n");
  }
  chprintf(chp, "sdcRead OK\r\n");

  for( i = 0; i < 512; i++ )
  {
    if( block_buffer[i] != (uint8_t)i )
    {
      chprintf(chp, "Test failed\r\noffset = %d, value = %02x\r\n", i, block_buffer[i]);
      goto sdc_failed;
    }
  }

  chprintf(chp, "Test Passed\r\n");

  chprintf(chp, "== Single block write speed test\r\n");
  chprintf(chp, "%d blocks, %d KB\r\n", TEST_BLOCK_COUNT, TEST_BLOCK_COUNT / 2);

  tmStartMeasurement(tm_ws);
  for( i = 0; i < TEST_BLOCK_COUNT; i++ )
  {
    if( sdcWrite(&SDCD1, i, block_buffer, 1) == CH_FAILED )
    {
      chprintf(chp, "sdcWrite FAILED\r\n");
      print_sdc_error();
    }
  }
  tmStopMeasurement(tm_ws);

  kbps_value = KBPS(RTT2US(tm_ws->last));
  chprintf(chp, "%8d us, %d KB/s\r\n", RTT2US(tm_ws->last), kbps_value);
  chprintf(chp, "Test Passed\r\n");


  chprintf(chp, "== Multiple block write speed test\r\n");
  chprintf(chp, "%d blocks, %d KB\r\n", TEST_BLOCK_COUNT, TEST_BLOCK_COUNT / 2);
  chprintf(chp, "%d blocks per write\r\n", BUFFER_BLOCK_COUNT);

  tmStartMeasurement(tm_wm);
  for( i = 0; i < TEST_BLOCK_COUNT; i += BUFFER_BLOCK_COUNT )
  {
    if( sdcWrite(&SDCD1, i, block_buffer, BUFFER_BLOCK_COUNT) == CH_FAILED )
    {
      chprintf(chp, "sdcWrite FAILED\r\n");
      print_sdc_error();
    }
  }
  tmStopMeasurement(tm_wm);

  kbps_value = KBPS(RTT2US(tm_wm->last));
  chprintf(chp, "%8d us, %d KB/s\r\n", RTT2US(tm_wm->last), kbps_value);
  chprintf(chp, "Test Passed\r\n");


  chprintf(chp, "== Single block read speed test\r\n");
  chprintf(chp, "%d blocks, %d KB\r\n", TEST_BLOCK_COUNT, TEST_BLOCK_COUNT / 2);

  tmStartMeasurement(tm_rs);
  for( i = 0; i < TEST_BLOCK_COUNT; i++ )
  {
    if( sdcRead(&SDCD1, i, block_buffer, 1) == CH_FAILED )
    {
      chprintf(chp, "sdcRead FAILED\r\n");
      print_sdc_error();
    }
  }
  tmStopMeasurement(tm_rs);

  kbps_value = KBPS(RTT2US(tm_rs->last));
  chprintf(chp, "%8d us, %d KB/s\r\n", RTT2US(tm_rs->last), kbps_value);
  chprintf(chp, "Test Passed\r\n");


  chprintf(chp, "== Multiple block read speed test\r\n");
  chprintf(chp, "%d blocks, %d KB\r\n", TEST_BLOCK_COUNT, TEST_BLOCK_COUNT / 2);
  chprintf(chp, "%d blocks per read\r\n", BUFFER_BLOCK_COUNT);

  tmStartMeasurement(tm_rm);
  for( i = 0; i < TEST_BLOCK_COUNT; i += BUFFER_BLOCK_COUNT )
  {
    if( sdcRead(&SDCD1, i, block_buffer, BUFFER_BLOCK_COUNT) == CH_FAILED )
    {
      chprintf(chp, "sdcWrite FAILED\r\n");
      print_sdc_error();
    }
  }
  tmStopMeasurement(tm_rm);

  kbps_value = KBPS(RTT2US(tm_rm->last));
  chprintf(chp, "%8d us, %d KB/s\r\n", RTT2US(tm_rm->last), kbps_value);
  chprintf(chp, "Test Passed\r\n");


  sdcDisconnect(&SDCD1);
  chprintf(chp, "Card Disconnected\r\n");

sdc_failed:

  return(0);
}


/*===========================================================================*/
/* Initialization and main thread.                                           */
/*===========================================================================*/

/*
 * Application entry point.
 */
int main(void) {
  uint32_t i;
  static TimeMeasurement tm_ws, tm_rs, tm_wm, tm_rm;

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Activates the serial driver 2 using the driver default configuration.
   * PA2(TX) and PA3(RX) are routed to USART2.
   */
#if STM32_SERIAL_USE_USART2
  sdStart(&SD2, NULL);
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));
#endif
  
  /*
   * Initialize SDIO peripheral
   */
  sdcStart(&SDCD1, 0);
  sdcDisconnect(&SDCD1);

  tmObjectInit(&tm_ws);
  tmObjectInit(&tm_rs);
  tmObjectInit(&tm_wm);
  tmObjectInit(&tm_rm);

  /*
   * Creates the example thread.
   */
  chThdCreateStatic(waThreadLED, sizeof(waThreadLED), NORMALPRIO, ThreadLED, NULL);

  while (TRUE) {
    for( i = 5; i > 0; i-- )
    {
      chprintf(chp, ".", i);
      chThdSleepSeconds(1);
    }
    chprintf(chp, "\r\n");
    chprintf(chp, "================\r\n");
    MMCTest(&tm_ws, &tm_rs, &tm_wm, &tm_rm);
    chprintf(chp, "================\r\n");
  }
}

