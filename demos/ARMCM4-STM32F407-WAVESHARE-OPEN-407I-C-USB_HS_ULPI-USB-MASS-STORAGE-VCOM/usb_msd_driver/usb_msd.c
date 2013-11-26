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

/**
 * @file    usb_msd.c
 * @brief   USB Mass Storage Driver code.
 *
 * @addtogroup MSD_USB
 * @{
 */

#include "ch.h"
#include "hal.h"
#include "usb_msd.h"
#include "chprintf.h"
#include "string.h"


#if HAL_USE_MASS_STORAGE_USB || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define MSD_ENABLE_PERF_DEBUG_GPIOS    FALSE

//#define MSD_DEBUG   FALSE
#define MSD_DEBUG   (palReadPad(GPIOI, GPIOI_PIN4))
#define msd_debug_print(chp_arg, args ...) if (MSD_DEBUG && chp_arg != NULL ) { chprintf(chp_arg, args); }
#define msd_debug_err_print(chp_arg, args ...) if ( chp_arg != NULL ) { chprintf(chp_arg, args); }

#if !defined(MSD_RW_LED_ON)
#define MSD_RW_LED_ON()
#endif

#if !defined(MSD_RW_LED_OFF)
#define MSD_RW_LED_OFF()
#endif



/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

static WORKING_AREA(waMassStorage, 1024);
static WORKING_AREA(waMassStorageUSBTransfer, 1024);

static msg_t MassStorageUSBTransferThd(void *arg);
static msg_t MassStorageThd(void *arg);

static Thread *msdThd = NULL;
static Thread *msdUSBTransferThd = NULL;

#define WAIT_ISR_SUCCESS                     0
#define WAIT_ISR_BUSS_RESET_OR_RECONNECT     1
static uint8_t WaitForISR(USBMassStorageDriver *msdp, const bool_t check_reset, const msd_wait_mode_t wait_mode);
static void msdSetDefaultSenseKey(USBMassStorageDriver *msdp);

#define BLOCK_SIZE_INCREMENT                 512
#define BLOCK_WRITE_ITTERATION_COUNT         32

typedef struct {
  uint8_t is_transfer_done;
  /*Number of blocks actually read from USB IN endpoint that should be written to SD card*/
  int num_blocks_to_write;
  /*Number of blocks to read from USB IN endpoint*/
  int max_blocks_to_read;
  uint8_t buf[(BLOCK_SIZE_INCREMENT * BLOCK_WRITE_ITTERATION_COUNT)];
} rw_usb_sd_buffer_t;

static volatile rw_usb_sd_buffer_t rw_ping_pong_buffer[2];
static uint8_t read_buffer[2][BLOCK_SIZE_INCREMENT];



inline uint32_t swap_uint32( uint32_t val ) {
    val = ((val << 8) & 0xFF00FF00 ) | ((val >> 8) & 0xFF00FF );
    return ((val << 16) & 0xFFFF0000) | ((val >> 16) & 0x0000FFFF);
}

#define swap_uint16(x) ((((x) >> 8) & 0xff) | (((x) & 0xff) << 8))

inline uint32_t swap_4byte_buffer(uint8_t *buff) {
  //Note: this is specifically to avoid pointer aliasing and de-referencing words on non-word boundaries
  uint32_t temp = 0;
  memcpy(&temp, buff, sizeof(temp));
  return(swap_uint32(temp));
}

inline uint16_t swap_2byte_buffer(uint8_t *buff) {
  //Note: this is specifically to avoid pointer aliasing and de-referencing words on non-half-word boundaries
  uint16_t temp = 0;
  memcpy(&temp, buff, sizeof(temp));
  return(swap_uint16(temp));
}

void msdBulkInCallbackComplete(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  (void)ep;

  if (ep > 0 && usbp->in_params[ep - 1] != NULL) {
    USBMassStorageDriver *msdp = (USBMassStorageDriver *)usbp->in_params[ep - 1];

    chSysLockFromIsr();
    chBSemSignalI(&(msdp->bsem));

    if (msdp->wait_bulk_in_isr_counter == 0) {
      msdp->wait_bulk_in_isr_counter = 1;
    }

    chSysUnlockFromIsr();
  }
}

void msdBulkOutCallbackComplete(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  (void)ep;

  if (ep > 0 && usbp->in_params[ep - 1] != NULL) {
    USBMassStorageDriver *msdp = (USBMassStorageDriver *)usbp->in_params[ep - 1];

    chSysLockFromIsr();
    chBSemSignalI(&(msdp->bsem));

    if (msdp->wait_bulk_out_isr_counter == 0) {
      msdp->wait_bulk_out_isr_counter = 1;
    }

    chSysUnlockFromIsr();
  }
}


/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/


/**
 * @brief This function will initialize a USBMassStorageDriver structure.
 *
 * @pre Upon entry, the BlockDevice state should be BLK_READY. If it's not BLK_READY, then this will
 * wait untile it becomes BLK_READY.
 *
 * @param[in] usbp            pointer to the @p USBDriver object
 * @param[in] bbdp            pointer to the @p BaseBlockDevice object, such as an SDCDriver object
 * @param[in] msdp            pointer to the @p USBMassStorageDriver object
 * @param[in] ms_ep_number    USB Endpoint Number to be used by the mass storage endpoint
 *
 * @init
 */
usb_msd_driver_state_t msdInit(USBDriver *usbp, BaseBlockDevice *bbdp, USBMassStorageDriver *msdp,
             const usbep_t ms_ep_number, const uint16_t msd_interface_number) {
  uint8_t i;

  msdp->usbp = usbp;
  msdp->driver_state = USB_MSD_DRIVER_OK;
  msdp->state = MSD_STATE_IDLE;
  msdp->trigger_transfer_index = UINT32_MAX;
  msdp->bbdp = bbdp;
  msdp->ms_ep_number = ms_ep_number;
  msdp->msd_interface_number = msd_interface_number;
  msdp->chp = NULL;
  msdp->enable_media_removial = true;

  chEvtInit(&msdp->evt_connected);
  chEvtInit(&msdp->evt_ejected);

  /* Initialize binary semaphore as taken, will cause the thread to initially
   * wait on the  */
  chBSemInit(&msdp->bsem, TRUE);
  /* Initialize binary semaphore as NOT taken */
  chBSemInit(&msdp->usb_transfer_thread_bsem, FALSE);
  chBSemInit(&msdp->mass_sorage_thd_bsem, FALSE);

  /* Initialize sense values to zero */
  for (i = 0; i < sizeof(scsi_sense_response_t); i++) {
    msdp->sense.byte[i] = 0x00;
  }

  /* Response code = 0x70, additional sense length = 0x0A */
  msdp->sense.byte[0] = 0x70;
  msdp->sense.byte[7] = 0x0A;

  /* make sure block device is working and get info */
  msdSetDefaultSenseKey(msdp);

  const uint32_t max_init_wait_time_ms = 2000;
  const uint32_t sleep_time_ms = 50;
  uint32_t sleep_time_counter_ms = 0;

  while (TRUE) {
    blkstate_t state = blkGetDriverState(bbdp);
    if (state == BLK_READY) {
      break;
    }

    chThdSleepMilliseconds(sleep_time_ms);
    sleep_time_counter_ms += sleep_time_counter_ms;
    if( sleep_time_counter_ms > max_init_wait_time_ms ) {
      msdp->driver_state = USB_MSD_DRIVER_ERROR_BLK_DEV_NOT_READY;
      break;
    }
  }

  blkGetInfo(bbdp, &msdp->block_dev_info);

  usbp->in_params[ms_ep_number - 1] = (void *)msdp;
  //usbp->epc[ms_ep_number - 1]->in_cb = &msd_bulk_in_callback_complete;
  //usbp->epc[ms_ep_number - 1]->out_cb = &msd_bulk_out_callback_complete;

  return(msdp->driver_state);
}

/**
 * @brief Starts data handling threads for USB mass storage driver
 *
 * @param[in] msdp      pointer to the @p USBMassStorageDriver object
 *
 * @api
 */
usb_msd_driver_state_t msdStart(USBMassStorageDriver *msdp) {
  /*upon entry, USB bus should be disconnected*/

  if (msdThd == NULL) {
    msdThd = chThdCreateStatic(waMassStorage, sizeof(waMassStorage), NORMALPRIO,
                               MassStorageThd, msdp);
  }

  if (msdUSBTransferThd == NULL) {
    msdUSBTransferThd = chThdCreateStatic(waMassStorageUSBTransfer,
                                          sizeof(waMassStorageUSBTransfer),
                                          NORMALPRIO, MassStorageUSBTransferThd,
                                          msdp);
  }

  return(msdp->driver_state);
}

/**
 * @brief USB Event handler calback
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        USB Endpoint Number
 *
 * @api
 */
#if 0
void msdUsbEvent(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  (void)ep;

  if (ep > 0 && usbp->in_params[ep - 1] != NULL) {
    USBMassStorageDriver *msdp = (USBMassStorageDriver *)usbp->in_params[ep - 1];

    chSysLockFromIsr();
    chBSemSignalI(&(msdp->bsem));
    msdp->interupt_counter++;

    if (msdp->wait_for_command_block_isr_counter == 0) {
      msdp->wait_for_command_block_isr_counter = 1;
    }

    if (msdp->wait_general_isr_counter == 0) {
      msdp->wait_general_isr_counter = 1;
    }
    if (msdp->wait_ping_pong_isr_counter == 0) {
      msdp->wait_ping_pong_isr_counter = 1;
    }

    chSysUnlockFromIsr();
  }
}
#endif


/**
 * @brief   Default requests hook.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @return              The hook status.
 * @retval TRUE         Message handled internally.
 * @retval FALSE        Message not handled.
 *
 * @api
 */
bool_t msdRequestsHook(USBDriver *usbp) {
  return(msdRequestsHook2(usbp, NULL));
}

bool_t msdRequestsHook2(USBDriver *usbp, USBMassStorageDriver *msdp) {
  if (((usbp->setup[0] & USB_RTYPE_TYPE_MASK) == USB_RTYPE_TYPE_CLASS)
      && ((usbp->setup[0] & USB_RTYPE_RECIPIENT_MASK)
          == USB_RTYPE_RECIPIENT_INTERFACE)) {
    /* check that the request is for the MSD interface number.*/
    if( msdp != NULL ) {
      if (MSD_SETUP_INDEX(usbp->setup) != msdp->msd_interface_number)
        return FALSE;
    } else if (MSD_SETUP_INDEX(usbp->setup) != 0 ) {
        return FALSE;
    }

    /* act depending on bRequest = setup[1] */
    switch (usbp->setup[1]) {
      case MSD_REQ_RESET:
        /* check that it is a HOST2DEV request */
        if (((usbp->setup[0] & USB_RTYPE_DIR_MASK) != USB_RTYPE_DIR_HOST2DEV)
            || (MSD_SETUP_LENGTH(usbp->setup) != 0)
            || (MSD_SETUP_VALUE(usbp->setup) != 0))
          return FALSE;

        /* reset all endpoints */
        /* TODO!*/
        /* The device shall NAK the status stage of the device request until
         * the Bulk-Only Mass Storage Reset is complete.
         */
        return TRUE;
      case MSD_GET_MAX_LUN:
        /* check that it is a DEV2HOST request */
        if (((usbp->setup[0] & USB_RTYPE_DIR_MASK) != USB_RTYPE_DIR_DEV2HOST)
            || (MSD_SETUP_LENGTH(usbp->setup) != 1)
            || (MSD_SETUP_VALUE(usbp->setup) != 0))
          return FALSE;

        static uint8_t len_buf[1] = {0};
        usbSetupTransfer(usbp, len_buf, 1, NULL);
        return TRUE;
      default:
        return FALSE;
        break;
    }
  }
  return FALSE;
}


const char* usb_msd_driver_state_t_to_str(const usb_msd_driver_state_t driver_state) {
  switch (driver_state) {
    case USB_MSD_DRIVER_UNINITIALIZED:
      return ("USB_MSD_DRIVER_UNINITIALIZED");
    case USB_MSD_DRIVER_ERROR:
      return ("USB_MSD_DRIVER_ERROR");
    case USB_MSD_DRIVER_OK:
      return ("USB_MSD_DRIVER_OK");
    case USB_MSD_DRIVER_ERROR_BLK_DEV_NOT_READY:
      return ("USB_MSD_DRIVER_ERROR_BLK_DEV_NOT_READY");
  }
  return ("USB_MSD_DRIVER_UNKNOWN");
}


/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/


/* Event Flow Functions */


static uint8_t WaitForISR(USBMassStorageDriver *msdp, const bool_t check_reset, const msd_wait_mode_t wait_mode) {
  uint8_t ret = WAIT_ISR_SUCCESS;
  /* sleep until it completes */
  chSysLock();
#if 1

  msd_debug_print(msdp->chp, "WaitISR(mode=%d)\r\n", wait_mode);
  for (;;) {
    const msg_t m = chBSemWaitTimeoutS(&msdp->bsem, 1);
    if (m == RDY_OK) {
      break;
    }

    if( wait_mode == MSD_WAIT_MODE_BULK_IN ) {
      if( msdp->wait_bulk_in_isr_counter != 0 ) {
        break;
      }
    } else if( wait_mode == MSD_WAIT_MODE_BULK_OUT ) {
      if( msdp->wait_bulk_out_isr_counter != 0 ) {
        break;
      }
    }

    if (msdp->reconfigured_or_reset_event) {
      ret = WAIT_ISR_BUSS_RESET_OR_RECONNECT;
      break;
    }
  }

#else
  if (check_reset) {
    for (;;) {
      const msg_t m = chBSemWaitTimeoutS(&msdp->bsem, 1);
      if (m == RDY_OK) {
        break;
      }

      if (msdp->reconfigured_or_reset_event) {
        ret = WAIT_ISR_BUSS_RESET_OR_RECONNECT;
        break;
      }
    }
  } else {
    chBSemWaitS(&msdp->bsem);
  }
#endif


  chSysUnlock();
  return (ret);
}


static void WaitForUSBTransferComplete(USBMassStorageDriver *msdp,
                                       const int ping_pong_buffer_index) {
  while (TRUE) {
    chBSemWaitTimeout(&msdp->mass_sorage_thd_bsem, MS2ST(1));

    if (rw_ping_pong_buffer[ping_pong_buffer_index].is_transfer_done) {
      break;
    } else {
      chThdSleepMilliseconds(1);
    }
  }
}



/* SCSI Functions */


static inline void SCSISetSense(USBMassStorageDriver *msdp, uint8_t key,
                                uint8_t acode, uint8_t aqual) {
  msdp->sense.byte[2] = key;
  msdp->sense.byte[12] = acode;
  msdp->sense.byte[13] = aqual;
}


static void msdSetDefaultSenseKey(USBMassStorageDriver *msdp) {
  SCSISetSense(msdp, SCSI_SENSE_KEY_GOOD,
             SCSI_ASENSE_NO_ADDITIONAL_INFORMATION,
             SCSI_ASENSEQ_NO_QUALIFIER);
}

static msd_wait_mode_t SCSICommandInquiry(USBMassStorageDriver *msdp) {
  msd_cbw_t *cbw = &(msdp->cbw);

  static const scsi_inquiry_response_t inquiry =
      {0x00, /* peripheral, direct access block device */
       0x80, /* removable */
       0x04, /* version, SPC-2 */
       0x02, /* response data format */
       0x20, /* additional_length, response has 0x20 + 4 bytes */
       0x00, /* sccstp*/
       0x00, /* bqueetc*/
       0x00, /* cmdqueue*/
       "Chibios",
       "Mass Storage",
       {'v', CH_KERNEL_MAJOR + '0', '.', CH_KERNEL_MINOR + '0'}, };

  if ((cbw->scsi_cmd_data[1] & ((1 << 0) | (1 << 1))) || cbw->scsi_cmd_data[2]) {
    /* Optional but unsupported bits set - update the SENSE key and fail
     * the request */
    msd_debug_err_print(msdp->chp, " INQ ERR 0x%X 0x%X\r\n", cbw->scsi_cmd_data[1], cbw->scsi_cmd_data[2]);
    SCSISetSense(msdp, SCSI_SENSE_KEY_ILLEGAL_REQUEST,
                 SCSI_ASENSE_INVALID_FIELD_IN_CDB, SCSI_ASENSEQ_NO_QUALIFIER);

    //we should indicate that the command failed to the host, but still return data
    msdp->command_succeeded_flag = false;
  }

  usbPrepareTransmit(msdp->usbp, msdp->ms_ep_number, (uint8_t *)&inquiry,
                     sizeof(scsi_inquiry_response_t));

  chSysLock();
  msdp->wait_bulk_in_isr_counter = 0;
  usbStartTransmitI(msdp->usbp, msdp->ms_ep_number);
  chSysUnlock();

  msdp->result_unused = TRUE;

  /* wait for ISR */
  return MSD_WAIT_MODE_BULK_IN;
}

static msd_wait_mode_t SCSICommandRequestSense(USBMassStorageDriver *msdp) {
  //This command should not affect the sense key
  msdp->set_default_sense_key_unused = false;

  usbPrepareTransmit(msdp->usbp, msdp->ms_ep_number, (uint8_t *)&msdp->sense,
                     sizeof(scsi_sense_response_t));
  chSysLock();
  msdp->wait_bulk_in_isr_counter = 0;
  usbStartTransmitI(msdp->usbp, msdp->ms_ep_number);
  chSysUnlock();

  msdp->result_unused = TRUE;

  /* wait for ISR */
  return MSD_WAIT_MODE_BULK_IN;
}

static msd_wait_mode_t SCSICommandReadFormatCapacity(USBMassStorageDriver *msdp) {
  static SCSIReadFormatCapacityResponse_t response;

  msdSetDefaultSenseKey(msdp);

  const uint32_t formated_capactiy_descriptor_code = 0x02;//see usbmass-ufi10.doc for codes

  response.payload_byte_length[3] = 8;
  response.last_block_addr = swap_uint32(msdp->block_dev_info.blk_num - 1);
  response.block_size = swap_uint32(msdp->block_dev_info.blk_size) | formated_capactiy_descriptor_code;

  usbPrepareTransmit(msdp->usbp, msdp->ms_ep_number, (uint8_t *)&response,
                     sizeof(response));

  chSysLock();
  msdp->wait_bulk_in_isr_counter = 0;
  usbStartTransmitI(msdp->usbp, msdp->ms_ep_number);
  chSysUnlock();

  msdp->result_unused = TRUE;

  /* wait for ISR */
  return MSD_WAIT_MODE_BULK_IN;
}

static msd_wait_mode_t SCSICommandReadCapacity10(USBMassStorageDriver *msdp) {
  static SCSIReadCapacity10Response_t response;

  msdSetDefaultSenseKey(msdp);

  response.block_size = swap_uint32(msdp->block_dev_info.blk_size);
  response.last_block_addr = swap_uint32(msdp->block_dev_info.blk_num - 1);

  usbPrepareTransmit(msdp->usbp, msdp->ms_ep_number, (uint8_t *)&response,
                     sizeof(response));

  chSysLock();
  msdp->wait_bulk_in_isr_counter = 0;
  usbStartTransmitI(msdp->usbp, msdp->ms_ep_number);
  chSysUnlock();

  msdp->result_unused = TRUE;

  /* wait for ISR */
  return MSD_WAIT_MODE_BULK_IN;
}

static msd_wait_mode_t SCSICommandSendDiagnostic(USBMassStorageDriver *msdp) {
  msd_cbw_t *cbw = &(msdp->cbw);

  if ((!cbw->scsi_cmd_data[1]) & (1 << 2)) {
    /* Only self-test supported - update SENSE key and fail the command */
    SCSISetSense(msdp, SCSI_SENSE_KEY_ILLEGAL_REQUEST,
                 SCSI_ASENSE_INVALID_FIELD_IN_CDB, SCSI_ASENSEQ_NO_QUALIFIER);

    return FALSE;
  }

  /* TODO: actually perform the test */
  msdp->result_unused = TRUE;

  /* don't wait for ISR */
  return MSD_WAIT_MODE_NONE;
}

static void SCSIWriteTransferPingPong(USBMassStorageDriver *msdp,
                                      volatile rw_usb_sd_buffer_t *dest_buffer) {
  int cnt;
  dest_buffer->is_transfer_done = FALSE;
  dest_buffer->num_blocks_to_write = 0;

#if MSD_ENABLE_PERF_DEBUG_GPIOS
  palSetPad(GPIOH, GPIOH_LED2);
#endif
  for (cnt = 0;
      cnt < BLOCK_WRITE_ITTERATION_COUNT
          && cnt < dest_buffer->max_blocks_to_read; cnt++) {
    usbPrepareReceive(msdp->usbp, msdp->ms_ep_number,
                      (uint8_t*)&dest_buffer->buf[cnt * BLOCK_SIZE_INCREMENT],
                      (msdp->block_dev_info.blk_size));

    chSysLock();
    msdp->wait_bulk_out_isr_counter = 0;
    usbStartReceiveI(msdp->usbp, msdp->ms_ep_number);
    chSysUnlock();

    WaitForISR(msdp, FALSE, MSD_WAIT_MODE_BULK_OUT);
    dest_buffer->num_blocks_to_write++;
  }
  dest_buffer->is_transfer_done = TRUE;
#if MSD_ENABLE_PERF_DEBUG_GPIOS
  palClearPad(GPIOH, GPIOH_LED2);
#endif
}


static msd_wait_mode_t SCSICommandStartReadWrite10(USBMassStorageDriver *msdp) {
  msd_cbw_t *cbw = &(msdp->cbw);
  int read_success;
  int retry_count;

  msdSetDefaultSenseKey(msdp);

  if ((cbw->scsi_cmd_data[0] == SCSI_CMD_WRITE_10) && blkIsWriteProtected(msdp->bbdp)) {
    msd_debug_err_print(msdp->chp, "\r\nWrite Protected!\r\n");
    /* device is write protected and a write has been issued */
    /* Block address is invalid, update SENSE key and return command fail */
    SCSISetSense(msdp, SCSI_SENSE_KEY_DATA_PROTECT, SCSI_ASENSE_WRITE_PROTECTED,
                 SCSI_ASENSEQ_NO_QUALIFIER);

    msdp->result_unused = FALSE;
    msdp->command_succeeded_flag = false;
    msdp->set_default_sense_key_unused = false;
    msdp->stall_in_endpoint = true;
    return MSD_WAIT_MODE_NONE;
  }

  uint32_t rw_block_address = swap_4byte_buffer(&cbw->scsi_cmd_data[2]);
  const uint16_t total_blocks = swap_2byte_buffer(&cbw->scsi_cmd_data[7]);
  //uint32_t rw_block_address = swap_uint32(*(uint32_t *)&cbw->scsi_cmd_data[2]);
  //const uint16_t total_blocks = swap_uint16(*(uint16_t *)&cbw->scsi_cmd_data[7]);
  uint16_t i = 0;

  if (rw_block_address >= msdp->block_dev_info.blk_num) {
    msd_debug_err_print(msdp->chp, "\r\nBlock Address too large %u > %u\r\n", rw_block_address, msdp->block_dev_info.blk_num);
    /* Block address is invalid, update SENSE key and return command fail */
    SCSISetSense(msdp, SCSI_SENSE_KEY_DATA_PROTECT, SCSI_ASENSE_WRITE_PROTECTED,
                 SCSI_ASENSEQ_NO_QUALIFIER);

    msdp->result_unused = FALSE;
    msdp->command_succeeded_flag = false;
    msdp->set_default_sense_key_unused = false;
    msdp->stall_in_endpoint = true;

    /* don't wait for ISR */
    return MSD_WAIT_MODE_NONE;
  }

  for (i = 0; i < 2; i++) {
    /*initialized ping pong buffer*/
    rw_ping_pong_buffer[i].max_blocks_to_read = 0;
    rw_ping_pong_buffer[i].num_blocks_to_write = 0;
    rw_ping_pong_buffer[i].is_transfer_done = FALSE;
  }

  if (cbw->scsi_cmd_data[0] == SCSI_CMD_WRITE_10) {
    /* loop over each block */

    int ping_pong_buffer_index = 0;
    /*initiate a transfer*/
    rw_ping_pong_buffer[ping_pong_buffer_index].is_transfer_done = FALSE;
    rw_ping_pong_buffer[ping_pong_buffer_index].max_blocks_to_read =
        total_blocks;

    /*Trigger the transfer in the other thread*/
    msdp->trigger_transfer_index = ping_pong_buffer_index;

    /*wake other thread on semaphore to trigger the transfer*/
    chBSemSignal(&msdp->usb_transfer_thread_bsem);

    WaitForUSBTransferComplete(msdp, ping_pong_buffer_index);

    for (i = 0; i < total_blocks;) {
      const int done_buffer_index = ping_pong_buffer_index;
      const int empty_buffer_index = ((ping_pong_buffer_index + 1) % 2);

      /*initiate another transfer in the other ping pong buffer*/
      int queue_another_transfer = FALSE;
      if ((i + BLOCK_WRITE_ITTERATION_COUNT) < total_blocks) {
        queue_another_transfer = TRUE;
      }

      if (queue_another_transfer) {
        while (TRUE) {
          if (msdp->trigger_transfer_index == UINT32_MAX) {
            rw_ping_pong_buffer[empty_buffer_index].max_blocks_to_read =
                total_blocks - i - BLOCK_WRITE_ITTERATION_COUNT;

            msdp->trigger_transfer_index = empty_buffer_index;

            /*wake other thread on semaphore to trigger the transfer*/
            chBSemSignal(&msdp->usb_transfer_thread_bsem);
            break;
          } else {
            chThdSleepMilliseconds(1);
          }
        }
      }

      if (rw_ping_pong_buffer[done_buffer_index].num_blocks_to_write <= 0) {
        /*This should never happen!!! Something is seriously wrong!*/
        msd_debug_err_print(
              msdp->chp, "\r\nCant write 0 blocks, this should not happen, halting\r\n");
        chThdSleepMilliseconds(50);
        chSysHalt();
      }

      /* now write the block to the block device */
      if (blkWrite(msdp->bbdp, rw_block_address,
          (uint8_t*)rw_ping_pong_buffer[done_buffer_index].buf,
          rw_ping_pong_buffer[done_buffer_index].num_blocks_to_write)
          == CH_FAILED) {
        msd_debug_err_print(msdp->chp, "\r\nSD Block Write Error, halting\r\n");

        chThdSleepMilliseconds(50);
        msdp->write_error_count++;

        msdp->result_unused = FALSE;
        msdp->set_default_sense_key_unused = false;
        msdp->command_succeeded_flag = false;
        msdp->stall_out_endpoint = true;
        SCSISetSense(msdp, SCSI_SENSE_KEY_MEDIUM_ERROR,
                     SCSI_ASENSE_NO_ADDITIONAL_INFORMATION,
                     SCSI_ASENSEQ_NO_QUALIFIER);

        if (queue_another_transfer) {
          /*Let the previous queued transfer finish and ignore it.*/
          WaitForUSBTransferComplete(msdp, empty_buffer_index);
        }

        return (MSD_WAIT_MODE_NONE);
      }
      rw_block_address +=
          rw_ping_pong_buffer[done_buffer_index].num_blocks_to_write;
      i += rw_ping_pong_buffer[done_buffer_index].num_blocks_to_write;
      rw_ping_pong_buffer[done_buffer_index].is_transfer_done = FALSE;
      rw_ping_pong_buffer[done_buffer_index].num_blocks_to_write = 0;

      if (queue_another_transfer) {
        WaitForUSBTransferComplete(msdp, empty_buffer_index);
      }

      /*Swap the ping pong buffers*/
      ping_pong_buffer_index = empty_buffer_index;
    }

  } else {
    i = 0;
    /* read the first block from block device */
    read_success = FALSE;
    for (retry_count = 0; retry_count < 3; retry_count++) {
      if (blkRead(msdp->bbdp, rw_block_address, read_buffer[i % 2], 1)
          == CH_FAILED) {
        msd_debug_err_print(msdp->chp, "\r\nSD Block Read Error\r\n");
        msdp->read_error_count++;
      } else {
        read_success = TRUE;
        break;
      }
    }

    static bool_t did_one_error_flag = false;
    bool_t force_error_flag = palReadPad(GPIOI, GPIOI_PIN6);


    if ((!read_success) || (!did_one_error_flag && force_error_flag) ) {
      did_one_error_flag = true;

      msd_debug_err_print(msdp->chp, "\r\nSD Block Read Error 1, breaking read sequence\r\n");

      /*wait for printing to finish*/
      chThdSleepMilliseconds(10);

      msdp->result_unused = FALSE;
      msdp->set_default_sense_key_unused = false;
      msdp->command_succeeded_flag = false;
      msdp->stall_in_endpoint = true;

      msd_debug_err_print(
            msdp->chp, "\r\nSetting sense code %u\r\n", SCSI_SENSE_KEY_MEDIUM_ERROR);

      SCSISetSense(msdp, SCSI_SENSE_KEY_MEDIUM_ERROR,
                   SCSI_ASENSE_NO_ADDITIONAL_INFORMATION,
                   SCSI_ASENSEQ_NO_QUALIFIER);

      return MSD_WAIT_MODE_NONE;
    }

    rw_block_address++;

    /* loop over each block */
    for (i = 0; i < total_blocks; i++) {
      /* transmit the block */
      while (usbGetTransmitStatusI(msdp->usbp, msdp->ms_ep_number)) {
          //wait for the prior transmit to complete
          //chThdSleepMicroseconds(500);
      }
      usbPrepareTransmit(msdp->usbp, msdp->ms_ep_number, read_buffer[i % 2],
                         msdp->block_dev_info.blk_size);

      chSysLock();
      msdp->wait_bulk_in_isr_counter = 0;
      usbStartTransmitI(msdp->usbp, msdp->ms_ep_number);
      chSysUnlock();

      if (i < (total_blocks - 1)) {
        /* there is at least one more block to be read from device */
        /* so read that while the USB transfer takes place */
        read_success = FALSE;
        for (retry_count = 0; retry_count < 3; retry_count++) {
          if (blkRead(msdp->bbdp, rw_block_address, read_buffer[(i+1) % 2], 1)
              == CH_FAILED) {
            msd_debug_err_print(msdp->chp, "\r\nSD Block Read Error 2\r\n");

            msdp->read_error_count++;
          } else {
            read_success = TRUE;
            break;
          }
        }

        if ((!read_success)) {
          msd_debug_err_print(
                msdp->chp, "\r\nSD Block Read Error 22, addr=%d, halting\r\n", rw_block_address);

          /*wait for printing to finish*/
          chThdSleepMilliseconds(70);

          msdp->result_unused = FALSE;
          msdp->set_default_sense_key_unused = false;
          msdp->command_succeeded_flag = false;
          msdp->stall_in_endpoint = true;

          msd_debug_err_print(
                msdp->chp, "\r\nSetting sense code %u\r\n", SCSI_SENSE_KEY_MEDIUM_ERROR);

          SCSISetSense(msdp, SCSI_SENSE_KEY_MEDIUM_ERROR,
                       SCSI_ASENSE_NO_ADDITIONAL_INFORMATION,
                       SCSI_ASENSEQ_NO_QUALIFIER);
          return MSD_WAIT_MODE_NONE;
        }



        rw_block_address++;


      }

      /*FIXME In the event that the USB connection is unplugged while we're waiting for a bulk
       * endpoint ISR, this will never return, and when re-plugged into the host, the drive will
       * not show back up on the host. We need a way to break out of this loop when disconnected from the bus.
       */

      if (WaitForISR(msdp, TRUE, MSD_WAIT_MODE_NONE) == WAIT_ISR_BUSS_RESET_OR_RECONNECT) {
        //fixme are we handling the reset case properly
        msdp->result_unused = FALSE;
        return MSD_WAIT_MODE_NONE;
      }

    }
  }

  msdp->result_unused = TRUE;

  /* don't wait for ISR */
  return MSD_WAIT_MODE_NONE;
}

static msd_wait_mode_t SCSICommandStartStopUnit(USBMassStorageDriver *msdp) {
  SCSIStartStopUnitRequest_t *ssu =
      (SCSIStartStopUnitRequest_t *)&(msdp->cbw.scsi_cmd_data);

  if ((ssu->loej_start & 0b00000011) == 0b00000010) {
    /* device has been ejected */
    if (!msdp->disable_usb_bus_disconnect_on_eject) {
      chEvtBroadcast(&msdp->evt_ejected);
      msdp->state = MSD_STATE_EJECTED;
    }
  }

  msdp->result_unused = TRUE;

  /* don't wait for ISR */
  return MSD_WAIT_MODE_NONE;
}

static msd_wait_mode_t SCSICommandPreventAllowMediumRemovial(USBMassStorageDriver *msdp) {
  msd_cbw_t *cbw = &(msdp->cbw);

  if( (cbw->scsi_cmd_data[4] & 0x01) ) {
    //prohibit media removal
    if( msdp->enable_media_removial ) {
      //this can have positive performance
      msdp->command_succeeded_flag = false;
    }
  }

  /* don't wait for ISR */
  return MSD_WAIT_MODE_NONE;
}

static msd_wait_mode_t SCSICommandModeSense6(USBMassStorageDriver *msdp) {
  /* Send an empty header response with the Write Protect flag status */
  /* TODO set byte3 to 0x80 if disk is read only */
  static uint8_t response[4] = {0x00, 0x00, 0x00, 0x00};

  usbPrepareTransmit(msdp->usbp, msdp->ms_ep_number, response, 4);

  chSysLock();
  msdp->wait_bulk_in_isr_counter = 0;
  usbStartTransmitI(msdp->usbp, msdp->ms_ep_number);
  chSysUnlock();

  msdp->result_unused = TRUE;

  /* wait for ISR */
  return MSD_WAIT_MODE_BULK_IN;
}

static msd_wait_mode_t msdWaitForCommandBlock(USBMassStorageDriver *msdp) {
  usbPrepareReceive(msdp->usbp, msdp->ms_ep_number, (uint8_t *)&msdp->cbw,
                    sizeof(msd_cbw_t));

  chSysLock();
  msdp->wait_bulk_out_isr_counter = 0;
  usbStartReceiveI(msdp->usbp, msdp->ms_ep_number);
  chSysUnlock();

  msdp->state = MSD_STATE_READ_CMD_BLOCK;

  return(MSD_WAIT_MODE_BULK_OUT);/* wait for ISR */
}

/*  */
/**
 * @brief   A command block has been received

 *
 * @param[in] p1        description of parameter one
 * @param[out] p2       description of parameter two
 * @param[in,out] p3    description of parameter three
 * @return              Description of the returned value, must be omitted if
 *                      a function returns void.
 * @retval TRUE         On success
 * @retval FALSE        On failure
 *
 */
static msd_wait_mode_t msdProcessCommandBlock(USBMassStorageDriver *msdp) {
  msd_cbw_t *cbw = &(msdp->cbw);

  /* by default transition back to the idle state */
  msdp->state = MSD_STATE_IDLE;

  /* check the command */
  if ((cbw->signature != MSD_CBW_SIGNATURE) || (cbw->lun > 0)
      || ((cbw->data_len > 0) && (cbw->flags & 0x1F))
      || (cbw->scsi_cmd_len == 0) || (cbw->scsi_cmd_len > 16)) {

    msd_debug_err_print(msdp->chp, "Bad CB\r\n");
    /* stall both IN and OUT endpoints */
    chSysLock();
    usbStallReceiveI(msdp->usbp, msdp->ms_ep_number);
    //usbStallTransmitI(msdp->usbp, msdp->ms_ep_number);
    chSysUnlock();

    /* don't wait for ISR */
    return MSD_WAIT_MODE_NONE;
  }

  msd_debug_print(msdp->chp, " CMD 0x%X\r\n", cbw->scsi_cmd_data[0]);
  msdp->command_succeeded_flag = true;
  msdp->stall_in_endpoint = false;
  msdp->stall_out_endpoint = false;
  msdp->set_default_sense_key_unused = false;

  msd_wait_mode_t wait_mode = MSD_WAIT_MODE_NONE;
  switch (cbw->scsi_cmd_data[0]) {
    case SCSI_CMD_INQUIRY:
      msd_debug_print(msdp->chp, "CMD_INQ\r\n");
      wait_mode = SCSICommandInquiry(msdp);
      break;
    case SCSI_CMD_REQUEST_SENSE:
      msd_debug_print(msdp->chp, "\r\nCMD_RS\r\n");
      wait_mode = SCSICommandRequestSense(msdp);
      break;
    case SCSI_CMD_READ_FORMAT_CAPACITY:
      msd_debug_print(msdp->chp, "CMD_RFC\r\n");
      wait_mode = SCSICommandReadFormatCapacity(msdp);
      break;
    case SCSI_CMD_READ_CAPACITY_10:
      msd_debug_print(msdp->chp, "CMD_RC10\r\n");
      wait_mode = SCSICommandReadCapacity10(msdp);
      break;
    case SCSI_CMD_READ_10:
    case SCSI_CMD_WRITE_10:
      msd_debug_print(msdp->chp, "CMD_RW\r\n");
      MSD_RW_LED_ON();
      wait_mode = SCSICommandStartReadWrite10(msdp);
      MSD_RW_LED_OFF();
      break;
    case SCSI_CMD_SEND_DIAGNOSTIC:
      msd_debug_print(msdp->chp, "CMD_DIA\r\n");
      wait_mode = SCSICommandSendDiagnostic(msdp);
      break;
    case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
      msd_debug_print(msdp->chp, "CMD_PAMR\r\n");
      wait_mode = SCSICommandPreventAllowMediumRemovial(msdp);
      break;
    case SCSI_CMD_TEST_UNIT_READY:
    case SCSI_CMD_VERIFY_10:
      msd_debug_print(msdp->chp, "CMD_00_1E_2F\r\n");
      /* don't handle */
      msdp->result_unused = TRUE;
      break;
    case SCSI_CMD_MODE_SENSE_6:
      msd_debug_print(msdp->chp, "\r\nCMD_S6\r\n");
      wait_mode = SCSICommandModeSense6(msdp);
      break;
    case SCSI_CMD_START_STOP_UNIT:
      msd_debug_print(msdp->chp, "CMD_STOP\r\n");
      wait_mode = SCSICommandStartStopUnit(msdp);
      break;
    default:
      msd_debug_err_print(msdp->chp, "CMD default 0x%X\r\n", cbw->scsi_cmd_data[0]);
      msdp->set_default_sense_key_unused = false;
      SCSISetSense(msdp, SCSI_SENSE_KEY_ILLEGAL_REQUEST,
                   SCSI_ASENSE_INVALID_COMMAND, SCSI_ASENSEQ_NO_QUALIFIER);

      /* stall IN endpoint */
      chSysLock()
      msdp->wait_bulk_in_isr_counter = 0;
      usbStallTransmitI(msdp->usbp, msdp->ms_ep_number);
      chSysUnlock()

      WaitForISR(msdp, TRUE, MSD_WAIT_MODE_BULK_IN);

      cbw->data_len = 0;
      return MSD_WAIT_MODE_NONE;
  }

  cbw->data_len = 0;

#if 0
  if (msdp->result) {
    /* update sense with success status */
    msdSetDefaultSenseKey(msdp);
  } else {
    msd_debug_err_print(msdp->chp, "stalling IN endpoint\r\n");
    /* stall IN endpoint */
    chSysLock();
    usbStallTransmitI(msdp->usbp, msdp->ms_ep_number);
    chSysUnlock();

    cbw->data_len = 0;
    return MSD_WAIT_MODE_NONE;
  }
#endif

#if 0
  if( msdp->set_default_sense_key ) {
    /* update sense with success status */
    msdSetDefaultSenseKey(msdp);
  }
#endif

  if( msdp->stall_in_endpoint || msdp->stall_out_endpoint ) {
    /* stall IN endpoint */
    chSysLock();
    if( msdp->stall_in_endpoint ) {
      msd_debug_err_print(msdp->chp, "stalling IN endpoint\r\n");
      msdp->wait_bulk_in_isr_counter;
      usbStallTransmitI(msdp->usbp, msdp->ms_ep_number);
    }
    if( msdp->stall_out_endpoint ) {
      msd_debug_err_print(msdp->chp, "stalling OUT endpoint\r\n");
      usbStallReceiveI(msdp->usbp, msdp->ms_ep_number);
    }
    chSysUnlock();

    if( msdp->stall_in_endpoint ) {
      WaitForISR(msdp, TRUE, MSD_WAIT_MODE_BULK_IN);
    }

    cbw->data_len = 0;
  }

  if (wait_mode != MSD_WAIT_MODE_NONE ) {
    if (WaitForISR(msdp, TRUE, wait_mode) == WAIT_ISR_BUSS_RESET_OR_RECONNECT) {
      return (MSD_WAIT_MODE_NONE);
    }
  }

  msd_csw_t *csw = &(msdp->csw);

#if 0
  if ((!msdp->result) && cbw->data_len) {
    msd_debug_err_print(msdp->chp, "stalling in and out endpoints\r\n");
    /* still bytes left to send, this is too early to send CSW? */
    chSysLock();
    usbStallReceiveI(msdp->usbp, msdp->ms_ep_number);
    usbStallTransmitI(msdp->usbp, msdp->ms_ep_number);
    chSysUnlock();

    return MSD_WAIT_MODE_NONE;
  }
#endif

  csw->status = (msdp->command_succeeded_flag) ? MSD_COMMAND_PASSED : MSD_COMMAND_FAILED;
  csw->signature = MSD_CSW_SIGNATURE;
  csw->data_residue = cbw->data_len;
  csw->tag = cbw->tag;

  usbPrepareTransmit(msdp->usbp, msdp->ms_ep_number, (uint8_t *)csw,
                     sizeof(msd_csw_t));

  chSysLock();
  //msdp->wait_bulk_out_isr_counter = 0;
  //msdWaitForCommandBlock(msdp);

  msdp->wait_bulk_in_isr_counter = 0;
  usbStartTransmitI(msdp->usbp, msdp->ms_ep_number);
  chSysUnlock();

  //WaitForISR(msdp, TRUE, MSD_WAIT_MODE_BULK_IN);//wait for our status to be sent back

  /* wait on ISR */
  return MSD_WAIT_MODE_BULK_IN;
  //return MSD_WAIT_MODE_BULK_OUT;
}





/*===========================================================================*/
/* Threads                                                   */
/*===========================================================================*/

/**
 * @brief   This thread is responsible for triggering a USB write of date
 *          from the MCU to the host. It is run as a separate thread to allow
 *          for concurrent RXing of data and writing of data to the SD card to
 *          thus significantly improve performance.
 *
 * @param[in] arg       pointer to the @p USBMassStorageDriver object
 *
 * @special
 */
static msg_t MassStorageUSBTransferThd(void *arg) {
  USBMassStorageDriver *msdp = (USBMassStorageDriver *)arg;

  chRegSetThreadName("USB-MSD-Transfer");

  for (;;) {
    if (msdp->trigger_transfer_index != UINT32_MAX) {
      SCSIWriteTransferPingPong(
          msdp, &rw_ping_pong_buffer[msdp->trigger_transfer_index]);
      msdp->trigger_transfer_index = UINT32_MAX;
      /*notify other thread*/
      chBSemSignal(&msdp->mass_sorage_thd_bsem);
    }

    chBSemWaitTimeout(&msdp->usb_transfer_thread_bsem, MS2ST(1));
  }

  return (0);
}



static msg_t MassStorageThd(void *arg) {
  USBMassStorageDriver *msdp = (USBMassStorageDriver *)arg;
  chRegSetThreadName("USB-MSD");

  msd_wait_mode_t wait_for_isr = MSD_WAIT_MODE_NONE;

  /* wait for the usb to be initialized */
  msd_debug_print(msdp->chp, "Y");
  WaitForISR(msdp, FALSE, MSD_WAIT_MODE_NONE);
  msd_debug_print(msdp->chp, "y");

  while (TRUE) {
    wait_for_isr = MSD_WAIT_MODE_NONE;

    if (msdp->reconfigured_or_reset_event) {
      /*If the devices is unplugged and re-plugged but did not have a CPU reset,
       * we must set the state back to idle.*/
      msdp->reconfigured_or_reset_event = FALSE;
      msdp->state = MSD_STATE_IDLE;

      msdSetDefaultSenseKey(msdp);
    }

    bool_t enable_msd = true;
    if (msdp->enable_msd_callback != NULL) {
      enable_msd = msdp->enable_msd_callback();
    }

    if( msdp->driver_state != USB_MSD_DRIVER_OK ) {
      enable_msd = false;
    }

    if (enable_msd ) {
      msd_debug_print(msdp->chp, "state=%d\r\n", msdp->state);
      /* wait on data depending on the current state */
      switch (msdp->state) {
        case MSD_STATE_IDLE:
          msd_debug_print(msdp->chp, "IDL");
          wait_for_isr = msdWaitForCommandBlock(msdp);
          msd_debug_print(msdp->chp, "x\r\n");
          break;
        case MSD_STATE_READ_CMD_BLOCK:
          msd_debug_print(msdp->chp, "RCB");
          wait_for_isr = msdProcessCommandBlock(msdp);
          msd_debug_print(msdp->chp, "x\r\n");
          break;
        case MSD_STATE_EJECTED:
          /* disconnect usb device */
          msd_debug_print(msdp->chp, "ejected\r\n");
          if (!msdp->disable_usb_bus_disconnect_on_eject) {
            chThdSleepMilliseconds(70);
            usbDisconnectBus(msdp->usbp);
            usbStop(msdp->usbp);
            chThdExit(0);
          }
          return 0;
      }
    }

   if (wait_for_isr && (!msdp->reconfigured_or_reset_event)) {
      /* wait until the ISR wakes thread */
      msd_debug_print(msdp->chp, "W%d,%d", wait_for_isr, msdp->state);
      WaitForISR(msdp, TRUE, wait_for_isr);
      msd_debug_print(msdp->chp, "w\r\n");
    }
  }

  return 0;
}





#endif /* HAL_USE_MASS_STORAGE_USB */
