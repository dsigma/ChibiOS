/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2007        */
/*-----------------------------------------------------------------------*/
/* This is a stub disk I/O module that acts as front end of the existing */
/* disk I/O modules and attach it to FatFs module with common interface. */
/*-----------------------------------------------------------------------*/

#include "ch.h"
#include "hal.h"
#include "ffconf.h"
#include "diskio.h"

#define ENABLE_SDC_READ_WRITE_RETRYS    1

#if HAL_USE_MMC_SPI && HAL_USE_SDC
#error "cannot specify both MMC_SPI and SDC drivers"
#endif

#if HAL_USE_MMC_SPI
extern MMCDriver MMCD1;
#elif HAL_USE_SDC
extern SDCDriver SDCD1;
#else
#error "MMC_SPI or SDC driver must be specified"
#endif

#if HAL_USE_RTC
#include "chrtclib.h"
extern RTCDriver RTCD1;
#endif

/*-----------------------------------------------------------------------*/
/* Correspondence between physical drive number and physical drive.      */

#define MMC     0
#define SDC     0

uint32_t mmc_is_in_sleep_mode = 0;

bool disk_wake(SDCDriver *sdcp) {
  bool ret = true;
  if( mmc_is_in_sleep_mode ) {
    //Wake the EMMC
    //See section 6.6.24 of JESD84-B50
    uint32_t resp[1];
    if (sdc_lld_send_cmd_short(sdcp, MMCSD_CMD_SLEEP_AWAKE, 0x0, resp)) {
      ret = false;
    } else {
      //EMMC will pull down DAT0 while waking, wait for it to go high
    }
    mmc_is_in_sleep_mode = 0;
  }

  return(ret);
}

bool disk_sleep(SDCDriver *sdcp) {
  bool ret = true;
  if( ! mmc_is_in_sleep_mode ) {
    //See section 6.6.24 of JESD84-B50
    uint32_t resp[1];
    if (sdc_lld_send_cmd_short(sdcp, MMCSD_CMD_SLEEP_AWAKE, (1<<15), resp)) {
      ret = false;
    } else {
      //EMMC will pull down DAT0 while going to sleep, wait for it to go high
    }

    mmc_is_in_sleep_mode = 1;
  }
  return(ret);
}

/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */

DSTATUS disk_initialize (
    BYTE drv                /* Physical drive nmuber (0..) */
)
{
  DSTATUS stat;



  switch (drv) {
#if HAL_USE_MMC_SPI
  case MMC:
    stat = 0;
    /* It is initialized externally, just reads the status.*/
    if (blkGetDriverState(&MMCD1) != BLK_READY)
      stat |= STA_NOINIT;
    if (mmcIsWriteProtected(&MMCD1))
      stat |=  STA_PROTECT;
    return stat;
#else
  case SDC:
    disk_wake(&SDCD1);
    stat = 0;
    /* It is initialized externally, just reads the status.*/
    if (blkGetDriverState(&SDCD1) != BLK_READY)
      stat |= STA_NOINIT;
    if (sdcIsWriteProtected(&SDCD1))
      stat |=  STA_PROTECT;
    return stat;
#endif
  }
  return STA_NODISK;
}



/*-----------------------------------------------------------------------*/
/* Return Disk Status                                                    */

DSTATUS disk_status (
    BYTE drv        /* Physical drive nmuber (0..) */
)
{
  DSTATUS stat;

  switch (drv) {
#if HAL_USE_MMC_SPI
  case MMC:
    stat = 0;
    /* It is initialized externally, just reads the status.*/
    if (blkGetDriverState(&MMCD1) != BLK_READY)
      stat |= STA_NOINIT;
    if (mmcIsWriteProtected(&MMCD1))
      stat |= STA_PROTECT;
    return stat;
#else
  case SDC:
    disk_wake(&SDCD1);
    stat = 0;
    /* It is initialized externally, just reads the status.*/
    if (blkGetDriverState(&SDCD1) != BLK_READY)
      stat |= STA_NOINIT;
    if (sdcIsWriteProtected(&SDCD1))
      stat |= STA_PROTECT;
    return stat;
#endif
  }
  return STA_NODISK;
}

extern void error_log_emmc_disk_io(const DRESULT d, const char function_indicator);

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
extern void sdcard_log_disk_read_time(const systime_t now_time, const systime_t start_time);

DRESULT disk_read (
    BYTE drv,        /* Physical drive nmuber (0..) */
    BYTE *buff,        /* Data buffer to store read data */
    DWORD sector,    /* Sector address (LBA) */
    UINT count        /* Number of sectors to read (1..255) */
)
{
  const systime_t start_time = chTimeNow();

  switch (drv) {
#if HAL_USE_MMC_SPI
  case MMC:
    if (blkGetDriverState(&MMCD1) != BLK_READY)
      return RES_NOTRDY;
    if (mmcStartSequentialRead(&MMCD1, sector))
      return RES_ERROR;
    while (count > 0) {
      if (mmcSequentialRead(&MMCD1, buff))
        return RES_ERROR;
      buff += MMCSD_BLOCK_SIZE;
      count--;
    }
    if (mmcStopSequentialRead(&MMCD1))
        return RES_ERROR;
    return RES_OK;
#else
  case SDC:
    disk_wake(&SDCD1);
    if (blkGetDriverState(&SDCD1) != BLK_READY) {
      error_log_emmc_disk_io(RES_NOTRDY, 'R');
      return RES_NOTRDY;
    }

    if (sdcRead(&SDCD1, sector, buff, count)) {
      error_log_emmc_disk_io(RES_ERROR, 'R');
#if ENABLE_SDC_READ_WRITE_RETRYS
      if (sdcRead(&SDCD1, sector, buff, count)) {
        error_log_emmc_disk_io(RES_ERROR, 'R');
        if (sdcRead(&SDCD1, sector, buff, count)) {
          error_log_emmc_disk_io(RES_ERROR, 'R');
          return RES_ERROR;
        } else {
          error_log_emmc_disk_io(RES_OK, 'R');
        }
      } else {
        error_log_emmc_disk_io(RES_OK, 'R');
      }
#else
      return RES_ERROR;
#endif

    }

    const systime_t end_time = chTimeNow();
    sdcard_log_disk_read_time(end_time, start_time);
    return RES_OK;
#endif
  }
  error_log_emmc_disk_io(RES_PARERR, 'R');
  return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)
*/

extern void sdcard_log_disk_write_time(const systime_t now_time, const systime_t start_time);

#if _READONLY == 0
DRESULT disk_write (
    BYTE drv,            /* Physical drive nmuber (0..) */
    const BYTE *buff,    /* Data to be written */
    DWORD sector,        /* Sector address (LBA) */
    UINT count            /* Number of sectors to write (1..255) */
)
{
  const systime_t start_time = chTimeNow();

  switch (drv) {
#if HAL_USE_MMC_SPI
  case MMC:
    if (blkGetDriverState(&MMCD1) != BLK_READY)
        return RES_NOTRDY;
    if (mmcIsWriteProtected(&MMCD1))
        return RES_WRPRT;
    if (mmcStartSequentialWrite(&MMCD1, sector))
        return RES_ERROR;
    while (count > 0) {
        if (mmcSequentialWrite(&MMCD1, buff))
            return RES_ERROR;
        buff += MMCSD_BLOCK_SIZE;
        count--;
    }
    if (mmcStopSequentialWrite(&MMCD1))
        return RES_ERROR;
    return RES_OK;
#else
  case SDC:
    disk_wake(&SDCD1);
    if (blkGetDriverState(&SDCD1) != BLK_READY) {
      error_log_emmc_disk_io(RES_NOTRDY, 'W');
      return RES_NOTRDY;
    }
    if (sdcWrite(&SDCD1, sector, buff, count)) {
      error_log_emmc_disk_io(RES_ERROR, 'W');
#if ENABLE_SDC_READ_WRITE_RETRYS
      if (sdcWrite(&SDCD1, sector, buff, count)) {
        error_log_emmc_disk_io(RES_ERROR, 'W');
        if (sdcWrite(&SDCD1, sector, buff, count)) {
          error_log_emmc_disk_io(RES_ERROR, 'W');
          return RES_ERROR;
        } else {
          error_log_emmc_disk_io(RES_OK, 'W');
        }
      } else {
        error_log_emmc_disk_io(RES_OK, 'W');
      }
#else
      return RES_ERROR;
#endif


    }
    const systime_t end_time = chTimeNow();

    sdcard_log_disk_write_time(end_time, start_time);

    return RES_OK;
#endif
  }
  error_log_emmc_disk_io(RES_PARERR, 'W');
  return RES_PARERR;
}
#endif /* _READONLY */



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */

DRESULT disk_ioctl (
    BYTE drv,        /* Physical drive nmuber (0..) */
    BYTE ctrl,        /* Control code */
    void *buff        /* Buffer to send/receive control data */
)
{
  switch (drv) {
#if HAL_USE_MMC_SPI
  case MMC:
    switch (ctrl) {
    case CTRL_SYNC:
        return RES_OK;
    case GET_SECTOR_SIZE:
        *((WORD *)buff) = MMCSD_BLOCK_SIZE;
        return RES_OK;
#if _USE_ERASE
    case CTRL_ERASE_SECTOR:
        mmcErase(&MMCD1, *((DWORD *)buff), *((DWORD *)buff + 1));
        return RES_OK;
#endif
    default:
        return RES_PARERR;
    }
#else
  case SDC:
    disk_wake(&SDCD1);
    switch (ctrl) {
    case CTRL_SYNC:
        return RES_OK;
    case GET_SECTOR_COUNT:
        *((DWORD *)buff) = mmcsdGetCardCapacity(&SDCD1);
        return RES_OK;
    case GET_SECTOR_SIZE:
        *((WORD *)buff) = MMCSD_BLOCK_SIZE;
        return RES_OK;
    case GET_BLOCK_SIZE:
        *((DWORD *)buff) = 256; /* 512b blocks in one erase block */
        return RES_OK;
#if _USE_ERASE
    case CTRL_ERASE_SECTOR:
        sdcErase(&SDCD1, *((DWORD *)buff), *((DWORD *)buff + 1));
        return RES_OK;
#endif
    default:
        error_log_emmc_disk_io(RES_PARERR, 'I');
        return RES_PARERR;
    }
#endif
  }
  error_log_emmc_disk_io(RES_PARERR, 'I');
  return RES_PARERR;
}

extern DWORD get_fattime2();
DWORD get_fattime(void) {
#if HAL_USE_RTC
    return(get_fattime2());
    //return rtcGetTimeFat(&RTCD1);
#else
    return ((uint32_t)0 | (1 << 16)) | (1 << 21); /* wrong but valid time */
#endif
}



