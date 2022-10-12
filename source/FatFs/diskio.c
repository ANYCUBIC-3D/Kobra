/*-----------------------------------------------------------------------*/
/* Low level disk I/O module SKELETON for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */

#include "hc32_common.h"
#include "bsp_sdio.h"


/* Definitions of physical drive number for each drive */
#define DEV_RAM		0	/* Example: Map Ramdisk to physical drive 0 */
#define DEV_MMC		1	/* Example: Map MMC/SD card to physical drive 1 */
#define DEV_USB		2	/* Example: Map USB MSD to physical drive 2 */


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	int result;


//    result = MMC_disk_status();

    // translate the reslut code here

    return stat;

}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	int result;

//		result = MMC_disk_initialize();

		// translate the reslut code here

		return stat;

	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	LBA_t sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	DRESULT res;

	// translate the arguments here

//	result = MMC_disk_read(buff, sector, count);

    en_result_t result = SDCARD_ReadBlocks(&stcSdhandle, sector, count, (uint8_t *)buff, 20000);

	// translate the reslut code here

    if(Ok != result) {
        res = RES_ERROR;
    } else{
        res = RES_OK;
    }

    return res;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	LBA_t sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	DRESULT res;

//	result = MMC_disk_write(buff, sector, count);
    en_result_t result = SDCARD_WriteBlocks(&stcSdhandle, sector, count, (uint8_t *)buff, 20000);

    if(Ok != result) {
        res = RES_ERROR;
    } else{
        res = RES_OK;
    }

	// translate the reslut code here

	return res;

}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;
	int result;

    switch (cmd) {
    case CTRL_SYNC:
//            printf("CTRL_SYNC\r\n");
        return RES_OK;

    case GET_SECTOR_COUNT:
//            printf("GET_SECTOR_COUNT\r\n");
        *(DWORD * )buff = stcSdhandle.stcSdCardInfo.u32BlockNbr;
        return RES_OK;

    case GET_SECTOR_SIZE :
//            printf("GET_SECTOR_SIZE\r\n");
        *(WORD * )buff = 512;
        return RES_OK;

    case GET_BLOCK_SIZE :
//            printf("GET_BLOCK_SIZE\r\n");
        *(DWORD * )buff = 1;
        return RES_OK;

    default:
        return RES_PARERR;
    }

	// Process of the command for the MMC/SD card

	return res;

}

