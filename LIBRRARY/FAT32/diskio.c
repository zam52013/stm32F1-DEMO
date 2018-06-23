/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2016        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "diskio.h"     /* FatFs lower layer API */
#include "fm25v01.h"

#ifdef USE_SDIO_SD
#include "stm32_eval_sdio_sd.h"
#endif
#ifdef USE_SPI_SD
#include "stm32_eval_spi_sd.h"
#endif

/* Definitions of physical drive number for each drive */
#define DEV_RAM     0   /* Example: Map Ramdisk to physical drive 0 */
#define DEV_MMC     1   /* Example: Map MMC/SD card to physical drive 1 */
#define DEV_USB     2   /* Example: Map USB MSD to physical drive 2 */

#define SECTOR_SIZE 512

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status(
    BYTE pdrv       /* Physical drive nmuber to identify the drive */
)
{
    DSTATUS stat;
    int result;

    switch(pdrv)
    {
        case DEV_RAM :
            //result = RAM_disk_status();

            // translate the reslut code here

            return stat;

        case DEV_MMC :
            //result = MMC_disk_status();
            result = SD_GetStatus();

            // translate the reslut code here

            return result;

        case DEV_USB :
            //result = USB_disk_status();

            // translate the reslut code here

            return stat;
    }

    return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize(
    BYTE pdrv               /* Physical drive nmuber to identify the drive */
)
{
    DSTATUS stat;
    int result;

    switch(pdrv)
    {
        case DEV_RAM :
            //result = RAM_disk_initialize();

            // translate the reslut code here

            return stat;

        case DEV_MMC :
            //result = MMC_disk_initialize();
            result = SD_Init();
            // translate the reslut code here

            return result;

        case DEV_USB :
            //result = USB_disk_initialize();

            // translate the reslut code here

            return stat;
    }

    return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read(
    BYTE pdrv,      /* Physical drive nmuber to identify the drive */
    BYTE *buff,     /* Data buffer to store read data */
    DWORD sector,   /* Start sector in LBA */
    UINT count      /* Number of sectors to read */
)
{
    DRESULT res;
    int result;
    SD_Error Status;

    switch(pdrv)
    {
        case DEV_RAM :
            // translate the arguments here

            //result = RAM_disk_read(buff, sector, count);
            Fram_read(sector << 9, count, buff);

            // translate the reslut code here

            return res;

        case DEV_MMC :
            // translate the arguments here

            //result = MMC_disk_read(buff, sector, count);
            Status = SD_ReadMultiBlocks((uint8_t *)buff, sector << 9, SECTOR_SIZE, count);

            //result =SD_ReadBlock(buff,sector << 9 ,count);

            // translate the reslut code here
            if(Status == 0)
            {
                return RES_OK;
            }

            return RES_ERROR;

        case DEV_USB :
            // translate the arguments here

            //result = USB_disk_read(buff, sector, count);

            // translate the reslut code here

            return res;
    }

    return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT disk_write(
    BYTE pdrv,          /* Physical drive nmuber to identify the drive */
    const BYTE *buff,   /* Data to be written */
    DWORD sector,       /* Start sector in LBA */
    UINT count          /* Number of sectors to write */
)
{
    DRESULT res;
    int result;
    SD_Error Status;

    switch(pdrv)
    {
        case DEV_RAM :
            // translate the arguments here

            //result = RAM_disk_write(buff, sector, count);
            Fram_wirte(sector << 9, count, (uint8_t *)buff);

            // translate the reslut code here

            return res;

        case DEV_MMC :
            // translate the arguments here

            //result = MMC_disk_write(buff, sector, count);
            Status = SD_WriteMultiBlocks((uint8_t *)buff, sector << 9, SECTOR_SIZE, count);

            //result =SD_WriteBlock((uint8_t *)buff,sector << 9 ,count);

            // translate the reslut code here
            if(Status == 0)
            {
                return RES_OK;
            }

            return RES_ERROR;


        case DEV_USB :
            // translate the arguments here

            //result = USB_disk_write(buff, sector, count);

            // translate the reslut code here

            return res;
    }

    return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl(
    BYTE pdrv,      /* Physical drive nmuber (0..) */
    BYTE cmd,       /* Control code */
    void *buff      /* Buffer to send/receive control data */
)
{
    DRESULT res;
    int result;

    switch(pdrv)
    {
        case DEV_RAM :

            // Process of the command for the RAM drive

            return res;

        case DEV_MMC :

            // Process of the command for the MMC/SD card

            return res;

        case DEV_USB :

            // Process of the command the USB drive

            return res;
    }

    return RES_PARERR;
}
/*
*********************************************************************************************************
*   函 数 名: get_fattime
*   功能说明: 获得系统时间，用于改写文件的创建和修改时间。
*   形    参：无
*   返 回 值: 无
*********************************************************************************************************
*/
DWORD get_fattime(void)
{
    /* 如果有全局时钟，可按下面的格式进行时钟转换. 这个例子是2013-01-01 00:00:00 */

    return ((DWORD)(2013 - 1980) << 25)     /* Year = 2013 */
           | ((DWORD)1 << 21)              /* Month = 1 */
           | ((DWORD)1 << 16)              /* Day_m = 1*/
           | ((DWORD)0 << 11)              /* Hour = 0 */
           | ((DWORD)0 << 5)               /* Min = 0 */
           | ((DWORD)0 >> 1);              /* Sec = 0 */
}

