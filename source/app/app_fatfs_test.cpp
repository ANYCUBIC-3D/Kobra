#include "hc32_ddl.h"
#include "bsp_adc.h"

#include "diskio.h"



void app_fatfs_test(void)
{
    FATFS FatFs, *fs;
    FRESULT res;
    DIR Dir;
    FIL File[2];
    FIL fp;
    FILINFO Finfo;
    BYTE b, drv = 0;
    UINT num = 0;
    uint64_t acc_size = 0;
    uint32_t acc_files = 0, acc_dirs = 0, dw = 0;

    res = f_mount(&FatFs, "", 1);
    if(res == FR_OK) {
        printf("f_mount ok!\n");
    } else {
        printf("f_mount failed!\n");
    }

    res = f_opendir(&Dir, "/");
    if(res == FR_OK) {
        printf("f_opendir ok!\n");
    } else {
        printf("f_opendir failed!\n");
    }

    while(1) {
        res = f_readdir(&Dir, &Finfo);
        if ((res != FR_OK) || !Finfo.fname[0]) break;
        if (Finfo.fattrib & AM_DIR) {
            acc_dirs++;
        } else {
            acc_files++; acc_size += Finfo.fsize;
        }
        printf("%c%c%c%c%c %u/%02u/%02u %02u:%02u %9lu  %s\n",
                (Finfo.fattrib & AM_DIR) ? 'D' : '-',
                (Finfo.fattrib & AM_RDO) ? 'R' : '-',
                (Finfo.fattrib & AM_HID) ? 'H' : '-',
                (Finfo.fattrib & AM_SYS) ? 'S' : '-',
                (Finfo.fattrib & AM_ARC) ? 'A' : '-',
                (Finfo.fdate >> 9) + 1980, (Finfo.fdate >> 5) & 15, Finfo.fdate & 31,
                (Finfo.ftime >> 11), (Finfo.ftime >> 5) & 63,
                Finfo.fsize, Finfo.fname);
    }

    printf("%4u File(s),%10llu bytes total\n%4u Dir(s)", acc_files, acc_size, acc_dirs);

    res = f_getfree("/", &dw, &fs);
    if (res == FR_OK) {
        printf(", %10llu bytes free\n", (QWORD)dw * fs->csize * 512);
    } else {
        printf("f_getfree failed!\n");
    }

    res = f_open(&fp, (const char *)"adc.txt", FA_READ | FA_WRITE | FA_OPEN_ALWAYS);
    if (res == FR_OK) {
        printf("f_open ok\n");
    } else {
        printf("f_open failed!\n");
    }

    char str[] = "ABC\nabc\n123\n";

    res = f_sync(&fp);
    if (res == FR_OK) {
        printf("f_sync ok\n");
        printf("f_size: %d\n", f_size(&fp));
    } else {
        printf("f_sync failed!\n");
    }

    res = f_lseek(&fp, f_size(&fp));
    if (res == FR_OK) {
        printf("f_lseek bytes: %d\n", f_size(&fp));
    } else {
        printf("f_lseek failed!\n");
    }

    res = f_write(&fp, str, strlen(str), &num);
    if (res == FR_OK) {
        printf("f_write bytes: %d\n", num);
    } else {
        printf("f_write failed!\n");
    }

    res = f_close(&fp);
    if (res == FR_OK) {
        printf("f_close ok\n");
    } else {
        printf("f_close failed!\n");
    }
}

void loop_write_adc_to_file(void)
{
    FIL fp;
    UINT num = 0;
    char adc_buf[1024];
    FRESULT res;

    while(1) {
    
        sprintf(adc_buf, "%.5d    %.4d    %.4d    %.4d\n", SysTick_GetTick(), g_adc_value[0], g_adc_value[1], g_adc_value[2]);
        
        printf("adc_buf: %s\n", adc_buf);
        
        res = f_write(&fp, adc_buf, strlen(adc_buf), &num);
        if (res == FR_OK) {
            printf("f_write bytes: %d\n", num);
        } else {
            printf("f_write failed!\n");
        }
        
        res = f_sync(&fp);
        if (res == FR_OK) {
            printf("f_sync ok\n");
            printf("f_size: %d\n", f_size(&fp));
        } else {
            printf("f_sync failed!\n");
        }
    }
}

