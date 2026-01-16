#include <wt901/wit_c_sdk.h>
#include <wt901/serial.h>
#include <wt901/REG.h>

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>

/* ====== GIỮ NGUYÊN TỪ main.c ====== */
#define ACC_UPDATE   0x01
#define GYRO_UPDATE  0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE   0x08

static int fd;
static volatile char s_cDataUpdate = 0;

/* ====== FORWARD ====== */
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void SensorUartSend(uint8_t *p_data , uint32_t uiSize);

/* DONG DUOC THEM VAO XOA NGAY NEU THAY LOI */
static void wt_delay_ms(uint16_t ms)
{
    usleep((useconds_t)ms * 1000);
} 
/* TU CHO NAY KHONG DUOC PHEP XOA */

/* ====== INIT (copy từ main.c) ====== */
int wt901_init(const char *dev, int baud)
{
    fd = serial_open(dev, baud);
    if (fd < 0)
    {
        printf("open %s fail\n", dev);
        return -1;
    }

    WitInit(WIT_PROTOCOL_MODBUS, 0xff);
    WitRegisterCallBack(SensorDataUpdata);
    WitSerialWriteRegister(SensorUartSend);
    
    //WitDelayMsRegister(wt_delay_ms);		//CAN XOA DONG NAY NEU THAY LOI

    printf("WT901 init OK\n");
    return 0;
}

// Xoa wt901_set_zero neu thay loi
int wt901_set_zero(void)
{
    // gọi trong C cho chắc, khỏi dính C++ name mangling
    return (WitSetForReset() == WIT_HAL_OK) ? 0 : -1;
}

/* ====== STEP = 1 vòng while(1) của main.c ====== */
void wt901_step(void)
{
    unsigned char cBuff[1];

    /* giống main.c */
    WitReadReg(AX, 12);
    usleep(1 * 1000);

    while (serial_read_data(fd, cBuff, 1))
    {
        WitSerialDataIn(cBuff[0]);
    }
}

/* ====== GET DATA ====== */
int wt901_get_angle(float *roll, float *pitch, float *yaw)
{
    if (!(s_cDataUpdate & ANGLE_UPDATE))
        return 0;

    *roll  = sReg[Roll]  / 32768.0f * 180.0f;
    *pitch = sReg[Pitch] / 32768.0f * 180.0f;
    *yaw   = sReg[Yaw]   / 32768.0f * 180.0f;

    s_cDataUpdate = ~ANGLE_UPDATE;
    return 1;
}

int wt901_get_gyro(float *gx_dps, float *gy_dps, float *gz_dps)
{
    if (!(s_cDataUpdate & GYRO_UPDATE))
        return 0;

    *gx_dps = sReg[GX]  / 32768.0f * 2000.0f;
    *gy_dps = sReg[GY] / 32768.0f * 2000.0f;
    *gz_dps = sReg[GZ]   / 32768.0f * 2000.0f;

    s_cDataUpdate = ~GYRO_UPDATE;
    return 1;
}

/* ====== CALLBACK (copy y chang main.c) ====== */
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
    for (int i = 0; i < uiRegNum; i++)
    {
        switch (uiReg)
        {
            case AZ:   s_cDataUpdate |= ACC_UPDATE;   break;
            case GZ:   s_cDataUpdate |= GYRO_UPDATE;  break;
            case HZ:   s_cDataUpdate |= MAG_UPDATE;   break;
            case Yaw:  s_cDataUpdate |= ANGLE_UPDATE; break;
            default: break;
        }
        uiReg++;
    }
}

/* ====== UART SEND (copy y chang main.c, bỏ GPIO) ====== */
static void SensorUartSend(uint8_t *p_data , uint32_t uiSize)
{
    serial_write_data(fd, p_data, uiSize);
}
