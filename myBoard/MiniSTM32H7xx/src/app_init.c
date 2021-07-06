#include <nuttx/config.h>
#include <debug.h>
#include <assert.h>
#include <nuttx/timers/pwm.h>
#include <nuttx/sensors/qencoder.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/semaphore.h>

#include "arch/board/board.h"

#include "stm32_gpio.h"
#include "stm32_pwm.h"
#include "stm32_lowputc.h"
#include "stm32_qencoder.h"
#include "stm32_i2c.h"
#include "ll_motor.h"

static void led_init(void)
{
    stm32_configgpio(LED_GPIO_CONFIG);
}

static void pwm_init(void)
{
}

static void qe_init(void)
{
    int ret;
    ret = stm32_qeinitialize(M1_QE_PATH, 2);
    assert(ret == OK);
    ret = stm32_qeinitialize(M2_QE_PATH, 3);
    assert(ret == OK);
    ret = stm32_qeinitialize(M3_QE_PATH, 4);
    assert(ret == OK);
    ret = stm32_qeinitialize(M4_QE_PATH, 5);
    assert(ret == OK);
}

static void m_gpio_init(void)
{
}

static struct i2c_master_s *g_i2c_master;

static sem_t g_sem;

static void i2c_send(uint8_t reg, uint8_t regval)
{
    static struct i2c_msg_s msg;
    static uint8_t txbuffer[2];
    txbuffer[0] = reg;
    txbuffer[1] = regval;

    msg.frequency = 100000;
    msg.addr = 0x78;
    msg.flags = 0;
    msg.buffer = txbuffer;
    msg.length = 2;

    int ret = I2C_TRANSFER(g_i2c_master, &msg, 1);
}

static void i2c_cmd(uint8_t regval)
{
    i2c_send(0x00, regval);
}
static void i2c_data(uint8_t regval)
{
    i2c_send(0x40, regval);
}

void oled_init(void)
{
    g_i2c_master = stm32_i2cbus_initialize(1);
    sleep(1);

    i2c_cmd(0xae); //--turn off oled panel
    i2c_cmd(0x00); //---set low column address
    i2c_cmd(0x10); //---set high column address
    i2c_cmd(0x40); //--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    i2c_cmd(0x81); //--set contrast control register
    i2c_cmd(0xcf); // Set SEG Output Current Brightness
    i2c_cmd(0xa1); //--Set SEG/Column Mapping     0xa0×óÓÒ·ŽÖÃ 0xa1Õý³£
    i2c_cmd(0xc8); //Set COM/Row Scan Direction   0xc0ÉÏÏÂ·ŽÖÃ 0xc8Õý³£
    i2c_cmd(0xa6); //--set normal display
    i2c_cmd(0xa8); //--set multiplex ratio(1 to 64)
    i2c_cmd(0x3f); //--1/64 duty
    i2c_cmd(0xd3); //-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
    i2c_cmd(0x00); //-not offset
    i2c_cmd(0xd5); //--set display clock divide ratio/oscillator frequency
    i2c_cmd(0x80); //--set divide ratio, Set Clock as 100 Frames/Sec
    i2c_cmd(0xd9); //--set pre-charge period
    i2c_cmd(0xf1); //Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    i2c_cmd(0xda); //--set com pins hardware configuration
    i2c_cmd(0x12);
    i2c_cmd(0xdb); //--set vcomh
    i2c_cmd(0x40); //Set VCOM Deselect Level
    i2c_cmd(0x20); //-Set Page Addressing Mode (0x00/0x01/0x02)
    i2c_cmd(0x02); //
    i2c_cmd(0x8d); //--set Charge Pump enable/disable
    i2c_cmd(0x14); //--set(0x10) disable
    i2c_cmd(0xa4); // Disable Entire Display On (0xa4/0xa5)
    i2c_cmd(0xa6); // Disable Inverse Display On (0xa6/a7)
    i2c_cmd(0xaf); //--turn on oled panel

    //fill
    for (int i = 0; i < 8 ; i++)
    {
        i2c_cmd(0xb0+i);
        i2c_cmd(0x01);
        i2c_cmd(0x10);
        for (int j = 0; j < 128; j++)
        {
            i2c_data(0x01);
        }
    }
}


int board_app_initialize(uintptr_t arg)
{

    led_init();
    m_gpio_init();
    ll_motor_initialize();

    return OK;
}
