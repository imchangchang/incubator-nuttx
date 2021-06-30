#include <nuttx/config.h>
#include <debug.h>
#include <assert.h>
#include <nuttx/timers/pwm.h>
#include <nuttx/sensors/qencoder.h>

#include "arch/board/board.h"

#include "stm32_gpio.h"
#include "stm32_pwm.h"
#include "stm32_lowputc.h"
#include "stm32_qencoder.h"

static void led_init(void)
{
    stm32_configgpio(LED_GPIO_CONFIG);
}

static void pwm_init(void)
{
    struct pwm_lowerhalf_s *pwm_lowerhalf = stm32_pwminitialize(1);
    int ret = pwm_register(MOTOR_PWM_PATH, pwm_lowerhalf);
    assert(ret == OK);
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

int board_app_initialize(uintptr_t arg)
{

    led_init();
    pwm_init();
    qe_init();
    m_gpio_init();

    return OK;
}
