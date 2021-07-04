#include "ll_motor.h"
#include <nuttx/config.h>
#include <debug.h>
#include <assert.h>
#include <nuttx/timers/pwm.h>
#include <nuttx/sensors/qencoder.h>

#include "arch/board/board.h"

#include "stm32_gpio.h"
#include "stm32_pwm.h"
#include "stm32_qencoder.h"

static int ll_motor_setup(struct ll_motor_dev_s * dev, bool direction);
static int ll_motor_get_pos(struct ll_motor_dev_s * dev, int32_t * pos);
static int ll_motor_set_pwm(struct ll_motor_dev_s * dev, int16_t pwm);
static int ll_motor_start(struct ll_motor_dev_s * dev);
static int ll_motor_stop(struct ll_motor_dev_s * dev);
static int ll_motor_break(struct ll_motor_dev_s * dev);


static const struct ll_motor_ops g_motor_ops = 
{
    .setup = ll_motor_setup,    
    .get_pos = ll_motor_get_pos,
    .set_pwm = ll_motor_set_pwm,
    .start = ll_motor_start,
    .stop = ll_motor_stop,
    .brk = ll_motor_break
};

static void ll_motor_set_tim_pwm(int32_t chl, int32_t pwm);
static void ll_motor_pwm_init(void);

static const struct ll_motor_dev_s g_motor_dev[] = 
{
    {
        .ops = &g_motor_ops,
        .direction = false,
        .id = 1,
        .encoder_dev = "/dev/qe0",
        .encoder_idx = 2,
        .fd_encoder = 0,
        .gpio_a_pin = M1_A_PORT,
        .gpio_b_pin = M1_B_PORT,
        .pwm_chl = 1,
        .set_tim_pwm = ll_motor_set_tim_pwm,
    },
};

struct ll_motor_pwm_s 
{
    bool init;
    char * pwm_dev;
    int pwm_idx;
    int pwm_fd;
    int32_t duty[4];

}g_motor_pwm = 
{
    .init = false,
    .pwm_dev = "/dev/pwm0",
    .pwm_idx = 1,
    .pwm_fd = 0,
};


static int ll_motor_setup(struct ll_motor_dev_s * dev, bool direction)
{
    int ret;
    // check if need init pwm first
    ll_motor_pwm_init();
    //
    ret = stm32_qeinitialize(dev->encoder_dev, dev->encoder_idx);
    assert(ret == OK);

    stm32_configgpio(MxGPIO_CONFIG(dev->gpio_a_pin));
    stm32_configgpio(MxGPIO_CONFIG(dev->gpio_b_pin));


    dev->fd_encoder = open(dev->encoder_dev);

    return OK;
}

static int ll_motor_get_pos(struct ll_motor_dev_s * dev, int32_t * pos)
{
    return OK;
}
static int ll_motor_set_pwm(struct ll_motor_dev_s * dev, int16_t pwm)
{
    return OK;
}
static int ll_motor_start(struct ll_motor_dev_s * dev)
{
    return OK;
}
static int ll_motor_stop(struct ll_motor_dev_s * dev)
{
    return OK;
}
static int ll_motor_break(struct ll_motor_dev_s * dev)
{
    return OK;
}


static void ll_motor_set_tim_pwm(int32_t chl, int32_t pwm)
{
    g_motor_pwm.duty[chl] = pwm;
    //update pwm here
    //TODO
}

static void ll_motor_pwm_init(void)
{
    if (g_motor_pwm.init)
        return;

    g_motor_pwm.init = true;
    struct pwm_lowerhalf_s *pwm_lowerhalf = stm32_pwminitialize(g_motor_pwm.pwm_idx);
    int ret = pwm_register(g_motor_pwm.pwm_dev, pwm_lowerhalf);
    assert(ret == OK);

    g_motor_pwm.pwm_fd = open(g_motor_pwm.pwm_dev);
}