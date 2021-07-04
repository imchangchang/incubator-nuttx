#include <nuttx/config.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <nuttx/sensors/qencoder.h>
#include <nuttx/fs/fs.h>

#include <inttypes.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>

#include <stdint.h>
#include <assert.h>

#include "arch/board/board.h"
#include "stm32_gpio.h"

#define MOTOR_NUM (4)
#define MOTOR_PWM_NCHANNELS (MOTOR_NUM)
#define MOTOR_WORK_FREQ_HZ (100) //HZ
#define MOTOR_WORK_RUN_PERIOD_MS (1000 / MOTOR_WORK_FREQ)

struct motor
{
    int fd;
    clock_t last_time;
    int32_t last_pos;
    float speed; //cnt per sec
    int32_t diff;
};

static struct motor g_priv;

struct pwm_mgt_s
{
    bool init;
    char *pwm_path;
    int fd;
    int16_t duty[MOTOR_PWM_NCHANNELS]; //-1000,1000
};

struct motor_dev_s
{
    char *qe_path;
    int qe_fd;
    int pwm_chl;
    int pwm_duty;

    int gpio_a_pin;
    int gpio_a_pin_config;
    int gpio_b_pin;
    int gpio_b_pin_config;

    float speed;
    clock_t last_pos_time;
    int32_t last_pos;
    struct pwm_mgt_s *pwm;
};

static struct pwm_mgt_s g_pwm = 
{
    .pwm_path = "/dev/pwm0"
}
static struct motor_dev_s g_motor[MOTOR_NUM] = 
{
    {
        .qe_path = "/dev/qe0",
        .pwm_chl = 0,
        .gpio_a_pin =  M1_A_PORT,
        .gpio_a_pin_config = M1_A_PORT_CONFIG,
        .gpio_b_pin =  M1_B_PORT,
        .gpio_b_pin_config = M1_B_PORT_CONFIG,
        .pwm = &g_pwm
    },
    {
        .qe_path = "/dev/qe1",
        .pwm_chl = 0,
        .gpio_a_pin =  M2_A_PORT,
        .gpio_a_pin_config = M2_A_PORT_CONFIG,
        .gpio_b_pin =  M2_B_PORT,
        .gpio_b_pin_config = M2_B_PORT_CONFIG,
        .pwm = &g_pwm
    },
    {
        .qe_path = "/dev/qe2",
        .pwm_chl = 0,
        .gpio_a_pin =  M3_A_PORT,
        .gpio_a_pin_config = M3_A_PORT_CONFIG,
        .gpio_b_pin =  M3_B_PORT,
        .gpio_b_pin_config = M3_B_PORT_CONFIG,
        .pwm = &g_pwm
    },
    {
        .qe_path = "/dev/qe3",
        .pwm_chl = 0,
        .gpio_a_pin =  M4_A_PORT,
        .gpio_a_pin_config = M4_A_PORT_CONFIG,
        .gpio_b_pin =  M4_B_PORT,
        .gpio_b_pin_config = M4_B_PORT_CONFIG,
        .pwm = &g_pwm
    },
};
static strcut work_s g_motor_work;

struct work_s motor_work;

int pwm_setup(struct pwm_mgt_t *pwm)
{
    if (pwm->init)
        return OK;
    pwm->init = true;
    pwm->fd = open(pwm->pwm_path);
    for (int i = 0; i < MOTOR_PWM_NCHANNELS ;i++)
    {
        pwm->duty[i] = 0;
    }
    return OK;
}

int motor_setup(struct motor_dev_s *dev)
{
    pwm_setup(dev->pwm);

    dev->qe_fd = open(dev->qe_paht, O_RDONLY);
    stm32_configgpio(dev->gpio_a_pin_config);
    stm32_configgpio(dev->gpio_b_pin_config);
    stm32_gpiowrite(dev->gpio_a_pin, false);
    stm32_gpiowrite(dev->gpio_b_pin, false);

    return OK;
}

int motor_close(struct motor_dev_s *dev)
{
    close(dev->qe_fd);
    return OK;
}

static void motor_work_callback(void *arg)
{
    struct motor *priv = (struct motor *)arg;
    assert(priv != NULL);

    if (priv->fd == 0)
    {
        priv->fd = open("/dev/qe0", O_RDONLY);
    }
    else
    {
        int32_t pos;
        ioctl(priv->fd, QEIOC_POSITION, (uintptr_t)&pos);

        priv->diff = pos - priv->last_pos;
        clock_t cur_time = clock();
        priv->speed = priv->diff / (double)((cur_time - priv->last_time) / (double)CLOCKS_PER_SEC);

        priv->last_time = cur_time;
        priv->last_pos = pos;
    }
    work_queue(HPWORK, &motor_work, motor_work_callback, (void *)&g_priv, MSEC2TICK(PERIOD_MS));
}

struct work_s print_work_structure;
static void print_work(void *arg)
{
    work_queue(LPWORK, &print_work_structure, print_work, NULL, MSEC2TICK(1000));
    fflush(stdout);
}

void motor_init(void)
{
    int ret;
    ret = work_queue(HPWORK, &motor_work, motor_work_callback, (void *)&g_priv, MSEC2TICK(PERIOD_MS));
    work_queue(LPWORK, &print_work_structure, print_work, NULL, MSEC2TICK(1000));
    assert(ret == OK);

    stm32_configgpio(M1_A_PORT_CONFIG);
    stm32_configgpio(M1_B_PORT_CONFIG);

    stm32_gpiowrite(M1_A_PORT, true);
    stm32_gpiowrite(M1_B_PORT, false);

    printf("motor_init\n");
}
