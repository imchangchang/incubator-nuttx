#include "ll_motor.h"
#include <nuttx/config.h>
#include <debug.h>
#include <assert.h>
#include <string.h>
#include <nuttx/timers/pwm.h>
#include <nuttx/sensors/qencoder.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include "arch/board/board.h"

#include "stm32_gpio.h"
#include "stm32_pwm.h"
#include "stm32_qencoder.h"
#include <stdio.h>
#include "pid.h"

#define PWM_FREQ (1000)
#define MOTOR_WORK_DT (10) //ms
#define MOTOR_WORK_TICKS (MSEC_PER_TICK / (MOTOR_WORK_DT))
#define LOG_WORK (0)

#define MAX_SPEED (3000)
#define MIN_SPEED (500)
static void set_pwm(uint8_t chlx, int32_t duty);
static void motor_break(uint8_t chlx);
static void motor_release(uint8_t chlx);


static int motor_open(FAR struct file * filep);
static int motor_close(FAR struct file * filep);
static ssize_t motor_read(FAR struct file * filep, FAR char * buffer, size_t buflen);
static ssize_t motor_write(FAR struct file * filep, FAR char * buffer, size_t buflen);
static int motor_ioctl(FAR struct file * filep, int cmd, unsigned long arg);

static const struct file_operations g_motorops =
{
    .open = motor_open,
    .close = motor_close,
    .read = motor_read,
    .write = motor_write,
    .ioctl = motor_ioctl
};

enum motor_status_e
{
    MOTOR_STOP, //no output
    MOTOR_BREAK,
    MOTOR_PWM,
    MOTOR_PID, //in pid control
};

struct ll_motor_s
{
    struct qe_lowerhalf_s *qe;
    uint8_t qe_chl;
    uint8_t pwm_chl;
    uint32_t a_pin;
    uint32_t b_pin;
    //
    clock_t last_clock;
    float spd;
    PID_t pid;
    float last_pwm;
    enum motor_status_e status;
    int32_t sp_pwm;
    int32_t sp_spd;
};

static struct pwm_lowerhalf_s *g_pwm_lowerhalf;
static uint32_t g_pwm_duty[4];

struct work_s g_motor_work;

static struct ll_motor_s g_motor[4] =
{
    {
        .qe_chl = 2,
        .pwm_chl = 0,
        .a_pin = M1_A_PORT,
        .b_pin = M1_B_PORT,
    },
    {
        .qe_chl = 3,
        .pwm_chl = 1,
        .a_pin = M2_A_PORT,
        .b_pin = M2_B_PORT,
    },
    {
        .qe_chl = 4,
        .pwm_chl = 2,
        .a_pin = M3_A_PORT,
        .b_pin = M3_B_PORT,
    },
    {
        .qe_chl = 5,
        .pwm_chl = 3,
        .a_pin = M4_A_PORT,
        .b_pin = M4_B_PORT,
    },
};

static bool motor_locked(void)
{
    return stm32_gpioread(MOTOR_SW_PORT);
}
#define MOTOR_WORK_REPEAT() work_queue(HPWORK, &g_motor_work, motor_worker, (void *)g_motor, MOTOR_WORK_TICKS)
static void motor_worker(FAR void *arg)
{
    struct ll_motor_s *motor = (struct ll_motor_s *)arg;
    for (int i = 0; i < 4; i++)
    {
        int32_t cur_pos;
        motor[i].qe->ops->position(motor[i].qe, &cur_pos);
        clock_t cur_clock = clock();
        motor[i].qe->ops->reset(motor[i].qe);

        float time_diff_sec = (float)(cur_clock - motor[i].last_clock) / TICK_PER_SEC;
        motor[i].spd = cur_pos / time_diff_sec;
        motor[i].last_clock = cur_clock;

        if (motor_locked())
        {
            motor_release(i);
        }
        else {
            if (motor[i].status == MOTOR_PID)
            {
                float pwm = pid_calculate(&motor[i].pid, motor[i].sp_spd, motor[i].spd, 0, time_diff_sec);
#define MAX_ACC (65535 * 0.1)
                if (pwm >= motor[i].last_pwm + MAX_ACC)
                {
                    pwm = motor[i].last_pwm + MAX_ACC;
                }
                else if (pwm <= motor[i].last_pwm - MAX_ACC)
                {
                    pwm = motor[i].last_pwm - MAX_ACC;
                }
#undef MAX_ACC
                set_pwm(i, pwm);
                motor[i].last_pwm = pwm;
            }
            else if (motor[i].status == MOTOR_PWM)
            {
                set_pwm(i, motor[i].sp_pwm);
                motor[i].last_pwm = motor[i].sp_pwm;
            }
            else if (motor[i].status== MOTOR_BREAK)
            {
                motor_break(i);
                motor[i].last_pwm = 0;
            }
            else {
                motor_release(i);
                motor[i].last_pwm = 0;
            }
        } //else motor_locked()
    }
    MOTOR_WORK_REPEAT();
}

#define MnGPIO_CONFIG(port) (port | GPIO_OUTPUT | GPIO_SPEED_50MHz | GPIO_PUSHPULL | GPIO_FLOAT | GPIO_OUTPUT_CLEAR)

#if LOG_WORK
struct work_s g_log_work;
#define LOG_WORK_REPEAT() work_queue(LPWORK, &g_log_work, log_worker, (void *)NULL, SEC2TICK(1))

static void log_worker(FAR void *arg)
{
    for (int i = 0; i < 4; i++)
    {
        printf("(%5d %6.1f) ", g_pwm_duty[i], g_motor[i].spd);
    }
    printf(" (%d) \n", stm32_gpioread(MOTOR_SW_PORT));

    fflush(stdout);
    LOG_WORK_REPEAT();
}
#endif

static void _set_pwm(uint32_t duty1, uint32_t duty2, uint32_t duty3, uint32_t duty4)
{
    struct pwm_info_s pwm_info =
    {
        .frequency = PWM_FREQ,
        .channels =
        {
            {duty1, 1},
            {duty2, 2},
            {duty3, 3},
            {duty4, 4},
        }
    };
    g_pwm_lowerhalf->ops->start(g_pwm_lowerhalf, &pwm_info);
}

static void set_pwm(uint8_t chlx, int32_t duty)
{
    if (duty > 0)
    {
        stm32_gpiowrite(g_motor[chlx].a_pin, false);
        stm32_gpiowrite(g_motor[chlx].b_pin, true);
    }
    else
    {
        stm32_gpiowrite(g_motor[chlx].a_pin, true);
        stm32_gpiowrite(g_motor[chlx].b_pin, false);
        duty = -duty;
    }

    g_pwm_duty[chlx] = duty;

    _set_pwm(g_pwm_duty[0],
             g_pwm_duty[1],
             g_pwm_duty[2],
             g_pwm_duty[3]);
}

static void motor_break(uint8_t chlx)
{
    stm32_gpiowrite(g_motor[chlx].a_pin, true);
    stm32_gpiowrite(g_motor[chlx].b_pin, true);
    g_pwm_duty[chlx] = 0xffff;
    _set_pwm(g_pwm_duty[0],
             g_pwm_duty[1],
             g_pwm_duty[2],
             g_pwm_duty[3]);
}

static void motor_release(uint8_t chlx)
{
    stm32_gpiowrite(g_motor[chlx].a_pin, false);
    stm32_gpiowrite(g_motor[chlx].b_pin, false);
    g_pwm_duty[chlx] = 0;
}


static int motor_open(FAR struct file * filep)
{
    struct ll_motor_s *motor = (struct ll_motor_s *)(filep->f_inode->i_private);
    motor->status = MOTOR_STOP;
    return OK;
}
static int motor_close(FAR struct file * filep)
{
    struct ll_motor_s *motor = (struct ll_motor_s *)(filep->f_inode->i_private);
    motor->status = MOTOR_STOP;
    return OK;
}
static ssize_t motor_read(FAR struct file * filep, FAR char * buffer, size_t buflen)
{
    struct ll_motor_s *motor = (struct ll_motor_s *)(filep->f_inode->i_private);
    uint8_t _motor_locked = motor_locked();

    if (buflen < sizeof(motor->spd))
    {
        return -1;
    }
    memcpy(buffer, &motor->spd,sizeof(motor->spd));

    if (buflen < sizeof(motor->spd) + sizeof(_motor_locked))
    {
        return sizeof(motor->spd);
    }

    memcpy(buffer + sizeof(motor->spd), &_motor_locked, sizeof(_motor_locked));
    return sizeof(motor->spd) + sizeof(_motor_locked);
}

static ssize_t motor_write(FAR struct file * filep, FAR char * buffer, size_t buflen)
{
    struct ll_motor_s *motor = (struct ll_motor_s *)(filep->f_inode->i_private);
    int32_t sp;

    memcpy(&sp, buffer, sizeof(int32_t));

    if (motor->status == MOTOR_PWM)
    {
        motor->sp_pwm = (int32_t)((float)sp/10000 * 65535);
    }
    else if (motor->status == MOTOR_PID)
    {
        int _sp_spd = (int32_t)((float)sp/10000 * MAX_SPEED);
        if (_sp_spd < MIN_SPEED)
        {
            _sp_spd = 0;
        }
        motor->sp_spd = _sp_spd;
    }
    else {
        motor->sp_pwm = 0;
        motor->sp_spd = 0;
    }
    return OK;
}
static int motor_ioctl(FAR struct file * filep, int cmd, unsigned long arg)
{
    struct ll_motor_s *motor = (struct ll_motor_s *)(filep->f_inode->i_private);
    switch (cmd)
    {
    case MOTOR_STOP: //no output
        motor->status = MOTOR_STOP;
        motor->sp_pwm = 0;
        motor->sp_spd= 0;
        break;
    case MOTOR_BREAK:
        motor->status = MOTOR_BREAK;
        motor->sp_pwm = 0;
        motor->sp_spd= 0;
        break;
    case MOTOR_PWM:
        motor->status = MOTOR_PWM;
        motor->sp_pwm = 0;
        break;
    case MOTOR_PID: //in pid control
        motor->status = MOTOR_PID;
        motor->sp_spd= 0;
        break;
    default:
        motor->status = MOTOR_STOP;
        motor->sp_pwm = 0;
        motor->sp_spd= 0;
        break;
    }
    return OK;
}

void ll_motor_initialize(void)
{
    const char *dev_name[4] =
    {
        "/dev/qe0",
        "/dev/qe1",
        "/dev/qe2",
        "/dev/qe3",
    };
    const char *mdev_name[4] =
    {
        "/dev/motor0",
        "/dev/motor1",
        "/dev/motor2",
        "/dev/motor3",
    };
    stm32_configgpio(MOTOR_SW_GPIO_CONFIG);
    for (int i = 0; i < 4; i++)
    {
        g_motor[i].qe = (struct qe_lowerhalf_s *)stm32_qeinitialize(dev_name[i], g_motor[i].qe_chl);
        g_motor[i].qe->ops->setup(g_motor[i].qe);

        stm32_configgpio(MnGPIO_CONFIG(g_motor[i].a_pin));
        stm32_configgpio(MnGPIO_CONFIG(g_motor[i].b_pin));
        pid_init(&g_motor[i].pid, PID_MODE_DERIVATIV_CALC, MOTOR_WORK_DT / 1000);
        pid_set_parameters(&g_motor[i].pid, 50, 40, 1, 65535, 65535);
        g_motor[i].status = MOTOR_STOP;

        register_driver(mdev_name[i], &g_motorops, 777, (intptr_t)&g_motor[i]);
    }
    g_pwm_lowerhalf = stm32_pwminitialize(1);
    g_pwm_lowerhalf->ops->setup(g_pwm_lowerhalf);
    MOTOR_WORK_REPEAT();
#if LOG_WORK
    LOG_WORK_REPEAT();
#endif
}