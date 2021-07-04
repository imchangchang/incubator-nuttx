
#include <stdint.h>
#include <stdbool.h>



struct ll_motor_dev_s;

struct ll_motor_ops
{
    int (*setup)(struct ll_motor_dev_s * dev, bool direction);
    int (*get_pos)(struct ll_motor_dev_s * dev, int32_t * pos);
    int (*set_pwm)(struct ll_motor_dev_s * dev, int16_t pwm);
    int (*start)(struct ll_motor_dev_s * dev);
    int (*stop)(struct ll_motor_dev_s * dev);
    int (*brk)(struct ll_motor_dev_s * dev);
};


struct ll_motor_dev_s
{
    struct ll_motor_ops * ops;
    bool direction;
    uint8_t id;
    char * encoder_dev;
    int encoder_idx;
    int fd_encoder;
    int32_t gpio_a_pin;
    int32_t gpio_b_pin;
    int32_t pwm_chl;
    void (*set_tim_pwm)(int32_t chl, int32_t pwm);
};