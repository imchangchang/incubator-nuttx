
#include <stdint.h>




void    ll_motor_initialize(void);
//[-1000,1000]
void    ll_motor_set_duty(int chl, int16_t duty);

void    ll_motor_break(int chl);

