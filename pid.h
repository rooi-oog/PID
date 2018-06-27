#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>
#include <string.h>

#define FIXED_POINT_POSITION		16
#define INTEGER_PART				(1 << FIXED_POINT_POSITION)

#ifndef LIMIT
#ifdef BIDIRECTIONAL
#define LIMIT(l,val)				val > l ? l : val < -l ? -l : val
#else
#define LIMIT(l,val)				val > l ? l : val <  0 ?  0 : val
#endif
#endif

enum servo_mode
{
	POSITION = 0,
	SPEED,
	TORQUE,
	PWM
};

typedef struct
{
	int32_t VelocityLimit;
	int32_t Kf;
	int32_t Kp;
	int32_t Ki;
	int32_t Kd;
	int32_t Kg;
	int64_t align;
} arm_pid_gains_t;

typedef struct
{
	arm_pid_gains_t gains;
	int32_t subcmd;
	int32_t servo_mode;	
	int32_t setpoint;
	int32_t feedback;
	int32_t velocity;
	int32_t integral;
	int32_t integral_limit;
	int32_t last_error;
	int32_t last_position;
	int32_t pwm_max_val;
	uint32_t *pwm_p;
	uint32_t *pwm_n;
	uint32_t *encoder;
} arm_pid_t;

void pid_setup (arm_pid_t *pid_inst, const arm_pid_gains_t *, int32_t, int32_t, uint32_t *, uint32_t *, uint32_t *);
void pid_set_point (arm_pid_t *, int32_t);
void pid_set_parameters (arm_pid_t *, int32_t, uint8_t);
void pid_reset (arm_pid_t *);
void pid_set_mode (arm_pid_t *, uint8_t);
void pid_set_subcmd (arm_pid_t *, int32_t);
void pid_set_pwm (arm_pid_t *, int32_t);
void pid_update (arm_pid_t *, uint8_t);

#endif /* __PID_H__ */

