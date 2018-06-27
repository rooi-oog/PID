#include "config.h"
#include "pid.h"

static uint8_t pid_reset_flag = 0;

/* Total PID reset */
void pid_reset (arm_pid_t *pid)
{
	 pid_reset_flag = 1;
	 /* Reset cycle should takes some time in order to 
	  * compensate motor rotational inertia */   
	 for (uint16_t i = 0; i < 65534; i++) {
		 pid_set_pwm (pid, 0);		 
		 pid->last_error = 0;
		 pid->integral = 0;	
		 pid->setpoint = 0;
		 pid->feedback = 0;
		 pid->subcmd = 0;
		*pid->encoder  = 0;	
		 pid->velocity = 0;
		 pid->last_position = 0;		
	}
	pid_reset_flag = 0;	
}

/* Set PID parameters */
void pid_set_parameters (arm_pid_t *pid, int32_t value, uint8_t index)
{
	switch (index)
	{
	/* Velocity limit */
	case 0: pid->gains.VelocityLimit = value; break;
	/* Velocity feedback gain */
	case 1: pid->gains.Kf = value; break;
	/* Proportional gain */
	case 2: pid->gains.Kp = value; break;
	/* Integral gain */
	case 3: 
		pid->gains.Ki = value; 
		/* Anti wind-up: set integral limit to maximum PWM value */
		pid->integral_limit = 
			(pid->pwm_max_val << FIXED_POINT_POSITION) / pid->gains.Ki;
		break;
	/* Derivative gain */
	case 4: pid->gains.Kd = value; break;
	/* Torque feedback gain */
	case 5: pid->gains.Kg = value; break;
	}		
}

/* Set new PID position command */ 
void pid_set_point (arm_pid_t *pid, int32_t setpoint)
{
	pid->setpoint = setpoint;
}

/* Set command for servo submode (PWM, TORQUE, VELOCITY) */
void pid_set_subcmd (arm_pid_t *pid, int32_t cmd)
{
	pid->subcmd = cmd;
}

/* Set new servo mode */
void pid_set_mode (arm_pid_t *pid, uint8_t mode)
{
	pid->servo_mode = mode;
}

__attribute__((always_inline)) inline
void pid_set_pwm (arm_pid_t *pid, int32_t duty)
{
	*pid->pwm_p = duty > 0 ?  duty : 0;	
	*pid->pwm_n = duty < 0 ? -duty : 0;
}

/* Main PID function */
void pid_update (arm_pid_t *pid, uint8_t speed_compare)
{	
	/* Intermediate result of PID calc. */
	register int64_t out = 0;
	/* Difference between setpoint and feedback */
	register int32_t error;
	/* Servo mode sub-command */
	register int32_t pidcmd = pid->subcmd;
	
	/* If we under reset cycle */
	if (pid_reset_flag) return;
	
	/* Update current motor position */
	pid->feedback += (int16_t) *pid->encoder;
	 	
	/* Reset encoder counter */
	*pid->encoder = 0;

	if (speed_compare) {
		/* Calculating derivative of motor position */
		pid->velocity = pid->feedback - pid->last_position;
		pid->last_position = pid->feedback;
	}
	
	switch (pid->servo_mode)
	{
	case POSITION:
		/* Position error is velocity command */
		pidcmd = LIMIT (pid->gains.VelocityLimit, pid->setpoint - pid->feedback);
		__attribute__ ((fallthrough));	// Supress compilator's warning
			
	case SPEED:
		/* Get mismatch of setpoint and feedback with feedback gain */
		error = pidcmd - ((pid->velocity * pid->gains.Kf) >> FIXED_POINT_POSITION);
		
		/* Integral part */
		out = (int64_t) pid->integral * (int64_t) pid->gains.Ki;
		
		/* Accumulate error + anti wind-up */
		pid->integral = LIMIT (pid->integral_limit, pid->integral + error);
			
		/* Proportional part */
		out += (int64_t) error * (int64_t) pid->gains.Kp;
		
		/* Derivative part */
		out += (int64_t) (error - pid->last_error) * (int64_t) pid->gains.Kd;
		
		/* Get error derivative */
		pid->last_error = error;
		
		/* Torque limit */
		pidcmd = LIMIT (pid->integral_limit, out >> FIXED_POINT_POSITION);
		__attribute__ ((fallthrough));
		
	case TORQUE:
		pidcmd += (pid->velocity * pid->gains.Kg) >> FIXED_POINT_POSITION; 
		__attribute__ ((fallthrough));
		
	case PWM:
		pid_set_pwm (pid, LIMIT (pid->pwm_max_val, pidcmd));				
	}
}

void pid_setup (arm_pid_t *pid, const arm_pid_gains_t *pid_gains, int32_t mode,
	int32_t pwm_max_val, uint32_t *pwm_p, uint32_t *pwm_n, uint32_t *encoder)
{
	bzero (pid, sizeof (arm_pid_t));
	
	pid->pwm_max_val = pwm_max_val;
	pid->pwm_p = pwm_p;
	pid->pwm_n = pwm_n;
	pid->encoder = encoder;
	pid->servo_mode = mode;
	
	/* Load PID gains */
	memcpy (&pid->gains, pid_gains, sizeof (arm_pid_gains_t));
	
	/* Anti wind-up: set integral limit to maximum PWM value */
	pid->integral_limit = (pid->pwm_max_val << FIXED_POINT_POSITION) / pid->gains.Ki; 
}

