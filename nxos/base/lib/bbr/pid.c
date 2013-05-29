#include "base/types.h"


#include "base/lib/bbr/pid.h"


static S32 MAX_LONG_PLUS1=2147483648;  	// 32 bit signed
static S16 MAX_SHORT_PLUS1=32768;       	// 16 bit signed
static U8 SCALING_SHIFT=16;            	// Scaling factor: 65536

void initPID(struct PID_Control *pid, U32 alphaKP, U32 alphaKI, U32 alphaKD, S32 steady_state_thresh)
{

	pid->alphaKP=alphaKP;
	pid->alphaKI=alphaKI;
	pid->alphaKD=alphaKD;
	pid->steady_state_thresh=steady_state_thresh;

	pid->err_max=(MAX_SHORT_PLUS1-1)/ (pid->alphaKP+1);
	pid->errsum_max= (MAX_LONG_PLUS1-1)/ (pid->alphaKI+1);
	// initialize state variables
	pid->err_sum=0;
	pid->y_prev=0;
	pid->steady_state_count=0;
	pid->ref_val=0;

}

void setPIDReferenceVal(struct PID_Control *pid, S32 reference)
{
	pid->ref_val=reference;
	pid->err_sum=0;
	pid->y_prev=0;
	pid->steady_state_count=0;

}

bool checkPIDEnd(struct PID_Control *pid)
{
	if (pid->steady_state_thresh==0)
		return FALSE;

	if (pid->steady_state_count>pid->steady_state_thresh)
		return TRUE;
	else
		return FALSE;
}


S32 PIDController(struct PID_Control *pid, U8 systemStatus)
{
	S32 P, I, D, difference, error, tmp;

	difference=systemStatus-pid->y_prev;			//

	// update y(n-1) with y(n)
	pid->y_prev=systemStatus;

	error=pid->ref_val-systemStatus;		// Error = RefVal - y(n)

	if (error==0 && pid->steady_state_thresh>0)
	{
		pid->steady_state_count=pid->steady_state_count+1;

		if (pid->steady_state_count>pid->steady_state_thresh)
			return 0;
	}

	// Calculate Pterm and limit error overflow
	if (error > pid->err_max)
	{
		P=MAX_SHORT_PLUS1-1;
	}
	else if (error < -(pid->err_max))
	{
		P=-(MAX_SHORT_PLUS1-1);
	}
	else
	{
		P=(pid->alphaKP)*error;
	}

	// Calculate Iterm and limit integral runaway
	tmp=error+ pid->err_sum;
	if (tmp> pid->errsum_max)
	{
		I=(MAX_LONG_PLUS1-1)/2;
		pid->err_sum=pid->errsum_max;
	}
	else if (tmp < -pid->errsum_max)
	{
		I=-((MAX_LONG_PLUS1-1)/2);
		pid->err_sum=-pid->errsum_max;
	}
	else
	{
		I=pid->alphaKI * pid->err_sum;
		pid->err_sum=tmp;
	}


	// Calculate Dterm
	D=pid->alphaKD*difference;

	return ((P+I+D)>>SCALING_SHIFT);


}
