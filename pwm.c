#include <pwm.h>
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "gpio.h"

#define leftPos PORTB,4
#define leftNeg PORTB,5
#define rightPos PORTB,6
#define rightNeg PORTB,7

void initPWM()
{
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
    enablePort(PORTB);

    _delay_cycles(3);

    selectPinPushPullOutput(leftPos);
    selectPinPushPullOutput(leftNeg);
    selectPinPushPullOutput(rightPos);
    selectPinPushPullOutput(rightNeg);
    setPinAuxFunction(leftPos,4);
    setPinAuxFunction(leftNeg,4);
    setPinAuxFunction(rightPos,4);
    setPinAuxFunction(rightNeg,4);

    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;
    SYSCTL_SRPWM_R = 0;
    PWM0_0_CTL_R = 0;
    PWM0_1_CTL_R = 0;
    PWM0_0_GENA_R = PWM_0_GENA_ACTCMPBD_ONE | PWM_0_GENA_ACTLOAD_ZERO;

    PWM0_0_GENB_R = PWM_0_GENB_ACTCMPAD_ONE | PWM_0_GENB_ACTLOAD_ZERO;

    PWM0_1_GENA_R = PWM_0_GENA_ACTCMPBD_ONE | PWM_0_GENA_ACTLOAD_ZERO;

    PWM0_1_GENB_R = PWM_0_GENB_ACTCMPAD_ONE | PWM_0_GENB_ACTLOAD_ZERO;


    PWM0_0_LOAD_R = 1024;
    PWM0_1_LOAD_R = 1024;

    PWM0_0_CMPA_R = 0;
    PWM0_0_CMPB_R = 0;
    PWM0_1_CMPA_R = 0;
    PWM0_1_CMPB_R = 0;

    PWM0_0_CTL_R = PWM_1_CTL_ENABLE;
    PWM0_1_CTL_R = PWM_1_CTL_ENABLE;
    PWM0_ENABLE_R = PWM_ENABLE_PWM0EN | PWM_ENABLE_PWM1EN | PWM_ENABLE_PWM2EN | PWM_ENABLE_PWM3EN;
}

void rightForward(uint16_t duty)
{
    PWM0_0_CMPA_R = duty;
    PWM0_0_CMPB_R = 0;
}

void rightBackwards(uint16_t duty)
{
    PWM0_0_CMPA_R = 0;
    PWM0_0_CMPB_R = duty;
}

void rightStop(void)
{
    PWM0_0_CMPA_R = 0;
    PWM0_0_CMPB_R = 0;
}

void leftForward(uint16_t duty)
{
    PWM0_1_CMPA_R = duty;
    PWM0_1_CMPB_R = 0;
}

void leftBackwards(uint16_t duty)
{
    PWM0_1_CMPA_R = 0;
    PWM0_1_CMPB_R = duty;
}

void leftStop(void)
{
    PWM0_1_CMPA_R = 0;
    PWM0_1_CMPB_R = 0;
}
