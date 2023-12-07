#include <inttypes.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "uartparse.h"
#include "clock.h"
#include "tm4c123gh6pm.h"
#include "gpio.h"
#include "nvic.h"
#include "uart0.h"
#include "wait.h"
#include "pwm.h"
#include "i2c0.h"

#define signalIn PORTC,6
#define blue PORTF,2
#define opticalLeft PORTC,4
#define opticalRight PORTD,2
#define LEFTINMASK 16
#define RIGHTINMASK 4

enum IRCommands {
    forward = 59670,          //data corresponding to buttons on NEC remote
    backward = 61455,
    left = 65280,
    right = 64770,
    ok = 65025,
    meterBack = 32640,
    meterForward = 63240,
    right90 = 62985,
    left90 = 63495
};

float time_ms;               //IR command variables
uint8_t state = 0;
uint32_t data;
uint16_t command;
uint16_t prevCommand;
uint8_t done;
uint16_t dutyL = 900;        //Optical interrupt data
uint16_t dutyR = 920;
uint8_t leftCount;
uint8_t rightCount;
int8_t calA;
int8_t calB;
uint8_t dirA;
uint8_t dirB;

int16_t accx;                 //IMU orientation data
int16_t accy;
int16_t accz;
float accelPitch = 0.0;
float gyroPitch = 0.0;
float fusedPitch = 0.0;
float yaw;

int16_t gx;
int16_t gy;
int16_t gz;

int16_t gyroZero = 0;

bool timerRunning = false;

int32_t coeffKp = 300;         //PID control
int32_t coeffKi = 00;
int32_t coeffKd = 0;
int32_t coeffKo = 780;
int32_t coeffK = 100; 
int32_t integral = 0;
int32_t iMax = 75;
int32_t diff;
int32_t error;
int32_t u = 0;
int32_t deadBand = 5;

USER_DATA userData;           //Command-line function variables

uint8_t meterCount;

float setPitch = 0.0;

char dataStr[50];
char imuStr[70];

void readIMU();

void enableTimer()
{
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1;
    _delay_cycles(3);
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER1_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER1_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
                                                     // configure for edge time mode, count up
    WTIMER1_CTL_R = TIMER_CTL_TAEVENT_NEG;           // measure time from negative edge to negative edge
    WTIMER1_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER1_TAV_R = 0;                               // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    setNvicInterruptPriority(INT_WTIMER1A-16-96, 0);
    NVIC_EN3_R = 1 << (INT_WTIMER1A-16-96);         // turn-on interrupt 112 (WTIMER1A)
}

void wideTimer1Isr()                              //NEC IR command recieve
{
    time_ms = WTIMER1_TAV_R/40e3;
    WTIMER1_TAV_R = 0;

    if(time_ms>11.125 && time_ms<11.375 && done==1)   //repeat condition
    {
        command = prevCommand;
        done = 0;
        //snprintf(dataStr, sizeof(dataStr), "Previous command: %"PRIu16"\n",command);
        //putsUart0(dataStr);
    }

    else if(time_ms>13.25 && time_ms<13.75)         //start condition
    {
        state = 1;
    }

    else if(state>0 && state<=32)                  //if start condition valid and command not complete
    {
        if(time_ms>1 && time_ms<1.25)                //0 bit
        {
            data &= ~(1<<(state-1));
            state++;
        }
        else if(time_ms>2.125 && time_ms<2.375)      //1 bit
        {
            data |= (1<<(state-1));
            state++;
        }
    }

    if(state==33)                                    //full 32 bit command
    {
        data >>= 16;
        if(data!=0)
        {
            command = data;
            done = 0;
            snprintf(dataStr, sizeof(dataStr), "Command: %"PRIu32"\n",data);
            putsUart0(dataStr);
        }
    }
    WTIMER1_ICR_R = TIMER_ICR_CAECINT;
}

void initIR()
{
    initSystemClockTo40Mhz();

    enablePort(PORTF);
    enablePort(PORTC);

    selectPinDigitalInput(signalIn);
    setPinAuxFunction(signalIn,7);
    selectPinPushPullOutput(blue);
    setPinValue(blue,0);
}

void initOpticalCounters()
{
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R3 | SYSCTL_RCGCWTIMER_R0;
    _delay_cycles(3);

    enablePort(PORTC);
    enablePort(PORTD);

    GPIO_PORTC_AFSEL_R |= LEFTINMASK;              // select alternative functions for SIGNAL_IN pin
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC4_M;           // map alt fns to SIGNAL_IN
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_WT0CCP0;
    GPIO_PORTC_DEN_R |= LEFTINMASK;                // enable bit for digital input

    GPIO_PORTD_AFSEL_R |= RIGHTINMASK;              // select alternative functions for SIGNAL_IN pin
    GPIO_PORTD_PCTL_R &= ~GPIO_PCTL_PD2_M;           // map alt fns to SIGNAL_IN
    GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD2_WT3CCP0;
    GPIO_PORTD_DEN_R |= RIGHTINMASK;                // enable bit for digital input
}

void enableOpticalCounters()
{
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 2000000;                       // set load value to 40e6 for 1 Hz interrupt rate //change back to 2000000
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);              // turn-on interrupt 37 (TIMER1A)

    // Configure Wide Timer 1 as counter of external events on CCP0 pin //right
    WTIMER3_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER3_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER3_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge count mode, count up
    WTIMER3_CTL_R = TIMER_CTL_TAEVENT_POS;           // count positive edges
    WTIMER3_IMR_R = 0;                               // turn-off interrupts
    WTIMER3_TAV_R = 0;                               // zero counter for first period
    WTIMER3_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    //left
    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER0_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER0_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge count mode, count up
    WTIMER0_CTL_R = TIMER_CTL_TAEVENT_POS;           // count positive edges
    WTIMER0_IMR_R = 0;                               // turn-off interrupts
    WTIMER0_TAV_R = 0;                               // zero counter for first period
    WTIMER0_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter

    timerRunning = true;
}

void disableOpticalCounters()
{
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off time base timer
    WTIMER3_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off event counter
    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;
    NVIC_DIS0_R = 1 << (INT_TIMER1A-16);            // turn-off interrupt 37 (TIMER1A)
    timerRunning = false;
}

void enablePID()
{
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;

    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER2_TAILR_R = 2000000;                          // set load value to 40e3 for 1000 Hz interrupt rate
    TIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN0_R = 1 << (INT_TIMER2A-16);              // turn-on interrupt (TIMER2A)
}

void pidIsr()
{
    readIMU();

    static int32_t errorLast = 0;

    error = fusedPitch - setPitch;

    integral += error;

    int32_t iLimit = iMax * coeffKi;
    if (integral > iLimit)
        integral = iLimit;
    if (integral < -iLimit)
        integral = -iLimit;

    diff = error - errorLast;
    errorLast = error;

    u = (coeffKp * error) + (coeffKi * integral) + (coeffKd * diff);
    u /= coeffK;
    if (u > 173) u = 173;
    if (u < -173) u = -173;

    if (abs(error) > deadBand)
    {
        if(u >= 0)
        {
            dirA = 1;
            dirB = 1;
            leftForward(abs(u) + coeffKo);
            rightForward(abs(u) + coeffKo);
        }
        else
        {
            dirA = 0;
            dirB = 0;
            leftBackwards(abs(u) + coeffKo);
            rightBackwards(abs(u) + coeffKo);
        }
    }

    TIMER2_ICR_R = TIMER_ICR_TATOCINT;
}

void timer1Isr()
{
    rightCount = WTIMER3_TAV_R;                  // read counter input
    leftCount = WTIMER0_TAV_R;

    meterCount += rightCount;

    WTIMER3_TAV_R = 0;                           // reset counter for next period
    WTIMER0_TAV_R = 0;
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;           // clear interrupt flag

    if(timerRunning)
    {
        if(leftCount != rightCount)
        {
            calA = rightCount - leftCount;
            if(((dutyL + (10 * calA)) < 1020) && ((dutyL - (10 * calA)) > 0))
            {
                dutyL += (10 * calA);
            }
            if(dirA == 1)
            {
                leftForward(dutyL);
            }
            else
            {
                leftBackwards(dutyL);
            }
        }
    }
//    snprintf(dataStr, sizeof(dataStr), "Left: %"PRIu8"\nRight: %"PRIu8"\nLeft Duty: %"PRIu16"\nRight Duty: %"PRIu16"\n\n",leftCount, rightCount, dutyL, dutyR);
//    putsUart0(dataStr);
}

void initIMU()
{
    uint8_t add = 0;
    writeI2c0Register(0x68, 0x19, 0x7);  //sample rate 8khz/8 = 1khz
    writeI2c0Register(0x68, 0x1A, 0x3);  //setting filter and clock
    writeI2c0Register(0x68, 0x1B, 0x18); //+-2000 degrees per second
    writeI2c0Register(0x68, 0x1C, 0x0);  //+-2g
    writeI2c0Register(0x68, 0x6B, 0x2);  //PLL with x axis gyroscope reference

    while((!pollI2c0Address(add)) && (add < 255))
    {
        add++;
    }

    //snprintf(dataStr, sizeof(dataStr), "IMU address: %"PRIu8"\n", add);
    //putsUart0(dataStr);

//    int i;
//
//    waitMicrosecond(100000);
//    for(i=0;i<50;i++)
//    {
//        readIMU();
//        gyroZero += gy;
//        waitMicrosecond(10000);
//    }
//    gyroZero /= 50;
}

void getAccelPitch(int16_t * accx, int16_t * accz)
{
    accelPitch = (atan2(-*accx, *accz)) * (180.0/M_PI); //pitch from accelerometer readings
}

void getGyroPitch(int16_t * gy)
{
    gyroPitch += (float)(*gy / 16.4) * 0.05;            //pitch from gyro readings
}

void getYaw(int16_t * gz)
{
    yaw += (float)(*gz / 16.4) * 0.05;                  //yaw to determine which way robot is facing
}

void readIMU()
{
    uint8_t buffer[14];

    readI2c0Registers(0x68, 0x3B, buffer, 14);
    accx = 0;
    accx = (buffer[0] << 8) | buffer[1];

    accy = 0;
    accy = (buffer[2] << 8) | buffer[3];

    accz = 0;
    accz = (buffer[4] << 8) | buffer[5];

    gx = 0;
    gx = (buffer[8] << 8) | buffer[9];

    gy = 0;
    gy = ((buffer[10] << 8) | buffer[11]) + 15; //gyroZero;

    gz = 0;
    gz = ((buffer[12] << 8) | buffer[13]) - 10;

    getAccelPitch(&accx, &accz);
    getGyroPitch(&gy);
    getYaw(&gz);
    fusedPitch = (float)(((rightCount/4.0) * gyroPitch) + ((1 - (rightCount/4.0)) * accelPitch));
//    snprintf(imuStr, sizeof(imuStr), "Accel Pitch: %.2f\nGyro Pitch: %.2f\nFused Pitch: %.2f\n\n", accelPitch, gyroPitch, fusedPitch);
//    putsUart0(imuStr);
}

int main()
{
    initIR();
    initUart0();
    setUart0BaudRate(115200,40e6);
    putsUart0("start\n");
    initI2c0();
    initIMU();
    enablePID();
    enableTimer();
    initPWM();
    initOpticalCounters();
    enableOpticalCounters();

    int i;

    while(true)
    {
        if(kbhitUart0())                                   //user commands from uart0
        {
            getsUart0(&userData);
            parseFields(&userData);

            if(isCommand(&userData, "angle", 1))
            {
                yaw = 0;
            }
            else if(isCommand(&userData, "angle", 0))
            {
                snprintf(imuStr, sizeof(imuStr), "Angle: %.2f\n\n", yaw);
                putsUart0(imuStr);
            }
            else if(isCommand(&userData, "tilt", 0))
            {
                snprintf(imuStr, sizeof(imuStr), "Tilt: %.2f\n\n", fusedPitch);
                putsUart0(imuStr);
            }
            else if(isCommand(&userData, "rotate", 1))
            {
                if(*(getFieldString(&userData, 1) + 2) == 'w')
                {
                    command = left90;
                }
                else
                {
                    command = right90;
                }
            }
            else if(isCommand(&userData, "forward", 0))
            {
                command = meterForward;
            }
            else if(isCommand(&userData, "back", 0))
            {
                command = meterBack;
            }
        }
        switch(command){                   //user commands from IR remote
        case forward:
            setPitch = 25.0;
//            leftForward(dutyL);
//            rightForward(dutyR);
            dirA = 1;
            dirB = 1;
            timerRunning = true;
            prevCommand = command;
            command = 0;
            done = 1;
            break;
        case left:
            leftBackwards(800);
            rightForward(800);
            dirA = 0;
            dirB = 1;
            prevCommand = command;
            command = 0;
            done = 1;
            break;
        case backward:
            leftBackwards(dutyL);
            rightBackwards(dutyR);
            dirA = 0;
            dirB = 0;
            timerRunning = true;
            prevCommand = command;
            command = 0;
            done = 1;
            break;
        case right:
            leftForward(800);
            rightBackwards(800);
            dirA = 1;
            dirB = 0;
            prevCommand = command;
            command = 0;
            done = 1;
            break;
        case right90:
            leftForward(800);
            rightBackwards(800);
            dirA = 1;
            dirB = 0;
//            prevCommand = command;
            yaw = 0;
            while(yaw > -57);
            leftStop();
            rightStop();
            command = 0;
            done = 1;
            break;
        case left90:
            rightForward(800);
            leftBackwards(800);
            dirA = 1;
            dirB = 0;
//            prevCommand = command;
            yaw = 0;
            while(yaw < 57);
            leftStop();
            rightStop();
            command = 0;
            done = 1;
            break;
        case meterForward:
            meterCount = 0;
            rightForward(900);
            leftForward(900);
            dirA = 1;
            dirB = 1;
            timerRunning = true;
            while(meterCount < 100);
            leftStop();
            rightStop();
            command = 0;
            done = 1;
            break;
        case meterBack:
            meterCount = 0;
            rightBackwards(900);
            leftBackwards(900);
            dirA = 0;
            dirB = 0;
            timerRunning = true;
            while(meterCount < 100);
            leftStop();
            rightStop();
            command = 0;
            done = 1;
            break;
        case ok:
            for(i=800;i<1024;i++)
            {
                waitMicrosecond(20000);
                leftBackwards(i);
                rightForward(i);
            }
            waitMicrosecond(4000000);
            for(i=1023;i>=800;i--)
            {
                waitMicrosecond(20000);
                leftBackwards(i);
                rightForward(i);
            }
            prevCommand = command;
            command = 0;
            done = 1;
            break;
        default:                                       //default stops motors and calibrates gyro pitch
            waitMicrosecond(150000);
            if(done==1)
            {
                if(accelPitch < 2.5 && accelPitch > -2.5 && leftCount == 0 && rightCount == 0)
                {
                    gyroPitch = 0;
                }
                timerRunning = false;
                setPitch = 0.0;
                leftStop();
                rightStop();
            }
        }
    }
}
