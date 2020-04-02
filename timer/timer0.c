#include "timer0.h"
#include "tm4c123gh6pm.h"

void systick_init(int time){
   NVIC_ST_CTRL_R = 0x0;
   NVIC_ST_CURRENT_R |= 0x0;
   NVIC_ST_RELOAD_R = time;
   NVIC_ST_CTRL_R = 0x5;

}

void systick_delaym(int seconds) {
    int reload= 16000*seconds-1;

    systick_init(reload);

     while(!(NVIC_ST_CTRL_R & 0x10000));



}

void timer0_oneshot_init(){
    SYSCTL_RCGCTIMER_R|=0x01;  //enabling the sys clock for timer 0

    TIMER0_CTL_R=0x0;          //disabling timer
    TIMER0_CFG_R= 0x0;         //32-bit configuration
    TIMER0_TAMR_R=0x01;       //one shot mode
}

/*
delay in sec
mode: 1-> one-shot 2->periodic
*/

void timer0_set_m(int delay, int mode){
    timer0_oneshot_init();
    TIMER0_TAILR_R=16000*delay-1; //loading the delay value
    TIMER0_CTL_R|=mode;                //enabling timer0

}

void timer0_set_u(int delay, int mode){
    timer0_oneshot_init();
    TIMER0_TAILR_R=16*delay-1; //loadimg the delay value
    TIMER0_CTL_R|=mode;                //enabling timer0

}

void timer0_reset(){
   TIMER0_ICR_R = 0x1;      //clear the timeout flag for timerA
}

int timeout(){
    if(TIMER0_RIS_R & 1){
        return 1;
    }
    return 0;
}

void delaym(int time){
    timer0_set_m(time,1); //set timer0 A to one ssec in one-shot mode
    while(!timeout());
    timer0_reset();
}

void delayu(int time){
    timer0_set_u(time,1); //set timer0 A to one ssec in one-shot mode
    while(!timeout());
    timer0_reset();
}
