/* 
 * File:   sine_main.c
 * Author: Sean Carroll
 *
 * Created on September 10, 2015, 9:12 AM
 */

// graphics libraries
#include "config.h"
#include "tft_master.h"
#include "tft_gfx.h"
// need for rand function
#include <stdlib.h> 
#include <xc.h>
#include <math.h>

// threading library
// config.h sets 40 MHz
#define	SYS_FREQ 40000000
#include "pt_cornell_1_2.h"
#define SAMP_FREQ 20000
#define TIMER_PR (SYS_FREQ/SAMP_FREQ) // SAMP_FREQ = SYS_FREQ/TIMER_PR
#define INCR_CONST ((float)pow(2, 32)/SAMP_FREQ) // INCR = F_OUT * INCR_CONST
#define ISR_1_MS (SAMP_FREQ/1000) // 1 ms in ISR Ticks

// === the fixed point macros ========================================
typedef signed int fix16 ;
//multiply two fixed 16:16
#define multfix16(a,b) ((fix16)(((( signed long long)(a))*\
                                 (( signed long long)(b)))>>16)) 
#define float2fix16(a) ((fix16)((a)*65536.0)) // 2^16
#define fix2float16(a) ((float)(a)/65536.0)
#define fix2int16(a)   ((int)((a)>>16))
#define int2fix16(a)   ((fix16)((a)<<16))
#define divfix16(a,b)  ((fix16)((((signed long long)(a)<<16)/(b)))) 
#define sqrtfix16(a)   (float2fix16(sqrt(fix2float16(a)))) 
#define absfix16(a)    abs(a)

// == SPI Stuff ==========================================================
volatile unsigned int DAC_data ;// output value
volatile SpiChannel spiChn = SPI_CHANNEL2 ;	// the SPI channel to use
volatile int spiClkDiv = 2 ; // 20 MHz max speed for this DAC

// A-channel, 1x, active
#define DAC_config_chan_A  0b0011000000000000
// Sine Table Size
#define TABLE_BIT_SIZE                      8 
#define TABLE_SIZE      (1 << TABLE_BIT_SIZE)
#define SHIFT_AMT       (32 - TABLE_BIT_SIZE)
// Number of harmonics to use
#define HARMONICS                           4
// Random walk frequency bounds
#define UPPER_BOUND                     0.800
#define LOWER_BOUND                     0.000
#define WALK_INCR                       0.050
#define MID_BOUND (UPPER_BOUND-LOWER_BOUND)/2

// Max function
#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

// 16 bit unsigned (need 12 bits for DAC)
unsigned short sineTable[TABLE_SIZE];

// 32 bit unsigned
volatile unsigned int phase[HARMONICS], 
                       incr[HARMONICS];
volatile unsigned int modPhase = 0,\
                      modIncr  = 0;
static   unsigned int amplitude[HARMONICS];
volatile unsigned int value    = 0,\
                      modValue = 0;

// float (Never used in ISR)
volatile float modFreq = MID_BOUND; //Hz

// == TFT Stuff ==========================================================
// string buffer
char buffer[60];

// --- thread structures -------------------------------------------------
// thread control structs
// note that UART input and output are threads
static struct pt pt_timer,\
              pt_frequency ;

// system 1 second interval tick
int sys_time_seconds ;

// modified from Tahmid's DAC Tutorial
void initDAC(void){
    // control CS for DAC
    mPORTBSetPinsDigitalOut(BIT_4);
    mPORTBSetBits(BIT_4);
    PPSOutput(2, RPB5, SDO2); // use RPB5 (pin 14) for SDO2
    //Use SPI chn 2
    SpiChnOpen(spiChn, 
            SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV ,
            spiClkDiv);

    // Clock at 20MHz
}

// from Tahmid's DAC Tutorial, changed to SPI2
inline void writeDAC(unsigned short data){
    // CS low to start transaction
    mPORTBClearBits(BIT_4); // start transaction
    // test for ready
    while (TxBufFullSPI2());
    // write to spi2
    WriteSPI2(DAC_config_chan_A | data);
    // test for done
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
    // CS high
    mPORTBSetBits(BIT_4); // end transaction
}

// from Tahmid's DAC Tutorial
void generateTables(void){
    unsigned int i;
    for (i = 0; i<TABLE_SIZE; i++){
        sineTable[i] = (short) (2047.0 * sin(6.2832*((float)i)/(float)TABLE_SIZE));
        sineTable[i] = sineTable[i] + 2047;
    }
}

// from Tahmid's blog
void initTimers(void){
    // timer interrupt //////////////////////////
    // Set up timer2 on,  interrupts, internal clock, prescalar 1, toggle rate
    // at 30 MHz PB clock 60 counts is two microsec
    // 400 is 100 ksamples/sec
    // 2000 is 20 ksamp/sec
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, TIMER_PR);

    // set up the timer interrupt with a priority of 2
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    mT2ClearIntFlag(); // and clear the interrupt flag
    
}


// from Tahmid's blog
void __ISR(_TIMER_2_VECTOR, ipl2) T2Int(void){
    unsigned int i = 0;
    value = 0;
    
    // generate combination of wing tones
    for(i=0; i < HARMONICS; i++){
        phase[i] = phase[i] + incr[i];
        // add each successive amplitude multiplied by its weighting
        value = value + \
                ((amplitude[i] * sineTable[phase[i] >> SHIFT_AMT])/10000);
    }
    
    // generate modulation frequency
    modPhase = modPhase + modIncr;
    modValue = sineTable[modPhase >> SHIFT_AMT];
    
    // remove dc offset
    value = value - 2047;
    
    // AM Modulation
    value = (modValue * value) >> 12;
    
    // add dc offset back in
    value = value + 2047;
    
    // output to DAC
    writeDAC(value & 0xfff);
    
    mPORTAToggleBits(BIT_0);
    mT2ClearIntFlag();
}

// ==================================================
// ==== Threads =====================================
// ==================================================

// === Timer Thread =================================================
// update a 1 second tick counter
static PT_THREAD (protothread_timer(struct pt *pt))
{
    PT_BEGIN(pt);
    tft_setCursor(0, 0);
    tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(1);
    tft_writeString("Mosquito Project: Systime:\n");
     while(1) {
       // yield time 1 second
       
       sys_time_seconds++ ;

       // draw sys_time
       tft_fillRoundRect(0,10, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
       tft_setCursor(0, 10);
       tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
       sprintf(buffer,"%d", sys_time_seconds);
       tft_writeString(buffer);
       PT_YIELD_TIME_msec(1000) ;
       // NEVER exit while
     } // END WHILE(1)
  PT_END(pt);
} // timer thread

// === Frequency Thread =============================
static PT_THREAD (protothread_frequency(struct pt *pt))
{
    PT_BEGIN(pt);
    tft_setCursor(0, 50);
    tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(2);
    tft_writeString("Modulation Frequency\n");
    while(1) {
        
        // limit the lower quarter of random walk space to 0
        modIncr = (int) (INCR_CONST * max(modFreq-MID_BOUND/2, 0));
        
        //random walk the modulation freq
        
        //Random walk
        if (rand() > (RAND_MAX / 2)){
            modFreq += WALK_INCR;
        }
        else{
            modFreq -= WALK_INCR;
        }
        
        //bound the walk by lower and upper frequency bounds
        if (modFreq > UPPER_BOUND){
            modFreq = UPPER_BOUND;
        } else if (modFreq < LOWER_BOUND){
            modFreq = LOWER_BOUND;
        }
        
        // display the current AM frequency
        tft_fillRoundRect(0,80, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(0, 80);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
        sprintf(buffer,"%.03f Hz", modFreq);
        tft_writeString(buffer);
        PT_YIELD_TIME_msec(150);
        // NEVER exit while
    } // END WHILE(1)
    PT_END(pt);
} // frequency thread


// ==== Main ========================================
void main(void) {
    
    // Configure the device for maximum performance but do not change the PBDIV
	// Given the options, this function will change the flash wait states, RAM
	// wait state and enable prefetch cache but will not change the PBDIV.
	// The PBDIV value is already set via the pragma FPBDIV option above..
	SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    PT_setup();
    generateTables();
    
    initDAC();
    
    // initialize phases and frequency increments for wing tones
    phase[0] = 0x00000000; // phase of  0
    phase[1] = 0x80000000; // phase of pi
    phase[2] = 0x80000000; // phase of pi
    phase[3] = 0x00000000; // phase of  0
    
    incr[0] = INCR_CONST *  464;
    incr[1] = INCR_CONST *  929;
    incr[2] = INCR_CONST * 1391;
    incr[3] = INCR_CONST * 1856;
    
    // amplitudes must add up to 10000
    amplitude[0] = 6886;
    amplitude[1] = 1543;
    amplitude[2] = 1325;
    amplitude[3] =  246;
    
    // setup the interrupt timer
    initTimers();
    
    // enable interrupts
    INTEnableSystemMultiVectoredInt();
    
    // init the threads
    PT_INIT(&pt_timer);
    PT_INIT(&pt_frequency);
    
    // init the display
    tft_init_hw();
    tft_begin();
    tft_fillScreen(ILI9340_BLACK);
    
    //240x320 vertical display
    tft_setRotation(0); // Use tft_setRotation(1) for 320x240
    
    // schedule both threads in round robin
    while (1){
        PT_SCHEDULE(protothread_timer(&pt_timer));
        PT_SCHEDULE(protothread_frequency(&pt_frequency));
    }
} // end main

