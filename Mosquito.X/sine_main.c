/* 
 * File:   sine_main.c
 * Author: Sean Carroll
 *
 * Created on September 10, 2015, 9:12 AM
 */

// graphics libraries
#include "config.h"
// need for rand function
#include <stdlib.h> 
#include <xc.h>
#include <math.h>

// threading library
// config.h sets 40 MHz
#define	SYS_FREQ 40000000
#include "pt_cornell_1_2_1.h"
#define SAMP_FREQ 20000
#define TIMER_PR (SYS_FREQ/SAMP_FREQ) // SAMP_FREQ = SYS_FREQ/TIMER_PR
#define INCR_CONST ((float)pow(2, 32)/SAMP_FREQ) // INCR = F_OUT * INCR_CONST
#define ISR_1_MS (SAMP_FREQ/1000) // 1 ms in ISR Ticks

// == Flash Programming ==================================================
#define FLASH_PAGE_SIZE_BYTES	(4096) // 4k bytes/page for PIC32
#define FLASH_SIZE_BYTES (4*1024) // allocate 4k bytes of flash (one page)

extern volatile UINT8 _FLASH_BASE[];
extern void * FLASH_BASE;
UINT8 pagebuf[4096];
UINT8 buf[512];

// Note that:
// "bytes" needs to be a multiple of BYTE_PAGE_SIZE (and aligned that way) if you intend to erase
// "bytes" needs to be a multiple of BYTE_ROW_SIZE (and aligned that way) if you intend to write rows
// "bytes" needs to be a multiple of sizeof(int) if you intend to write words
#define NVM_ALLOCATE(name, align, bytes) volatile UINT8 name[(bytes)] \
__attribute__ ((aligned(align),section(".text,\"ax\",@progbits #"))) = \
{ [0 ...(bytes)-1] = 0xFF } 

NVM_ALLOCATE(_FLASH_BASE, FLASH_PAGE_SIZE_BYTES, FLASH_SIZE_BYTES); // allocate NVM flash logging buffer, aligned on erasable page
void* FLASH_BASE = &_FLASH_BASE;


// == SPI Stuff ==========================================================
volatile unsigned int DAC_data; // output value
volatile SpiChannel spiChn = SPI_CHANNEL2; // the SPI channel to use
volatile int spiClkDiv = 2; // 20 MHz max speed for this DAC
//serial variables
static char cmd[16];
static float cmd_value;
static int param;

// A-channel, 1x, active
#define DAC_config_chan_A (0b0011000000000000)
// Sine Table Size
#define TABLE_BIT_SIZE                     (8) 
#define TABLE_SIZE      (1 << TABLE_BIT_SIZE)
#define SHIFT_AMT       (32 - TABLE_BIT_SIZE)
// Number of harmonics to use
#define MAX_HARMONICS                      (6)
#define MAX_PHASE               (2147483648.0) // 0x80000000
#define DEFAULT_FUNDAMENTAL_FREQ         (464)
#define TWO_PI                        (6.2832)


// System settings struct, using in storing to flash
typedef struct _SYS_SETTINGS {
    int           FUNDAMENTAL;
    unsigned char HARMONICS;
    unsigned int  AMPLITUDE[MAX_HARMONICS];
    unsigned int  NORMAL[MAX_HARMONICS];
    unsigned int  PHASE[MAX_HARMONICS];
    unsigned int  INCR[MAX_HARMONICS];
} SYS_SETTINGS;

SYS_SETTINGS SETTINGS;

// Max function
#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

// 16 bit unsigned (need 12 bits for DAC)
unsigned short sineTable[TABLE_SIZE];

// 32 bit unsigned
volatile unsigned int phase[MAX_HARMONICS];
volatile unsigned int value = 0;

// Sound on flag
volatile unsigned char sound_on = 1;

// --- thread structures -------------------------------------------------
// thread control structs
// note that UART input and output are threads
static struct pt pt_timer,
pt_frequency,
pt_cmd,
pt_input,
pt_DMA_output,
pt_output;

// system 1 second interval tick
int sys_time_seconds;

void recalculateHarmonics(){
    SETTINGS.INCR[0] = INCR_CONST * SETTINGS.FUNDAMENTAL * 1;
    SETTINGS.INCR[1] = INCR_CONST * SETTINGS.FUNDAMENTAL * 2;
    SETTINGS.INCR[2] = INCR_CONST * SETTINGS.FUNDAMENTAL * 3;
    SETTINGS.INCR[3] = INCR_CONST * SETTINGS.FUNDAMENTAL * 4;
    SETTINGS.INCR[4] = INCR_CONST * SETTINGS.FUNDAMENTAL * 5;
    SETTINGS.INCR[5] = INCR_CONST * SETTINGS.FUNDAMENTAL * 6;
}

void normalizeAmplitudes(){
    int j = 0;
    SETTINGS.NORMAL[0] = SETTINGS.AMPLITUDE[0];
    SETTINGS.NORMAL[1] = SETTINGS.AMPLITUDE[0] + 
                         SETTINGS.AMPLITUDE[1];
    SETTINGS.NORMAL[2] = SETTINGS.AMPLITUDE[0] +
                         SETTINGS.AMPLITUDE[1] +
                         SETTINGS.AMPLITUDE[2];
    SETTINGS.NORMAL[3] = SETTINGS.AMPLITUDE[0] +
                         SETTINGS.AMPLITUDE[1] +
                         SETTINGS.AMPLITUDE[2] +
                         SETTINGS.AMPLITUDE[3];
    SETTINGS.NORMAL[4] = SETTINGS.AMPLITUDE[0] +
                         SETTINGS.AMPLITUDE[1] +
                         SETTINGS.AMPLITUDE[2] +
                         SETTINGS.AMPLITUDE[3] +
                         SETTINGS.AMPLITUDE[4];
    SETTINGS.NORMAL[5] = SETTINGS.AMPLITUDE[0] +
                         SETTINGS.AMPLITUDE[1] +
                         SETTINGS.AMPLITUDE[2] +
                         SETTINGS.AMPLITUDE[3] +
                         SETTINGS.AMPLITUDE[4] +
                         SETTINGS.AMPLITUDE[5];
    for (j = 0; j < MAX_HARMONICS; j++){
        if (SETTINGS.NORMAL[j] == 0) SETTINGS.NORMAL[j] = 1;
    }
}

void initPhase(){
    int i = 0;
    for (i = 0; i < MAX_HARMONICS; i++){
        phase[i] = SETTINGS.PHASE[i];
    }
}

// Function to Initialize All Parameters after Program being Flashed
// should only be called once after a flash
void flashInit() {
    // This is the first time through and the memory has not been initialized yet, so... do it
    SETTINGS.FUNDAMENTAL = DEFAULT_FUNDAMENTAL_FREQ;
    SETTINGS.HARMONICS   =                        5;
    
    // initialize phases and frequency increments for wing tones
    SETTINGS.PHASE[0] =         0; // phase of  0
    SETTINGS.PHASE[1] = MAX_PHASE; // phase of pi
    SETTINGS.PHASE[2] = MAX_PHASE; // phase of pi
    SETTINGS.PHASE[3] =         0; // phase of  0
    SETTINGS.PHASE[4] =         0;
    SETTINGS.PHASE[5] = MAX_PHASE;
    initPhase();

    SETTINGS.INCR[0] = INCR_CONST * SETTINGS.FUNDAMENTAL * 1;
    SETTINGS.INCR[1] = INCR_CONST * SETTINGS.FUNDAMENTAL * 2;
    SETTINGS.INCR[2] = INCR_CONST * SETTINGS.FUNDAMENTAL * 3;
    SETTINGS.INCR[3] = INCR_CONST * SETTINGS.FUNDAMENTAL * 4;
    SETTINGS.INCR[4] = INCR_CONST * SETTINGS.FUNDAMENTAL * 5;
    SETTINGS.INCR[5] = INCR_CONST * SETTINGS.FUNDAMENTAL * 6;

    // amplitudes
    SETTINGS.AMPLITUDE[0] = 6762; // arbitrary weightings of each harmonic
    SETTINGS.AMPLITUDE[1] = 1515; //   based on FFT analysis of flight tones
    SETTINGS.AMPLITUDE[2] = 1301;
    SETTINGS.AMPLITUDE[3] =  242;
    SETTINGS.AMPLITUDE[4] =  121;
    SETTINGS.AMPLITUDE[5] =   59;
    
    //normalize amplitudes
    normalizeAmplitudes();
    // now that the variables have been initialized, write them to flash
    
    // First, erase the page
    NVMErasePage(FLASH_BASE);
    // Write the page to memory
    NVMProgram(FLASH_BASE, &SETTINGS, sizeof (SETTINGS), pagebuf);
};

// Function to save all Parameters as they are
void saveParameters() {
    NVMErasePage(FLASH_BASE);

    NVMProgram(FLASH_BASE, &SETTINGS, sizeof (SETTINGS), pagebuf);
}

// modified from Tahmid's DAC Tutorial
void initDAC(void) {
    // control CS for DAC
    mPORTBSetPinsDigitalOut(BIT_4);
    mPORTBSetBits(BIT_4);
    PPSOutput(2, RPB5, SDO2); // use RPB5 (pin 14) for SDO2
    //Use SPI chn 2
    SpiChnOpen(spiChn,
            SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV,
            spiClkDiv);

    // Clock at 20MHz
}

// from Tahmid's DAC Tutorial, changed to SPI2
inline void writeDAC(unsigned short data) {
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
void generateTables(void) {
    unsigned int i;
    for (i = 0; i < TABLE_SIZE; i++) {
        sineTable[i] = (short) (2047.0 * sin(TWO_PI * ((float) i) / (float) TABLE_SIZE));
        sineTable[i] = sineTable[i] + 2047;
    }
}

// from Tahmid's blog
void initTimers(void) {
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
void __ISR(_TIMER_2_VECTOR, ipl2)T2Int(void) {
    unsigned int i = 0;
    value = 0;

    // generate combination of wing tones
    for (i = 0; i < SETTINGS.HARMONICS; i++) {
        // phase accumulator
        phase[i] = phase[i] + SETTINGS.INCR[i];
        // add each successive amplitude multiplied by its weighting
        value = value + \
                ((SETTINGS.AMPLITUDE[i] * sineTable[phase[i] >> SHIFT_AMT]) / 
                  SETTINGS.NORMAL[SETTINGS.HARMONICS-1]);
    }

    // output to DAC
    writeDAC(value & 0xfff);

    mT2ClearIntFlag();
}

// ==================================================
// ==== Threads =====================================
// ==================================================

// === Timer Thread =================================================
// update a 1 second tick counter
static PT_THREAD(protothread_timer(struct pt *pt)) {
    PT_BEGIN(pt);
    while (1) {
        sys_time_seconds++;
        mPORTAToggleBits(BIT_0);
        PT_YIELD_TIME_msec(1000);
        // NEVER exit while
    } // END WHILE(1)
    PT_END(pt);
} // timer thread

// === Frequency Thread =============================
static PT_THREAD(protothread_frequency(struct pt *pt)) {
    PT_BEGIN(pt);
    while (1) {
        PT_YIELD_TIME_msec(100);
        // NEVER exit while
    } // END WHILE(1)
    PT_END(pt);
} // frequency thread

// === Command Thread ======================================================
// The serial interface
static PT_THREAD(protothread_cmd(struct pt *pt)) {
    PT_BEGIN(pt);
    static int cmd_i = 0;
    while (1) {
        // send the prompt via DMA to serial
        sprintf(PT_send_buffer, "cmd>");
        // by spawning a print thread
        PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));

        //spawn a thread to handle terminal input
        // the input thread waits for input
        // -- BUT does NOT block other threads
        // string is returned in "PT_term_buffer"
        PT_SPAWN(pt, &pt_input, PT_GetSerialBuffer(&pt_input));
        // returns when the thead dies
        // in this case, when <enter> is pushed
        // now parse the string
        cmd_value = -1;
        param =  -1;
        sscanf(PT_term_buffer, "%s %f %d", cmd, &cmd_value, &param);
        // reset the send buffer
        sprintf(PT_send_buffer, "");

        // system time command
        if (cmd[0] == 't') {
            sprintf(PT_send_buffer, "%d \n", sys_time_seconds);
            // by spawning a print thread
            PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
        }

        // help command
        if (cmd[0] == 'h') {
            sprintf(PT_send_buffer, "Commands:\n");
            PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
            sprintf(PT_send_buffer, "-- t: Prints system time\n");
            PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
            sprintf(PT_send_buffer, "-- p: Prints current parameters\n");
            PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
            sprintf(PT_send_buffer, "-- i: Sets and saves all parameters to default values\n");
            PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
            sprintf(PT_send_buffer, "-- s: Saves all parameters as they are currently set\n");
            PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
        }

        // current parameters
        if (cmd[0] == 'p') {
            sprintf(PT_send_buffer, "Parameters:\n");
            PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output));
        }

        // Initialize memory for Frequency Bounds on random walk if not set yet
        if (cmd[0] == 'i') {
            flashInit();
            memcpy(&SETTINGS, FLASH_BASE, sizeof (SETTINGS));
        }

        // Save current parameters on Frequency Bounds
        if (cmd[0] == 's') {
            saveParameters();
        }
       
        //reset the command buffer
        sprintf(cmd, "");
        // never exit while
    } // END WHILE(1)
    PT_END(pt);
} // UART Thread

// ==== Main ========================================
void main(void) {

    // Configure the device for maximum performance but do not change the PBDIV
    // Given the options, this function will change the flash wait states, RAM
    // wait state and enable prefetch cache but will not change the PBDIV.
    // The PBDIV value is already set via the pragma FPBDIV option above..
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    PT_setup();
    
    // generate the sine lookup table for DDS
    generateTables();

    // Set all DAC Parameters
    initDAC();
    
    //recall parameters saved in flash memory
    memcpy(&SETTINGS, FLASH_BASE, sizeof (SETTINGS));
    
    // safety on flashing, not executed between resets
    if (SETTINGS.HARMONICS < 0 || SETTINGS.HARMONICS > 6){
        flashInit();
    }
    
    initPhase();
    
    // setup the interrupt timer
    initTimers();

    // enable interrupts
    INTEnableSystemMultiVectoredInt();

    // init the threads
    PT_INIT(&pt_timer);
    PT_INIT(&pt_frequency);
    PT_INIT(&pt_cmd);
    PT_INIT(&pt_input);
    PT_INIT(&pt_DMA_output);
    PT_INIT(&pt_output);
    
    // Initialize Blinking Power LED
    mPORTAClearBits(BIT_0);
    mPORTASetPinsDigitalOut(BIT_0);

    // schedule both threads in round robin
    while (1) {
        PT_SCHEDULE(protothread_timer(&pt_timer));
        PT_SCHEDULE(protothread_frequency(&pt_frequency));
        PT_SCHEDULE(protothread_cmd(&pt_cmd));
    }
} // end main

