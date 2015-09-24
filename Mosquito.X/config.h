/*
 * File:   config.h
 * Author: Syed Tahmid Mahbub
 *
 * Created on October 10, 2014
 */

#ifndef CONFIG_H
#define	CONFIG_H

#define _SUPPRESS_PLIB_WARNING 1

#include "plib.h"
// serial stuff
#include <stdio.h>

// Configuration Bit settings
// SYSCLK = 40 MHz (8MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 40 MHz
// Primary Osc w/PLL (XT+,HS+,EC+PLL)
// WDT OFF
// Other options are don't care
//                       8MHZ                          4MHz  
#pragma config FNOSC = FRCPLL, POSCMOD = HS, FPLLIDIV = DIV_2
//                        80MHz            40      <---    40MHz
#pragma config FPLLMUL = MUL_20, FPBDIV = DIV_1, FPLLODIV = DIV_2
#pragma config FWDTEN = OFF
#pragma config FSOSCEN = OFF, JTAGEN = OFF, DEBUG = OFF

#endif	/* CONFIG_H */

