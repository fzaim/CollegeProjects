/*Authors: Farjad Zaim (fzaim), Stephen Chong (schong)
 *Course: 18-348, Embedded System Engineering
 *Project: Lab 11, Real-Time Operating System
 *Items Included: PWM, Counter/Timer, Fixed Point Multiply,
 *   Timer ISR, LCD Display Output, Watchdog
 *Microprocessor: Freescale MC9S12C128
*/
#include <hidef.h>         /* common macros and defines */
#include <mc9s12c128.h>     /* derivative information */
#pragma LINK_INFO DERIVATIVE "mc9s12c128"

#include <stdio.h>
#include "lcd_lib.h"
#include "modclock.h"
#include "main_asm.h"

unsigned long timerCount;
char timerBuf[3];
char scoreBuf[9];
unsigned char chase_counter;
unsigned char pattern;
unsigned int gameOver;
unsigned int patternSet;
unsigned int win;
unsigned int difficultySet;
unsigned long winningScore;
unsigned char difficulty;
unsigned long score;

unsigned long multiply(unsigned int A, unsigned int B);
void youWin(void);
void youLose(void);
void readButtons(void);
void showPattern(void);
void readSwitch(void);
void setupTimer(void);
void writeTime(void);
void kickDog(void);

void main(void) {
  
  /* Set Port A as Input */
  DDRA = 0x00;
  
  /* Set Port B as Input */
  DDRB = 0x00;
  
  //set bus clock to 8 MHz
  clockSetup();
    
  //set up LCD for writing
  lcdSetup();
  
  //set up Timer
  setupTimer();
  
  COPCTL = 0x44;
    
  /************************************
   *       PWM CONGIFURATION          *
   ************************************/
  /*
  Configure Port P as follows:
    PTP0 as PWM0 output
    PTT1-7 as regular digital outputs
  Configure PWM0 as follows:
    Clock rate is 500 kHz  EDIT:  kHz not MHz
    No concatenation
    Left aligned PWM
    Period = 0xFF
    Default duty cycle = 0x80
  */

  // Port T as output
  DDRT = 0xFF;
  
  // PTT0 as PWM0, PTT1-7 as regular
  MODRR = 0x01;
  PWME = 0x01;
  
  // PWM0 configurations
  PWME = 0x01;
  PWMCLK = 0x00;
  PWMPRCLK = 0x04;
  PWMCAE = 0x00;   
  PWMCTL = 0x00;
  PWMPER0 = 0xFF;
  PWMDTY0 = 0x80;
  
  // Initialize variables
  chase_counter = 0;
  gameOver = 0;
  winningScore = 100000;
  difficulty = 0;
  patternSet = 0;
  difficultySet = 0;
  score = 0;
  win = 0;

	EnableInterrupts;

  for(;;) {
    if (gameOver == 0) {
      if (win == 1) {
        youWin();
      } else {
        readSwitch();
        showPattern();
        readButtons();
        writeTime();
      }
    } else {
      youLose();
    }
    kickDog();
  } /* loop forever */
  /* please make sure that you never leave main */
}

/* updates Score */
void updateScore(void) {
  
  switch (difficulty) {
    case 0: score = score + multiply(25, 100);
            break;
    case 1: score = score + multiply(50, 100);
            break;
    case 2: score = score + multiply(75, 100);
            break;
    case 3: score = score + multiply(100, 100);
            break;
  }
  
  if (score >= winningScore) {
    win = 1;
  }
  
}

/* Fixed Point Multiply */
unsigned long multiply(unsigned int A, unsigned int B){

  unsigned long Ahi, Alo, Bhi, Blo;
  unsigned long result;
  
  Ahi = ((A >> 8) & 0xFF);
  Alo = (A & 0xFF);
  Bhi = ((B >> 8) & 0xFF);
  Blo = (B & 0xFF);
  
  result = Alo * Blo;
  result += (Alo * Bhi) << 8;
  result += (Blo * Ahi) << 8;
  result += (Ahi * Bhi) << 16; 
  
  return result;
  
}

/* Win case */
void youWin(void) {
  
  lcdWriteLine(1, "You");
  lcdWriteLine(2, "Win");
    
} 

/* Game Over case */
void youLose(void) {
  
  lcdWriteLine(1, "Game");
  lcdWriteLine(2, "Over");
    
} 

/* Reads input pattern */
void readButtons(void) {

  unsigned char input;
  unsigned long count;
  unsigned int seconds;
  
  DisableInterrupts;
  
  count = timerCount;
  
  EnableInterrupts;
  
  seconds = count >> 16;
  input = PORTB;
  
  if (seconds <= 10) {
    switch (input) {
      case 0xFE: if (pattern == 0) {
                   DisableInterrupts;
                   timerCount = 0xF0000;
                   EnableInterrupts;
                   patternSet = 0;
                   difficultySet = 0;
                   updateScore();
                 } else {
                   gameOver = 1;
                 }
                 break;
      case 0xFD: if (pattern == 1) {
                   DisableInterrupts;
                   timerCount = 0xF0000;
                   EnableInterrupts;
                   patternSet = 0;
                   difficultySet = 0;
                   updateScore();
                 } else {
                   gameOver = 1;
                 }
                 break;
      case 0xFB: if (pattern == 2) {
                   DisableInterrupts;
                   timerCount = 0xF0000;
                   EnableInterrupts;
                   patternSet = 0;
                   difficultySet = 0;
                   updateScore();
                 } else {                  
                   gameOver = 1;
                 }
                 break;
      case 0xF7: if (pattern == 3) {
                   DisableInterrupts;
                   timerCount = 0xF0000;
                   EnableInterrupts;
                   patternSet = 0;
                   difficultySet = 0;
                   updateScore();
                 } else {
                   gameOver = 1;
                 }
                 break;
      case 0xEF: if (pattern == 4) {
                   DisableInterrupts;
                   timerCount = 0xF0000;
                   EnableInterrupts;
                   patternSet = 0;
                   difficultySet = 0;
                   updateScore();
                 } else {
                   gameOver = 1;
                 }
                 break;
      case 0xDF: if (pattern == 5) {
                   DisableInterrupts;
                   timerCount = 0xF0000;
                   EnableInterrupts;
                   patternSet = 0;
                   difficultySet = 0;
                   updateScore();
                 } else {
                   gameOver = 1;
                 }
                 break;
      case 0xBF: if (pattern == 6) {
                   DisableInterrupts;
                   timerCount = 0xF0000;
                   EnableInterrupts;
                   patternSet = 0;
                   difficultySet = 0;
                   updateScore();
                 } else {
                   gameOver = 1;
                 }
                 break;
      case 0x7F: if (pattern == 7) {
                   DisableInterrupts;
                   timerCount = 0xF0000;
                   EnableInterrupts;
                   patternSet = 0;
                   difficultySet = 0;
                   updateScore();
                 } else {                   
                   gameOver = 1;
                 }
                 break;
      default:   break;
    }
  }

}

/* Shows pattern on LCD */
void showPattern(void) {
  unsigned long count;
  unsigned int seconds;
  
  if (patternSet == 0) {
    DisableInterrupts;
  
    pattern = chase_counter;
    
    EnableInterrupts;
    
    patternSet = 1;
  }
  
  DisableInterrupts;
  
  count = timerCount;
  
  
  EnableInterrupts;
  
  seconds = count >> 16;
  
  if (seconds > 10) {
    PTT &= 0xE3;
    PTT |= (pattern << 2);
  } else {
    PWMDTY0 = 0xFF;
  }
  
}

/* Changes duty cycle based on switches */
void readSwitch(void) {

  unsigned long count;
  unsigned int seconds;
  
  if (difficultySet == 0) {
  
    difficulty = 0x03 & PORTA;
  
    DisableInterrupts;
  
    count = timerCount;
  
    EnableInterrupts;
  
    seconds = count >> 16;
  
    if (seconds > 10) { 
      switch(difficulty) {
        case 0: PWMDTY0 = 0x40;
                break;
        case 1: PWMDTY0 = 0x80;
                break;
        case 2: PWMDTY0 = 0xC0;
                break;
        case 3: PWMDTY0 = 0xF0;
                break;
      }
    }
    
    difficultySet = 1;
    
  }
  
}

/* Writes time on LCD */
void writeTime(void){
  unsigned long count;
  unsigned int seconds;

    
  DisableInterrupts;
  
  count = timerCount;
  
  EnableInterrupts;
  
  seconds = timerCount >> 16;
  
  if (seconds > 10) {
    lcdWriteLine(2, "");
  } else {
    sprintf(timerBuf, "%02d", seconds);
    lcdWriteLine(2, timerBuf);
  }
  
  sprintf(scoreBuf, "%Lu", score);
  lcdWriteLine(1, scoreBuf);
    
}

/* Kick the watchdog */
void kickDog(void) {
  ARMCOP = 0x55;
  ARMCOP = 0xAA;
}

/*********************
 *     Timer Task
 *********************/
//setup for timer tasks
void setupTimer( void ) {

/*TODO:  Setup timer */
  
	/* timer will operate at 0.5 MHz, enable TOI interrupt*/
	TSCR2 = 0x84;

	/* timer will stop in freeze and wait modes */
	TSCR1 = 0xE0;
	
	/* initialize global var timerCount to 15 sec */
	timerCount = 0xF0000;
	

}

/*****************************************
 *    Interrupt Service Routines
 *****************************************/
 
/*
  timer ISR
  
  Increment the timer counter.
*/
void interrupt 16 timerHandler(void) {

   /* decrease timerCount */
   timerCount -= 0x218E;
   
   /* check for running out of time */
   if (timerCount & 0xFFFF0000) { 
   } else {
     gameOver = 1;
   }
   /* acknowledge interrupt */
   TFLG2 = 0x80;
   
   chase_counter++;
   chase_counter &= 0x07;
   
}

/*
   Watchdog Reset
*/
void interrupt 2 watchdogHandler(void) {
  
   lcdSetup();
   
   for (;;) {
    lcdWriteLine(1, "Watchdog");
    lcdWriteLine(2, "Reset");
   }
   
}
