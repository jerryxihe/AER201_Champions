#include <xc.h>
#include <stdio.h>
#include <stdbool.h>
#include "configBits.h"
#include "lcd.h"
#include "I2C.h"
#include "I2C.c"

// Struct that contains info for each pole
typedef struct{
    unsigned char tiresDeployed;
    unsigned char tiresAfter;
    unsigned int dist;    // in mm
} pole;

// Struct that contains info for each run
typedef struct{
    unsigned char totalPoleNum;
    unsigned char opTime;   // in seconds
    unsigned char tiresSupplied;
    pole poleArray[10];
} run;

volatile unsigned char i;
volatile unsigned char keypress;
volatile int startTime[7];
volatile int endTime[7];
const unsigned int magazineStepperDelay = 1300;   //1400 microseconds
const unsigned int singleTireSteps = 3925;
const unsigned char maxPoles = 10;

// Used to set RTC
const char inputTime[7] = {
    0x00, // 0 Seconds 
    0x47, // 59 Minutes
    0x1, // 24 hour mode, set to 17:00
    0x01, // Sunday
    0x09, // 7th
    0x04, // April
    0x19  // 2019
};

void writeByteToDriveArduino(unsigned char byteToWrite){
	__delay_us(100);	// Allow Uno to drive unoBusy HIGH
	while (PORTCbits.RC2){}	// Wait until Uno isn't busy
    I2C_Master_Start(); // Start condition
    I2C_Master_Write(0b00010000); // 7 bit Arduino address + Write
    I2C_Master_Write(byteToWrite);
    I2C_Master_Stop(); // Stop condition
    return;
}

void mainMenu(unsigned char *menu){
    lcd_clear();
    lcd_set_ddram_addr(LCD_LINE3_ADDR);
    printf("(A)Start");
    lcd_set_ddram_addr(LCD_LINE4_ADDR);
    printf("(B)Logs (C)Setup");
    unsigned char time[7]; // Create a byte array to hold time read from RTC
	LATCbits.LATC5 = 1; // Tell Nano run finished
    while (!PORTBbits.RB1){ // Prints time until key is pressed
        // Reset RTC memory pointer
        I2C_Master_Start(); // Start condition
        I2C_Master_Write(0b11010000); // 7 bit RTC address + Write
        I2C_Master_Write(0x00); // Set memory pointer to seconds
        I2C_Master_Stop(); // Stop condition

        // Read current time
        I2C_Master_Start(); // Start condition
        I2C_Master_Write(0b11010001); // 7 bit RTC address + Read
        for(unsigned char i = 0; i < 6; i++){
            time[i] = I2C_Master_Read(ACK); // Read with ACK to continue reading
        }
        time[6] = I2C_Master_Read(NACK); // Final Read with NACK
        I2C_Master_Stop(); // Stop condition
        lcd_set_ddram_addr(LCD_LINE1_ADDR);
        printf("20%02x-%02x-%02x", time[6],time[5],time[4]); // Print date in YYYY/MM/DD
        lcd_set_ddram_addr(LCD_LINE2_ADDR);
        printf("%02x:%02x:%02x", time[2],time[1],time[0]); // HH:MM:SS
		if (PORTAbits.RA1){	// Bluetooth start
            // Reset RTC memory pointer
            I2C_Master_Start(); // Start condition
            I2C_Master_Write(0b11010000); // 7 bit RTC address + Write
            I2C_Master_Write(0x00); // Set memory pointer to seconds
            I2C_Master_Stop(); // Stop condition

            // Read current time
            I2C_Master_Start(); // Start condition
            I2C_Master_Write(0b11010001); // 7 bit RTC address + Read
            for(unsigned char i = 0; i < 6; i++){
                startTime[i] = I2C_Master_Read(ACK); // Read with ACK to continue reading
            }
            startTime[6] = I2C_Master_Read(NACK); // Final Read with NACK
            I2C_Master_Stop(); // Stop condition
            
			*menu = 1;
			return;
		}
    }
    keypress = (PORTB & 0xF0) >> 4; // Stores keypad info into keypress
    switch (keypress){
        case 0b0011:    // (A)
            // Reset RTC memory pointer
            I2C_Master_Start(); // Start condition
            I2C_Master_Write(0b11010000); // 7 bit RTC address + Write
            I2C_Master_Write(0x00); // Set memory pointer to seconds
            I2C_Master_Stop(); // Stop condition

            // Read current time
            I2C_Master_Start(); // Start condition
            I2C_Master_Write(0b11010001); // 7 bit RTC address + Read
            for(unsigned char i = 0; i < 6; i++){
                startTime[i] = I2C_Master_Read(ACK); // Read with ACK to continue reading
            }
            startTime[6] = I2C_Master_Read(NACK); // Final Read with NACK
            I2C_Master_Stop(); // Stop condition
            
            *menu = 1;  // Start
            break;
        case 0b0111:    // (B)
            *menu = 6;  // Log menu
            break;
        case 0b1011:    //(C)
            *menu = 8;  // Options menu
            break;
    }
    while (PORTBbits.RB1){} // Waits until key is released
}

void topMagStepper(unsigned int steps, unsigned char forward){
    unsigned int topMagCurrentStep = 0;
    
    if (forward){
        topMagCurrentStep = 0;
    } else {
        topMagCurrentStep = steps;
    }
    while ((forward && (topMagCurrentStep < steps)) || (!forward && (topMagCurrentStep > 0))){
        if (topMagCurrentStep % 4 == 0){
            LATBbits.LATB0 = 1;
            LATBbits.LATB2 = 1;
            LATBbits.LATB4 = 0;
            LATBbits.LATB6 = 0;
            if (forward){
                topMagCurrentStep++;
            } else {
                topMagCurrentStep--;
            }
        } else if (topMagCurrentStep % 4 == 1){
            LATBbits.LATB0 = 0;
            LATBbits.LATB2 = 1;
            LATBbits.LATB4 = 1;
            LATBbits.LATB6 = 0;
            if (forward){
                topMagCurrentStep++;
            } else {
                topMagCurrentStep--;
            }
        } else if (topMagCurrentStep % 4 == 2){
            LATBbits.LATB0 = 0;
            LATBbits.LATB2 = 0;
            LATBbits.LATB4 = 1;
            LATBbits.LATB6 = 1;
            if (forward){
                topMagCurrentStep++;
            } else {
                topMagCurrentStep--;
            }
        } else if (topMagCurrentStep % 4 == 3){
            LATBbits.LATB0 = 1;
            LATBbits.LATB2 = 0;
            LATBbits.LATB4 = 0;
            LATBbits.LATB6 = 1;
            if (forward){
                topMagCurrentStep++;
            } else {
                topMagCurrentStep--;
            }
        }
        __delay_us(magazineStepperDelay);
    }
    LATBbits.LATB0 = 0;
    LATBbits.LATB2 = 0;
    LATBbits.LATB4 = 0;
    LATBbits.LATB6 = 0;
}


void bottomMagStepper(unsigned int steps, unsigned char forward){
    unsigned int bottomMagCurrentStep = 0;
    
    if (forward){
        bottomMagCurrentStep = 0;
    } else {
        bottomMagCurrentStep = steps;
    }
    while ((forward && (bottomMagCurrentStep < steps)) || (!forward && (bottomMagCurrentStep > 0))){
        if (bottomMagCurrentStep % 4 == 0){
            LATBbits.LATB1 = 1;
            LATBbits.LATB3 = 1;
            LATBbits.LATB5 = 0;
            LATBbits.LATB7 = 0;
            if (forward){
                bottomMagCurrentStep++;
            } else {
                bottomMagCurrentStep--;
            }
        } else if (bottomMagCurrentStep % 4 == 1){
            LATBbits.LATB1 = 0;
            LATBbits.LATB3 = 1;
            LATBbits.LATB5 = 1;
            LATBbits.LATB7 = 0;
            if (forward){
                bottomMagCurrentStep++;
            } else {
                bottomMagCurrentStep--;
            }
        } else if (bottomMagCurrentStep % 4 == 2){
            LATBbits.LATB1 = 0;
            LATBbits.LATB3 = 0;
            LATBbits.LATB5 = 1;
            LATBbits.LATB7 = 1;
            if (forward){
                bottomMagCurrentStep++;
            } else {
                bottomMagCurrentStep--;
            }
        } else if (bottomMagCurrentStep % 4 == 3){
            LATBbits.LATB1 = 1;
            LATBbits.LATB3 = 0;
            LATBbits.LATB5 = 0;
            LATBbits.LATB7 = 1;
            if (forward){
                bottomMagCurrentStep++;
            } else {
                bottomMagCurrentStep--;
            }
        }
        __delay_us(magazineStepperDelay);
    }
    LATBbits.LATB1 = 0;
    LATBbits.LATB3 = 0;
    LATBbits.LATB5 = 0;
    LATBbits.LATB7 = 0;
}

void start(unsigned char *menu, run *currentRun, unsigned char *topMagTires, unsigned char *bottomMagTires){
    unsigned int distTravelled = 0;
    float floatDistTravelled = 0;
    unsigned char tiresBefore = 0;
    unsigned char tiresNeeded = 0;
    unsigned char currentTires = 0;
    bool tireInCell = true;
    unsigned char currentPole = 0;
    bool armReached = true;
	bool bottomMagPushed = true;
	bool tireOnBottomRamp = true;
    
    currentRun->totalPoleNum = 0;
    LATCbits.LATC1 = 1;     // Disables keypad by setting KPD to HIGH
    TRISB = 0x00;
    LATB = 0x00;
	lcd_clear();
    // Command 1: Reset encoder and timer 
    writeByteToDriveArduino(1);
	LATCbits.LATC5 = 0; // Tell Nano run began

    while (1){
        if ((*topMagTires <= 0) && (*bottomMagTires <= 0) && !tireInCell && !tireOnBottomRamp){      
            *menu = 2;
            return;
        }

//        __delay_us(100);    // Allow Uno to drive runFinished HIGH
        if (PORTCbits.RC0){
            *menu = 2;  // Run finished
            return;
        }
		
        // Command 2: Normal speed forward
        writeByteToDriveArduino(2);
        __delay_ms(350);    // Slight delay so laser doesn't detect same pole
        // Nano starts polling laser
        LATAbits.LATA4 = 1;
        __delay_ms(5);
        LATAbits.LATA4 = 0;
        
        if (PORTCbits.RC0){
            LATCbits.LATC5 = 1; // Tell Nano run finished
            *menu = 2;  // Run finished
            return;
        }
        
        if (!tireInCell && !tireOnBottomRamp){
            if (*bottomMagTires > 0){
                bottomMagStepper(singleTireSteps, true);
                (*bottomMagTires)--;
				bottomMagPushed = true;
            } else if (*topMagTires > 0){
                topMagStepper(singleTireSteps, true);
                (*topMagTires)--;
				bottomMagPushed = false;
            }
		}
        
        if (PORTCbits.RC0){
            LATCbits.LATC5 = 1; // Tell Nano run finished
            *menu = 2;  // Run finished
            return;
        }
        
        // Wait until laser detects a pole
        while (!PORTAbits.RA5){
            if (PORTCbits.RC0){
                LATCbits.LATC5 = 1; // Tell Nano run finished
                *menu = 2;  // Run finished
                return;
            }
        }
        
		// Inputs run data
		(currentRun->totalPoleNum)++;
		currentPole = currentRun->totalPoleNum - 1;
			
		// Read encoder data
		I2C_Master_Start();
		I2C_Master_Write(0b00010001); // 7-bit Arduino slave address + Read
		distTravelled = ((I2C_Master_Read(ACK) << 8) & 0xff00) | (I2C_Master_Read(NACK) & 0xff); // Read distance travelled
		I2C_Master_Stop(); // Stop condition
		floatDistTravelled = distTravelled * 0.469 - 8.7;
		currentRun->poleArray[currentPole].dist = (unsigned int)(floatDistTravelled*10);
		lcd_set_ddram_addr(LCD_LINE1_ADDR);
		printf("Distance: %hu.%hucm", currentRun->poleArray[currentPole].dist/10, currentRun->poleArray[currentPole].dist%10);
			
		currentRun->poleArray[currentPole].tiresAfter = 0;
		currentRun->poleArray[currentPole].tiresDeployed = 0;
		
        __delay_us(100);    // Allow Uno to drive runFinished HIGH
        if (PORTCbits.RC0){
            *menu = 2;  // Run finished
            return;
        }
		
		// Operate whacker
		if (!tireInCell){
			if (tireOnBottomRamp){
				LATEbits.LATE2 = 1;
				__delay_ms(10);
				LATEbits.LATE2 = 0;
				tireOnBottomRamp = false;
			} else if (bottomMagPushed){    // Bottom
				LATEbits.LATE2 = 1;
				__delay_ms(10);
				LATEbits.LATE2 = 0;
			} else {
				LATDbits.LATD1 = 1;
				__delay_ms(10);
				LATDbits.LATD1 = 0;
			}
			__delay_ms(2050);
			tireInCell = true;
		}
        
        LATAbits.LATA3 = 0; // Drive stopExtending low
        
        __delay_us(100);    // Allow Uno to drive runFinished HIGH
        if (PORTCbits.RC0){
            *menu = 2;  // Run finished
            return;
        }
		
        // Command 4: Extend arm
        writeByteToDriveArduino(4);
        __delay_ms(500);    // Doesn't detect laser sensor
        
        armReached = true;
        // While pole beam isn't broken
        while (PORTAbits.RA2){
            if (!PORTCbits.RC2){
                armReached = false;
                break;
            }
            if (PORTCbits.RC0){
                *menu = 2;  // Run finished
                return;
            }
        }
        LATAbits.LATA3 = 1; // Stop extending arm
        
        __delay_us(100);    // Allow Uno to drive runFinished HIGH
        if (PORTCbits.RC0){
            *menu = 2;  // Run finished
            return;
        }
        
        while (PORTCbits.RC2){}	// Wait until arm fully extends
		
        if (armReached){
            if (!PORTEbits.RE1){
                tiresBefore = 2;
            } else if (!PORTEbits.RE0){
                tiresBefore = 1;
            } else {
                tiresBefore = 0;
            }
			lcd_clear();
            lcd_set_ddram_addr(LCD_LINE2_ADDR);
            printf("Before: %u", tiresBefore);
			
			// If first pole
			if ((currentPole) == 0){
				tiresNeeded = 2;
			} else if ((currentRun->poleArray[currentPole].dist - currentRun->poleArray[currentPole - 1].dist) < 300){
				tiresNeeded = 1;
			} else if ((currentRun->poleArray[currentPole].dist - currentRun->poleArray[currentPole - 1].dist) >= 300){
				tiresNeeded = 2;
			}
			
            __delay_us(100);    // Allow Uno to drive runFinished HIGH
			if (PORTCbits.RC0){
				*menu = 2;  // Run finished
				return;
			}
            // Command 7: Position cell
            writeByteToDriveArduino(7);
            
            __delay_us(100);    // Allow Uno to drive unoBusy HIGH

            while (PORTCbits.RC2){}	// Wait until cell properly positioned
			
			__delay_us(100);    // Allow Uno to drive runFinished HIGH
            if (PORTCbits.RC0){
				*menu = 2;  // Run finished
				return;
			}
            
            if (tiresBefore >= tiresNeeded){
                currentRun->poleArray[currentPole].tiresAfter = tiresBefore;
                currentRun->poleArray[currentPole].tiresDeployed = 0;
            } else {
                currentTires = tiresBefore;
                if ((tiresNeeded - tiresBefore) == 1){
                    // Operate cell
                    LATAbits.LATA0 = 1;
                    __delay_ms(10);
                    LATAbits.LATA0 = 0;
                    __delay_ms(1150);
                    currentTires++;
                    tireInCell = false;
                } else if ((tiresNeeded - tiresBefore) >= 2){
                    // Operate cell
                    LATAbits.LATA0 = 1;
                    __delay_ms(10);
                    LATAbits.LATA0 = 0;
                    currentTires++;
					tireInCell = false;
					if (PORTCbits.RC0){
						currentRun->poleArray[currentPole].tiresAfter = currentTires;
						currentRun->poleArray[currentPole].tiresDeployed = currentTires - tiresBefore;
                        __delay_ms(1150);
						*menu = 2;  // Run finished
                        return;
                    }
					if (tireOnBottomRamp){
                        __delay_ms(1250);
						LATEbits.LATE2 = 1;
						__delay_ms(10);
						LATEbits.LATE2 = 0;
						tireOnBottomRamp = false;
					} else if (*bottomMagTires > 0){
                        bottomMagStepper(singleTireSteps, true);
						// Operate whacker
						LATEbits.LATE2 = 1;
						(*bottomMagTires)--;
						__delay_ms(10);
						LATEbits.LATE2 = 0;
                    } else if (*topMagTires > 0){
                        topMagStepper(singleTireSteps, true);
						// Operate whacker
						LATDbits.LATD1 = 1;
						(*topMagTires)--;
						__delay_ms(10);
						LATDbits.LATD1 = 0;
                    } else {
                        currentRun->poleArray[currentPole].tiresAfter = currentTires;
                        currentRun->poleArray[currentPole].tiresDeployed = currentTires - tiresBefore;
                        __delay_ms(1150);    // Allows the Nano to operate cell and the Uno to drive unoBusy high
                        *menu = 2;
                        return;
                    }
					__delay_ms(2050);
                    // Operate cell
                    LATAbits.LATA0 = 1;
                    __delay_ms(10);
                    LATAbits.LATA0 = 0;
                    __delay_ms(1150);
                    currentTires++;
                }
                currentRun->poleArray[currentPole].tiresAfter = currentTires;
                currentRun->poleArray[currentPole].tiresDeployed = currentTires - tiresBefore;
            }
        }

        // Command 5: Retract arm
        writeByteToDriveArduino(5);
		
		while (PORTCbits.RC2){}	// Wait until arm fully retracts
		
		__delay_us(100);    // Allow Uno to drive runFinished HIGH
        if (PORTCbits.RC0){
            *menu = 2;  // Run finished
            return;
        }
        
        if (currentRun->totalPoleNum >= maxPoles){
            *menu = 2;  // Run finished
            return;
        }
    }
}

unsigned char bcdToDec(unsigned char val){
  return( (val/16*10) + (val%16) );
}

void runFinished(unsigned char *menu, run *currentRun, unsigned char *polePage, run *displayRun, bool *logs){
    int startSec = 0;
    int endSec = 0;
	// Command 6: Return to start
	writeByteToDriveArduino(6);
    lcd_clear();
    lcd_set_ddram_addr(LCD_LINE1_ADDR);
    printf("Finished");
    // Nano stops polling laser
    LATAbits.LATA4 = 0;
    LATCbits.LATC1 = 0;     // Enables keypad by setting KPD to LOW
    TRISB = 0xff;
    LATB = 0x00;
    currentRun->tiresSupplied = 0;
    for (i=0; i<currentRun->totalPoleNum; i++){
        currentRun->tiresSupplied = currentRun->tiresSupplied + currentRun->poleArray[i].tiresDeployed;
    }
	// Waits for Uno to return to start
    while (PORTCbits.RC2){}
    // Reset RTC memory pointer
    I2C_Master_Start(); // Start condition
    I2C_Master_Write(0b11010000); // 7 bit RTC address + Write
    I2C_Master_Write(0x00); // Set memory pointer to seconds
    I2C_Master_Stop(); // Stop condition

    // Read current time
    I2C_Master_Start(); // Start condition
    I2C_Master_Write(0b11010001); // 7 bit RTC address + Read
    for(unsigned char i = 0; i < 6; i++){
        endTime[i] = I2C_Master_Read(ACK); // Read with ACK to continue reading
    }
    endTime[6] = I2C_Master_Read(NACK); // Final Read with NACK
    I2C_Master_Stop(); // Stop condition
    
    startSec = bcdToDec(startTime[0]) + bcdToDec(startTime[1])*60 + bcdToDec(startTime[2])*3600;
    endSec = bcdToDec(endTime[0]) + bcdToDec(endTime[1])*60 + bcdToDec(endTime[2])*3600;
    currentRun->opTime = (unsigned char)(endSec - startSec);
    *polePage = 0;  // Sets first pole as default
    *logs = false;
    *displayRun = *currentRun;
	
    *menu = 7;  // Run finished menu
}

void runFinishedMenu(unsigned char *menu, bool *logs){
    lcd_clear();
    lcd_set_ddram_addr(LCD_LINE1_ADDR);
    printf("Run completed!");
    lcd_set_ddram_addr(LCD_LINE2_ADDR);
    printf("(B) Run info");
    lcd_set_ddram_addr(LCD_LINE3_ADDR);
    printf("(C) Save run");
    lcd_set_ddram_addr(LCD_LINE4_ADDR);
    printf("(D) Main menu");    
	
    while (!PORTBbits.RB1){}
    keypress = (PORTB & 0xF0) >> 4;
    switch (keypress){
        case 0b0111:    // (B)
            *menu = 3;  // Run info menu
            break;
        case 0b1011:    // (C)
            *menu = 5;  // Save run menu
            break;
        case 0b01111:   // (D)
            *logs = true;
            *menu = 0;  // Main menu
            break;
    }
    while (PORTBbits.RB1){}
}

void runInfoMenu(unsigned char *menu, run *displayRun, bool logs){
    lcd_clear();
    lcd_set_ddram_addr(LCD_LINE1_ADDR);
    printf("Op time:%u:", displayRun->opTime/60);
    if (displayRun->opTime%60 < 10){
        printf("0");
    }
    printf("%u", displayRun->opTime%60);
    lcd_set_ddram_addr(LCD_LINE2_ADDR);
    printf("# supplied:%u", displayRun->tiresSupplied);
    lcd_set_ddram_addr(LCD_LINE3_ADDR);
    printf("# of poles:%u", displayRun->totalPoleNum);
    lcd_set_ddram_addr(LCD_LINE4_ADDR);
    if (displayRun->totalPoleNum != 0){
        printf("(C)Poles ");
    }
    printf("(D)Back");
    while (!PORTBbits.RB1){}
    keypress = (PORTB & 0xF0) >> 4;
    switch (keypress){
        case 0b1011:    // (C)
            if (displayRun->totalPoleNum != 0){
                *menu = 4;   // Pole Info
            }
            break;
        case 0b01111:   // (D)
            if (logs){
                *menu = 6;  // Select log menu
            } else {
                *menu = 7;   // Run finished menu
            }
            break;
    }
    while (PORTBbits.RB1){}
}

void poleInfo(unsigned char *menu, run *displayRun, unsigned char *polePage){
    lcd_clear();
    lcd_set_ddram_addr(LCD_LINE1_ADDR);
    printf("Pole #%u", *polePage + 1);
    lcd_set_ddram_addr(LCD_LINE2_ADDR);
    printf("Deploy:%u After:%u", displayRun->poleArray[*polePage].tiresDeployed, displayRun->poleArray[*polePage].tiresAfter);
    lcd_set_ddram_addr(LCD_LINE3_ADDR);
    printf("Distance:%hu.%hucm", displayRun->poleArray[*polePage].dist/10, displayRun->poleArray[*polePage].dist%10);
    lcd_set_ddram_addr(LCD_LINE4_ADDR);
    printf("<*  (D)Back   #>");
    while (!PORTBbits.RB1){}
    keypress = (PORTB & 0xF0) >> 4;
    switch (keypress){
        case 0b1111:    // (D)
            *menu = 3;   // Run Info
            break;
        case 0b01100:   // (*)
            if (*polePage == 0){
                *polePage = displayRun->totalPoleNum - 1;   // Pressing previous on first pole goes to last pole
            } else {
                *polePage = *polePage - 1;  // Previous Page
            }
            break;
        case 0b01110:   // (#)
            if (*polePage == displayRun->totalPoleNum - 1){ // Pressing next on last pole goes to first pole
                *polePage = 0;
            } else{
                *polePage = *polePage + 1;  // Next Page
            }
            break;
    }
    while (PORTBbits.RB1){}
}

void Write_eep( unsigned int badd,unsigned char bdat ){
	char GIE_BIT_VAL = 0;
	EEADRH = (badd >> 8) & 0x03;
	EEADR = (badd & 0x0ff);
	EEDATA = bdat;
  	EECON1bits.EEPGD = 0;
	EECON1bits.CFGS = 0;
	EECON1bits.WREN = 1;
	GIE_BIT_VAL = INTCONbits.GIE;
	INTCONbits.GIE = 0;
	EECON2 = 0x55;
	EECON2 = 0xAA;
	EECON1bits.WR = 1;
	while(EECON1bits.WR);				//Wait till the write completion
	INTCONbits.GIE = GIE_BIT_VAL;
	EECON1bits.WREN = 0;
}

unsigned char Read_eep( unsigned int badd ){
	EEADRH = (badd >> 8) & 0x03;
	EEADR = (badd & 0x0ff);
  	EECON1bits.CFGS = 0;
	EECON1bits.EEPGD = 0;
	EECON1bits.RD = 1;
	Nop();							//Nop may be required for latency at high frequencies
	Nop();							//Nop may be required for latency at high frequencies
	return ( EEDATA );              // return with read byte 
}

void saveRun(unsigned char *menu, run *currentRun){
    lcd_clear();
    lcd_set_ddram_addr(LCD_LINE1_ADDR);
    printf("Choose log to");
    lcd_set_ddram_addr(LCD_LINE2_ADDR);
    printf("overwrite:");
    lcd_set_ddram_addr(LCD_LINE3_ADDR);
    printf("(1) (2) (3) (4)");
    lcd_set_ddram_addr(LCD_LINE4_ADDR);
    printf("(D) Back");
    while (!PORTBbits.RB1){}
    keypress = (PORTB & 0xF0) >> 4;
    switch (keypress){
        case 0b0000:    // (1)
            for (i=0;i<sizeof(run);i++){
                Write_eep(i, *((char *)currentRun + i));
            }
            *menu = 7;  // Run finished menu
            break;
        case 0b0001:    // (2)
            for (i=0;i<sizeof(run);i++){
                Write_eep(sizeof(run) + i, *((char *)currentRun + i));
            }
            *menu = 7;  // Run finished menu
            break;
        case 0b0010:    // (3)
            for (i=0;i<sizeof(run);i++){
                Write_eep(2*sizeof(run) + i, *((char *)currentRun + i));
            }
            *menu = 7;  // Run finished menu
            break;
        case 0b0100:    // (4)
            for (i=0;i<sizeof(run);i++){
                Write_eep(3*sizeof(run) + i, *((char *)currentRun + i));
            }
            *menu = 7;  // Run finished menu
            break;
        case 0b01111:   // (D)
            *menu = 2;  // Run finished menu
            break;
    }
    while (PORTBbits.RB1){}
}

void selectLog(unsigned char *menu, run *displayRun){
    lcd_clear();
    lcd_set_ddram_addr(LCD_LINE1_ADDR);
    printf("Choose log to");
    lcd_set_ddram_addr(LCD_LINE2_ADDR);
    printf("view:");
    lcd_set_ddram_addr(LCD_LINE3_ADDR);
    printf("(1) (2) (3) (4)");
    lcd_set_ddram_addr(LCD_LINE4_ADDR);
    printf("(D) Main menu");
    while (!PORTBbits.RB1){}
    keypress = (PORTB & 0xF0) >> 4;
    switch (keypress){
        case 0b0000:    // (1)
            for (i=0;i<sizeof(run);i++){
                *((char *)displayRun + i) = Read_eep(i);
            }
            *menu = 3;  // Run info menu
            break;
        case 0b0001:    // (2)
            for (i=0;i<sizeof(run);i++){
                *((char *)displayRun + i) = Read_eep(sizeof(run) + i);
            }
            *menu = 3;  // Run info menu
            break;
        case 0b0010:    // (3)
            for (i=0;i<sizeof(run);i++){
                *((char *)displayRun + i) = Read_eep(2*sizeof(run) + i);
            }
            *menu = 3;  // Run info menu
            break;
        case 0b0100:    // (4)
            for (i=0;i<sizeof(run);i++){
                *((char *)displayRun + i) = Read_eep(3*sizeof(run) + i);
            }
            *menu = 3;  // Run info menu
            break;
        case 0b01111:   // (D)
            *menu = 0;  // Main menu
            break;
    }
    while (PORTBbits.RB1){}
}

void options(unsigned char *menu, unsigned char *topMagTires, unsigned char *bottomMagTires){
    unsigned char tens = 0;
    unsigned char ones = 0;
    unsigned char topMagTireMove;
    char topMagTireCheck;
    unsigned char bottomMagTireMove;
    char bottomMagTireCheck;
    bool forward = false;
    
    // Magazine 1
    lcd_clear();
    lcd_set_ddram_addr(LCD_LINE1_ADDR);
    printf("Top mag forward");
    lcd_set_ddram_addr(LCD_LINE2_ADDR);
    printf("or backward?");
    lcd_set_ddram_addr(LCD_LINE3_ADDR);
    printf("(#) Forward");
    lcd_set_ddram_addr(LCD_LINE4_ADDR);
    printf("(*) Backward");
    while (!PORTBbits.RB1){}
    keypress = (PORTB & 0xF0) >> 4;
    switch (keypress){
        case 0b1110:    // (#)
            forward = true;
            break;
        case 0b1100:    // (*)
            forward = false;
            break;
    }
    while (PORTBbits.RB1){}
    lcd_clear();
    lcd_set_ddram_addr(LCD_LINE1_ADDR);
    printf("Input tire");
    lcd_set_ddram_addr(LCD_LINE2_ADDR);
    printf("spaces to move");
    lcd_set_ddram_addr(LCD_LINE3_ADDR);
    printf("top mag (01-13):");
    while (!PORTBbits.RB1){}
    keypress = (PORTB & 0xF0) >> 4;
    switch (keypress){
        case 0b1101:    // (0)
            tens = 0;
            break;
        case 0b0000:    // (1)
            tens = 1;
            break;
    }
    while (PORTBbits.RB1){}
    lcd_set_ddram_addr(LCD_LINE4_ADDR);
    printf("%u", tens);
    while (!PORTBbits.RB1){}
    keypress = (PORTB & 0xF0) >> 4;
    switch (keypress){
        case 0b1101:    // (0)
            ones = 0;
            break;
        case 0b0000:    // (1)
            ones = 1;
            break;
        case 0b0001:    // (2)
            ones = 2;
            break;
        case 0b0010:    // (3)
            ones = 3;
            break;
        case 0b0100:    // (4)
            ones = 4;
            break;
        case 0b0101:    // (5)
            ones = 5;
            break;
        case 0b0110:    // (6)
            ones = 6;
            break;
        case 0b1000:    // (7)
            ones = 7;
            break;
        case 0b1001:    // (8)
            ones = 8;
            break;
        case 0b1010:    // (9)
            ones = 9;
            break;
    }
    while (PORTBbits.RB1){}
    topMagTireMove = tens*10 + ones;
    if (forward){
        topMagTireCheck = (*topMagTires) - topMagTireMove;
    } else {
        topMagTireCheck = (*topMagTires) + topMagTireMove;
    }
//    if (topMagTireCheck >= 0 && topMagTireCheck <= 12){
    if (true){
        *topMagTires = topMagTireCheck;
        lcd_clear();
        lcd_set_ddram_addr(LCD_LINE1_ADDR);
        printf("%u tire(s) can", *topMagTires);
        lcd_set_ddram_addr(LCD_LINE2_ADDR);
        printf("be loaded in");
        lcd_set_ddram_addr(LCD_LINE3_ADDR);
        printf("top mag");
        LATCbits.LATC1 = 1;     // Disables keypad by setting KPD to HIGH
        TRISB = 0x00;
        LATB = 0x00;
        topMagStepper(singleTireSteps*topMagTireMove, forward);
        LATCbits.LATC1 = 0;     // Enables keypad by setting KPD to LOW
        TRISB = 0xff;
        LATB = 0x00;
        if (topMagTireMove == 0){
            __delay_ms(1000);
        }
    } else {
        lcd_clear();
        lcd_set_ddram_addr(LCD_LINE1_ADDR);
        printf("Magazine can't");
        lcd_set_ddram_addr(LCD_LINE2_ADDR);
        printf("hold %u tires", topMagTireCheck);
        __delay_ms(1000);
    }
    
    // Magazine 2
    lcd_clear();
    lcd_set_ddram_addr(LCD_LINE1_ADDR);
    printf("Bot mag forward");
    lcd_set_ddram_addr(LCD_LINE2_ADDR);
    printf("or backward?");
    lcd_set_ddram_addr(LCD_LINE3_ADDR);
    printf("(#) Forward");
    lcd_set_ddram_addr(LCD_LINE4_ADDR);
    printf("(*) Backward");
    while (!PORTBbits.RB1){}
    keypress = (PORTB & 0xF0) >> 4;
    switch (keypress){
        case 0b1110:    // (#)
            forward = true;
            break;
        case 0b1100:    // (*)
            forward = false;
            break;
    }
    while (PORTBbits.RB1){}
    lcd_clear();
    lcd_set_ddram_addr(LCD_LINE1_ADDR);
    printf("Input tire");
    lcd_set_ddram_addr(LCD_LINE2_ADDR);
    printf("spaces to move");
    lcd_set_ddram_addr(LCD_LINE3_ADDR);
    printf("bot mag (01-13):");
    while (!PORTBbits.RB1){}
    keypress = (PORTB & 0xF0) >> 4;
    switch (keypress){
        case 0b1101:    // (0)
            tens = 0;
            break;
        case 0b0000:    // (1)
            tens = 1;
            break;
    }
    while (PORTBbits.RB1){}
    lcd_set_ddram_addr(LCD_LINE4_ADDR);
    printf("%u", tens);
    while (!PORTBbits.RB1){}
    keypress = (PORTB & 0xF0) >> 4;
    switch (keypress){
        case 0b1101:    // (0)
            ones = 0;
            break;
        case 0b0000:    // (1)
            ones = 1;
            break;
        case 0b0001:    // (2)
            ones = 2;
            break;
        case 0b0010:    // (3)
            ones = 3;
            break;
        case 0b0100:    // (4)
            ones = 4;
            break;
        case 0b0101:    // (5)
            ones = 5;
            break;
        case 0b0110:    // (6)
            ones = 6;
            break;
        case 0b1000:    // (7)
            ones = 7;
            break;
        case 0b1001:    // (8)
            ones = 8;
            break;
        case 0b1010:    // (9)
            ones = 9;
            break;
    }
    while (PORTBbits.RB1){}
    bottomMagTireMove = tens*10 + ones;
    if (forward){
        bottomMagTireCheck = (*bottomMagTires) - bottomMagTireMove;
    } else {
        bottomMagTireCheck = (*bottomMagTires) + bottomMagTireMove;
    }
//    if (bottomMagTireCheck >= 0 && bottomMagTireCheck <= 12){
    if (true){
        *bottomMagTires = bottomMagTireCheck;
        lcd_clear();
        lcd_set_ddram_addr(LCD_LINE1_ADDR);
        printf("%u tire(s) can", *bottomMagTires);
        lcd_set_ddram_addr(LCD_LINE2_ADDR);
        printf("be loaded in");
        lcd_set_ddram_addr(LCD_LINE3_ADDR);
        printf("bot mag");
        LATCbits.LATC1 = 1;     // Disables keypad by setting KPD to HIGH
        TRISB = 0x00;
        LATB = 0x00;
        bottomMagStepper((singleTireSteps)*bottomMagTireMove, forward);
        LATCbits.LATC1 = 0;     // Enables keypad by setting KPD to LOW
        TRISB = 0xff;
        LATB = 0x00;
        if (bottomMagTireMove == 0){
            __delay_ms(1000);
        }
    } else {
        lcd_clear();
        lcd_set_ddram_addr(LCD_LINE1_ADDR);
        printf("Magazine can't");
        lcd_set_ddram_addr(LCD_LINE2_ADDR);
        printf("hold %u tires", *bottomMagTires);
        __delay_ms(1000);
    }
    
    *menu = 0;
}

/** @brief Writes the inputTime array to the RTC memory */
void setTime(void){
    I2C_Master_Start(); // Start condition
    I2C_Master_Write(0b11010000); //7 bit RTC address + Write
    I2C_Master_Write(0x00); // Set memory pointer to seconds
    
    // Write array
    for(char i=0; i < 7; i++){
        I2C_Master_Write(inputTime[i]);
    }
    
    I2C_Master_Stop(); //Stop condition
}

void main(void){
    run currentRun;
    run displayRun;
    unsigned char menu = 0;
    unsigned char polePage = 0; // Which pole is displayed in log
    bool logs = true;
    unsigned char topMagTires = 0;
    unsigned char bottomMagTires = 0;
    
    // RD2 is the character LCD RS
    // RD3 is the character LCD enable (E)
    // RD4-RD7 are character LCD data lines
    LATA = 0x00;
    TRISA = 0x00;
    LATB = 0x00;
    TRISB = 0xff;
    LATD = 0x00;
    TRISD = 0x00;
    LATC = 0x00;
    TRISC = 0x00;
    LATE = 0x00;
    TRISE = 0x00;
    
    TRISCbits.RC2 = 1;  // Uno busy
    TRISAbits.RA5 = 1;  // Stop motors
    TRISCbits.RC0 = 1;  // Uno run finished
    TRISAbits.RA2 = 1;  // Pole break beam
    TRISEbits.RE0 = 1;  // Tire 1 break beam
    TRISEbits.RE1 = 1;  // Tire 2 break beam
    
    // Set all A/D ports to digital (pg. 222)
    ADCON1 = 0b00001111;
    
    // Initialize LCD
    initLCD();
    
    // Initialize I2C Master with 100 kHz clock
    I2C_Master_Init(100000);
    
    // Set the time in the RTC. To see the RTC keep time, comment this line out
    // after programming the PIC directly before with this line included
//    setTime();
    
    lcd_display_control(true, false, false);
    
//    while (1){
//        // Reset RTC memory pointer
//        I2C_Master_Start(); // Start condition
//        I2C_Master_Write(0b11010000); // 7 bit RTC address + Write
//        I2C_Master_Write(0x00); // Set memory pointer to seconds
//        I2C_Master_Stop(); // Stop condition
//
//        // Read current time
//        I2C_Master_Start(); // Start condition
//        I2C_Master_Write(0b11010001); // 7 bit RTC address + Read
//        for(unsigned char i = 0; i < 6; i++){
//            endTime[i] = I2C_Master_Read(ACK); // Read with ACK to continue reading
//        }
//        endTime[6] = I2C_Master_Read(NACK); // Final Read with NACK
//        I2C_Master_Stop(); // Stop condition
//
//        lcd_clear();
//        lcd_set_ddram_addr(LCD_LINE1_ADDR);
//        printf("%d", bcdToDec(endTime[2]));
//        lcd_set_ddram_addr(LCD_LINE2_ADDR);
//        printf("%d", bcdToDec(endTime[1]));
//        lcd_set_ddram_addr(LCD_LINE3_ADDR);
//        printf("%d", bcdToDec(endTime[0]));
//    }
    
    while(1){
        switch (menu){
            case 0:
                mainMenu(&menu);
                break;
            case 1:
                start(&menu, &currentRun, &topMagTires, &bottomMagTires);
                break;
            case 2:
                runFinished(&menu, &currentRun, &polePage, &displayRun, &logs);
                break;
            case 3:
                runInfoMenu(&menu, &displayRun, logs);
                break;
            case 4:
                poleInfo(&menu, &displayRun, &polePage);
                break;
            case 5:
                saveRun(&menu, &currentRun);
                break;
            case 6:
                selectLog(&menu, &displayRun);
                break;
            case 7:
                runFinishedMenu(&menu, &logs);
                break;
            case 8:
                options(&menu, &topMagTires, &bottomMagTires);
                break;
        }
    }
}