/*
 * Lab10.c
 *
 * Created: 4/4/2017 2:21:05 PM
 * Author : Liam
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>


#define REALSLOW 0x10
#define SLOW 0x15
#define MEDIUM 0x30
#define FAST 0x80

#define KEY 5

// global variable
volatile uint8_t done = 0; // this counts how many sensors have fired
volatile uint8_t sensors; // this holds which sensors have 
volatile uint16_t times[5]; // this holds the times the sensors took to fire
volatile uint8_t lightdark; // whether sensors are light or dark
volatile uint8_t leds;
volatile uint16_t myTime;
volatile uint8_t whichOneCount;
volatile uint8_t compareVal;
volatile uint8_t lastLightDark;
volatile int lost = 0;
volatile uint8_t solvedSteps[100]; // assume there are no more than 100 turns. 0 left turn, 1 u turn, 2 forward, 3 right
volatile int stepCounter = 0;
volatile uint8_t newTurn = 0;
volatile uint8_t finishedturning = 1;
volatile uint8_t solved = 0;
volatile uint16_t checking = 0;
volatile uint8_t lostTurn = 0; // 1 for left, 2 for right
volatile uint8_t turn = 0;
volatile uint8_t confirmReading = 0;
uint8_t numSolvedSteps;
uint8_t stepNum;
uint8_t testSteps[100];
volatile uint8_t doneCounter = 0;
uint8_t correctedSteps[100];



#define F_CPU 20000000UL
#define TIMEOUT_THRESH 0x0700
#define LD_THRESH 0x0700

// function prototypes
void moveMotors(uint8_t portd6, uint8_t portd5, uint8_t portd3, uint8_t portb3, uint8_t pwmspeed);
void pwmOn(void);
void pwmOff(void);
void forward(uint8_t spd);
void backward(uint8_t spd);
void turnLeft(uint8_t spd);
void turnRight(uint8_t spd);
void brake(void);
void configPWM(void);

void setUpCaps(void);
void configPCI(void);
void setUpSensing(void);
void testThreshold(void);

void getLightDark(void);

void checkIfDone(void);

uint8_t * convertTurns(uint8_t * pre, int numTurns, int *postTurns);


// ISRs
ISR(TIMER1_COMPA_vect) {
	TCCR1B = 0;
	
	DDRC = 0xFF;
	PORTC = 0x00; // force the pin change interrupt
	return;
}
ISR(PCINT1_vect) {
	TCCR1B = 0; // stop the clock
	leds = (PINC & 0b00011111);
	PORTC = (PORTC & 0b11011111); // turn off the LEDs
	leds = leds ^ sensors; // get the sensor which has been triggered
	DDRC |= leds; // turn this sensor to output and pull it low
	PORTC &= (~leds);
	sensors = leds ^ sensors; // turn this off and put it back in sensors
	
	myTime = TCNT1;
	whichOneCount = 4; // this is the index of the timeval
	compareVal = 0b00010000;
	while(compareVal) {
		if(compareVal & leds) { // if the led had been turned off
			times[whichOneCount] = myTime;
			done = done + 1;
		}
		compareVal = compareVal >> 1; // shift through the different LEDs to figure
		// out which ones have been turned off
		whichOneCount--; // index into time array
	}
	if(done == 5) {
		//TIMSK1 = 0x00;
		//TCNT1 = 0x00;
	}
	else {
		TCCR1B = (0<<CS12) | (1<<CS11) | (0<<CS10); // restart the timer
		PORTC |= 0b00100000; // turn the LEDs back on
	}
	
	return;
}

ISR(BADISR_vect) {
	return;
}

int main(void)
{
	uint8_t hasBeenSolved = eeprom_read_byte((uint8_t*)0);
	configPWM();
	pwmOn();
	
	if(hasBeenSolved == KEY ) {
		stepCounter = eeprom_read_byte((uint8_t *)2);
		numSolvedSteps = eeprom_read_byte((uint8_t*)1);
		eeprom_read_block(solvedSteps, (uint8_t *) 4, numSolvedSteps);
		int stepNum = 0;
		eeprom_read_block(testSteps, (uint8_t *) 100, 100);
		while(stepNum < numSolvedSteps) {
			getLightDark();
			
			if(lightdark & 0b00010001) {
				// this indicates a possible turn
				uint8_t options = (lightdark & 0b00010001);
				forward(FAST);
				while(lightdark & 0b00010001) {
					getLightDark();
				}
				if((lightdark & 0b00000100) || ((options & 0b00010001) == 0b00010001)) { // the first condition implies that there is the option to go forward; the second condition implies there is the option to go left or right
					// indicates we need to choose a turn
					uint8_t direction = solvedSteps[stepNum++];
					if(solvedSteps[stepNum] == 0) {
						turnLeft(MEDIUM); // next turn until the middle sensor sees nothing
						uint8_t confirmReading = 0;
						uint8_t lastReading;
						lastReading = lightdark;
						for(uint8_t i = 0; i < 10; ++i) {
							getLightDark();
							confirmReading += (lightdark == lastReading);
							lastReading = lightdark;
						}
						while((lightdark & 0b00000100) && confirmReading < 6) {
							confirmReading = 0;
							lastReading = lightdark;
							getLightDark();
							for(uint8_t i = 0; i < 10; ++i) {
								getLightDark();
								confirmReading += (lightdark == lastReading);
								lastReading = lightdark;
							}
						}
						while(!(lightdark & 0b00000010)) {
							getLightDark();
						}
						// now turn until the middle sensor sees the path again
						while(!(lightdark & 0b00000100)) {
							getLightDark();
						}
						
				} 
				else if(solvedSteps[stepNum] == 2) {
					forward(SLOW);
					while((lightdark & 0b00010001)) {
						getLightDark();
					}
				}
				else if(solvedSteps[stepNum] == 3) {
					turnRight(MEDIUM); // next turn until the middle sensor sees nothing
					uint8_t confirmReading = 0;
					uint8_t lastReading;
					lastReading = lightdark;
					for(uint8_t i = 0; i < 10; ++i) {
						getLightDark();
						confirmReading += (lightdark == lastReading);
						lastReading = lightdark;
					}
					while((lightdark & 0b00000100) && confirmReading < 6) {
						confirmReading = 0;
						lastReading = lightdark;
						getLightDark();
						for(uint8_t i = 0; i < 10; ++i) {
							getLightDark();
							confirmReading += (lightdark == lastReading);
							lastReading = lightdark;
						}
					}
					while(!(lightdark & 0b00001000)) {
						getLightDark();
					}
					// now turn until the middle sensor sees the path again
					while(!(lightdark & 0b00000100)) {
						getLightDark();
					}
				}
			stepNum++;
			}
			else if(options == 0b00000001) {
				turnLeft(FAST);
				while(!(lightdark & 0b00000100)) {
					getLightDark();
				}
			}
			else if(options == 0b00010000) {
				turnRight(FAST);
				while(!(lightdark & 0b00000100)) {
					getLightDark();
				}
			}
		}
		else {
			if(lightdark & 0b00000100) {
				forward(MEDIUM);
			}
			else if(lightdark & 0b00001000) {
				turnRight(SLOW);
			}
			else if(lightdark & 0b00000010) {
				turnLeft(SLOW);
			}
		}
		
		}
		pwmOff();
		brake();
	}
	else {
		/* Replace with your application code */
		while (1) 
		{
			if(solved == 2) {
					eeprom_write_byte((uint8_t *)0,KEY); // this indicates it's been solved
						
					eeprom_write_block(solvedSteps, (uint8_t *)100, stepCounter);
					
					eeprom_write_byte((uint8_t *) 2, stepCounter);
					
					int actualNumSteps = 0;
					uint8_t * correctedSteps = convertTurns(solvedSteps, correctedSteps, &actualNumSteps);
					
					
					eeprom_write_byte((uint8_t *)1, actualNumSteps);
					eeprom_write_block(correctedSteps,  (uint8_t *)4, actualNumSteps);
					solved = 1;
					pwmOff();
					brake();
				
			
			}
			
		
			if(!solved) {
				
				getLightDark();
			
				if(lightdark & 0b00000001) {
					uint8_t options = (lightdark & 0b00010100);
					forward(FAST); // move past the turn
					
					while(lightdark & 0b00000001) {
						getLightDark();
					}
					
					turnLeft(FAST);
					
					
					int lightdarkcount = 0; // extra error checking
					while((lightdark & 0b00000100) && lightdarkcount < 200) { // next turn until the middle sensor sees nothing
						lightdarkcount++;
						getLightDark();
					} 
					lightdarkcount = 0;
					while(!(lightdark & 0b00000010) && lightdarkcount < 250) {
						lightdarkcount++;
						getLightDark();
					}
					// now turn until the middle sensor sees the path again
					while(!(lightdark == 0b00000100)) {
						getLightDark();
					}
					
					if(options) { // only record the turn if there was the 
						solvedSteps[stepCounter] = 0; // 0 for left turn
						++stepCounter;
					}
					
					// finishedturning = 0;
					else if(lightdark & 0b00010000) {
						forward(FAST); // first move past the turn
						while(lightdark & 0b00010000) {
							getLightDark();
							checkIfDone();
						}
						
						if(lightdark & 0b00001110) {
							forward(SLOW);
							solvedSteps[stepCounter++] = 2;
						}
						else {
							turnRight(FAST);
							while(!(lightdark & 0b00000100)) {
								getLightDark();
							}
						}
						getLightDark();
						
					}
				}
				else if(lightdark & 0b00000100) {
					forward(MEDIUM);
					
				}
				
				
				
				
				
			
				
			
				else if(lightdark == 0b00000010) {
					turnLeft(MEDIUM);
				}
				else if(lightdark == 0b00001000) {
					turnRight(MEDIUM);
				}
			
				else if(lightdark == 0x00) {
						if(lastLightDark == 0b00000100) {
							
							// u-turn
							if(solvedSteps[stepCounter-1] !=1) { // don't double u-turn
								solvedSteps[stepCounter++] = 1;
								forward(FAST);
								_delay_ms(500);
							}
						turnRight(MEDIUM);
							
							
						}
						else if(lastLightDark & 0b00011100) {
							turnRight(FAST);
							while(!(lightdark & 0b00000100)) {
								getLightDark();
							}
						}
						
						else if(lastLightDark & 0b00000011) {
							turnLeft(FAST);
							while(!(lightdark & 0b00000100)) {
								getLightDark();
							}
						}
					
					
					
						lost = 1;

				}
				
			
				if(lightdark != 0x00) {
					lost = 0;
				}
		
		
		
				if(!lost) {
					lastLightDark = lightdark;
				}
			
			
			
			
				
			}
			else {
				pwmOff();
				brake();
			}
			
	
			}
		}
	return 0;
}

void moveMotors(uint8_t portd6, uint8_t portd5, uint8_t portd3, uint8_t portb3, uint8_t pwmspeed) {
	if(portb3) { // turn on portb and port d vals
		OCR2A = 0;
	}
	else {
		OCR2A = pwmspeed;
	}
	if(portd6) {
		OCR0A = 0;
	}
	else {
		OCR0A = pwmspeed;
	}
	if(portd5) {
		OCR0B = 0;
	}
	else {
		OCR0B = pwmspeed;
	}
	if(portd3) {
		OCR2B = 0;
	}
	else {
		OCR2B = pwmspeed;
	}
}

void pwmOn(void) {
	uint8_t val = 0b11110011; // set Fast PWM, non inverting
	TCCR0A = val;
	TCCR2A = val;
}

void pwmOff(void) {
	TCCR0A = 0; // disconnect oc
	TCCR2A = 0;
}

void forward(uint8_t spd) {
	//pd6, pd5, pd3, pb3
	// OCR0B is pd5, OCR0A is pd6, OCR2A is pb3, OCR2B is pd3
	moveMotors(1, 0, 0, 1, spd);
}

void turnRight(uint8_t spd) {
	moveMotors(1,0,1,0, spd);
}

void turnLeft(uint8_t spd) {
	moveMotors(0,1,0,1, spd);
}

void backward(uint8_t spd) {
	moveMotors(0,1,1,0, spd);
}

void brake(void) {
	PORTD |= 0b01101000;
	PORTB |= 0b00001000;
}

void configPWM(void) {
	DDRB |= 0b00001000;
	DDRD |= 0b01101000;
	TCCR0B = 0b00000001;
	TCCR2B = 0b00000001;
	TIMSK0 = 0b00000110;
	TIMSK2 = 0b00000110;
}

void setUpCaps(void) {
	DDRC = 0b00011111;
	PORTC = 0b00011111;
	_delay_us(50); // wait for them to charge
	DDRC = 0b00100000; // turn portc to input 
}

void configTimer1(void) {
	TCCR1B = 0; // stop the clock
	TCCR1C = 0;
	TCCR1A = (1<<COM1A1) | (0<<COM1A0); // setup output compare on OC reg A
	TIMSK1 = 0; // clear TC int mask reg
	TCNT1 = 0;
	
	OCR1A = 0x0A00;
	
	
	TIFR1 = 0x00;
	TIMSK1 = 0b00000010; // turn on output compare a interrupt
	
	TCCR1B = (0<<CS12)|(1<<CS11)|(0<<CS10); // start the clock
	
}

void configPCI(void) {
	PCIFR &= ~(1<<PCIF1); // clear interrupt flag
	PCMSK1 = 0b00011111;
	PCICR = (1<<PCIE1); // enable pin change interrupts on port C
}

void setUpSensing(void) {
	done = 0;
	lightdark = 0;
	sensors = 0b00011111; // all sensors are on initially
	PCIFR &= ~(1<<PCIF1); // clear interrupt flag	sensors = 0b00011111; // all sensors are on
	setUpCaps();
	configPCI();
	configTimer1();
	DDRC |= 0b00100000;
	PORTC |= 0b00100000; // turn on the LEDs
	sei(); // enable interrupts
}





// this function checks to see whether each sensor was over light
// or dark
void testThreshold() {
	TCCR1B = 0x00; // turn off the clock
	lightdark = 0x00;
	uint8_t index = 0b00010000;
	for(uint8_t i = 4; index; --i) { 
		if(times[i] < LD_THRESH) {
		lightdark |= index;
		}
		index = index >> 1; // shift the index for the next sensor
	}
	lightdark = ((~lightdark)&0b00011111);
}

void getLightDark(void) {
	setUpSensing(); // set up sensing here lol
	while(1) {
		if(done == 5) {
			cli();
			break;
		}
	}
	testThreshold();
	checkIfDone();
}

void checkIfDone() {
	if((lightdark & 0b00001110) == 0b00001110 && (lightdark != 0b00011111)) {
		++doneCounter;
	}
	else {
		doneCounter = 0;
	}
	if(doneCounter >= 80) {
		solved = 2;
		pwmOff();
		brake();
	}
}

uint8_t * convertTurns(uint8_t * pre, int numTurns, int *postTurns) {
	uint8_t post[100];
	while(numTurns >= 3) {
		uint8_t one = pre[numTurns -1];
		uint8_t two = pre[numTurns -2];
		uint8_t three = pre[numTurns -3];
		
		
		if(two == 1) {
			// u- turn
			if(one == 0) { // left
				if(three == 0) {
					// LUL = straight
					three = 2;
				}
				else if (three == 2) {
					// LUS
					three = 3;
				}
			}
			else if(one == 2) { // straight
				if(three == 0) { //SUL = right
					three = 3;
				}
			
		}
		post[99-*postTurns] = three;
			(*postTurns)++;
			numTurns-= 3;
		}
		else { // if no u-turns, put all of them in the post array
			post[99 - *postTurns] = three;
			(*postTurns)++;
			numTurns--;
		}
	}
	while(numTurns > 0) {
		post[99-*postTurns] = pre[numTurns -1];
		(*postTurns)++;
		numTurns--;
	}
	return post;
}
