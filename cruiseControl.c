/* cruiseControl.c - using TIM3 Measuring Frequency and RPM

 * This program configures the TIM3 CH1 is set to do Input Capture
 * from PA6. The program waits for capture flag (CC1IF) to set then
 * measures the period and calculates the frequency of the input signal,
 * then the RPM of the motor.
 * 
 * Some fuzzy logic and bang-bang control is used to have a functioning
 * cruise control. This can be toggled on and off, and when it is on 
 * the input signal is ignored.
 *
 */
#include "stm32f4xx.h"

#define RS 0x20     /* PB5 mask for reg select */
#define RW 0x40     /* PB6 mask for read/write */
#define EN 0x80     /* PB7 mask for enable */

void delayMs(int n);
void LCD_command(unsigned char command);
void LCD_data(char data);
void LCD_init(void);
void PORTS_init(void);
void LCD_displayOut(char[], char[]);

int period;
float frequency;
float ccFrequency;

int main(void) {
		LCD_init();
    int last = 0;
    int current;
    int cc = 0;
		int debugMode = 0;
		int result;
		float PWM_out;
		char lineOne[20];
		char lineTwo[20];
	
    GPIOA->AFR[0] |= 0x00300000;    /* PA5 pin for TIM8 */
    GPIOA->MODER &= ~0x00000C00;
    GPIOA->MODER |=  0x00000800;
    GPIOA->MODER |=  0xC;           /* PA1 analog */
	
		/* setup ADC1 */
    RCC->APB2ENR |= 0x00000100;     /* enable ADC1 clock */
    ADC1->CR2 = 0;                  /* SW trigger */
    ADC1->SQR3 = 1;                 /* conversion sequence starts at ch 1 */
    ADC1->SQR1 = 0;                 /* conversion sequence length 1 */
    ADC1->CR2 |= 1;                 /* enable ADC1 */
	
    /* setup TIM8 */
    RCC->APB2ENR |= 2;              /* enable TIM8 clock */
    TIM8->PSC = 10 - 1;             /* divided by 10 */
    TIM8->ARR = 26667 - 1;          /* divided by 26667 */
    TIM8->CNT = 0;
    TIM8->CCMR1 = 0x0068;           /* PWM mode */
    TIM8->CCER = 4;                 /* enable PWM Ch1N */
    TIM8->CCR1 = 90;                /* pulse width */
    TIM8->BDTR |= 0x8000;           /* enable output */
    TIM8->CR1 = 1;                  /* enable timer */
 
 
    // configure PA6 as input of TIM3 CH1
    RCC->AHB1ENR |=  1;             /* enable GPIOA clock */
    GPIOA->MODER &= ~0x00003000;    /* clear pin mode */
    GPIOA->MODER |=  0x00002000;    /* set pin to alternate function */
    GPIOA->AFR[0] &= ~0x0F000000;   /* clear pin AF bits */
    GPIOA->AFR[0] |= 0x02000000;    /* set pin to AF2 for TIM3 CH1 */

    // configure TIM3 to do input capture with prescaler ...
    RCC->APB1ENR |= 2;              /* enable TIM3 clock */
    TIM3->PSC = 16000 - 1;          /* divided by 16000 */
    TIM3->CCMR1 = 0x41;             /* set CH1 to capture at rising edges only */
    TIM3->CCER = 0x1;              /* enable CH 1 capture both edges */
    TIM3->CR1 = 1;                  /* enable TIM3 */

		GPIOA->PUPDR = 0x00050000;		/* enable pull-up resistors */

    while (1) {
				
				ADC1->CR2 |= 0x40000000;        /* start a conversion */
        while(!(ADC1->SR & 2)) {}       /* wait for conv complete */
        result = ADC1->DR;              /* read conversion result */
				
				//if not in cruise control, just use input
				if(cc == 0) {
					PWM_out = result * (26667/4096);
				}
				//if we are not at our target speed, change the PWM output
				//and, if we are over 20% away from the target speed, accelerate/decelerate faster
				else if (ccFrequency > frequency) {
					PWM_out = PWM_out + 50;
					LCD_data('+');
					if(ccFrequency*0.8 > frequency) {
						PWM_out = PWM_out + 200;
						LCD_data('^');
					}
				}
				else if (ccFrequency < frequency) {
					PWM_out = PWM_out - 50;
					LCD_data('-');
					if(ccFrequency*1.2 < frequency) {
						PWM_out = PWM_out - 200;
						LCD_data('v');
					}
				}
					
				TIM8->CCR1 = PWM_out; //output
						
				
        while (!(TIM3->SR & 2)) {}  /* wait until input edge is captured */
        current = TIM3->CCR1;       /* read captured counter value */
				
        period = current - last;    /* calculate the period */
        frequency = 1000.0f / period;
        last = current;
				
					
				//normal display mode
				if(debugMode == 0) {
					sprintf(lineOne, "%.1f MPH/",frequency*60*0.013387);
					if(cc == 1)
						sprintf(lineTwo, "%d  %.1f/",cc,ccFrequency*60*0.013887);	
					else
						sprintf(lineTwo, "%d/",cc);	
				}
				//debug display mode
				else {
					sprintf(lineOne, "%.1f  %d/",frequency*60,cc);
					sprintf(lineTwo, "%.1f  %.1f/",frequency,ccFrequency);
				}
				
				LCD_displayOut(lineOne, lineTwo); //display to LCD
				
				//flags
				if((GPIOA->IDR & 0x100) && (cc == 0)) {
					cc = 1;
					ccFrequency = frequency;
				}
				if(!(GPIOA->IDR & 0x100)) {
					cc = 0;
				}
				
				if((GPIOA->IDR & 0x200)) {
					debugMode = 1;
				}
				else {
					debugMode = 0;
				}
    }
}

/* output to LCD */
void LCD_displayOut(char rowOne[], char rowTwo[]) {
		LCD_command(0x01);
		for (int i = 0; i < 20 && rowOne[i] != '/'; i++) {
			LCD_data(rowOne[i]);
		}
		LCD_command(0xC0);
		for (int i = 0; i < 20 && rowTwo[i] != '/'; i++) {
			LCD_data(rowTwo[i]);
		}
}

/* initialize port pins then initialize LCD controller */
void LCD_init(void) {
    PORTS_init();

    delayMs(30);            /* initialization sequence */
    LCD_command(0x30);
    delayMs(10);
    LCD_command(0x30);
    delayMs(1);
    LCD_command(0x30);

    LCD_command(0x38);      /* set 8-bit data, 2-line, 5x7 font */
    LCD_command(0x06);      /* move cursor right after each char */
    LCD_command(0x01);      /* clear screen, move cursor to home */
    LCD_command(0x0F);      /* turn on display, cursor blinking */
}

void PORTS_init(void) {
    RCC->AHB1ENR |=  0x07;          /* enable GPIOB/C clock */

    /* PB5 for LCD R/S */
    /* PB6 for LCD R/W */
    /* PB7 for LCD EN */
    GPIOB->MODER &= ~0x0000FC00;    /* clear pin mode */
    GPIOB->MODER |=  0x00005400;    /* set pin output mode */
    GPIOB->BSRR = 0x00C00000;       /* turn off EN and R/W */

    /* PC0-PC7 for LCD D0-D7, respectively. */
    GPIOC->MODER &= ~0x0000FFFF;    /* clear pin mode */
    GPIOC->MODER |=  0x00005555;    /* set pin output mode */
}

void LCD_command(unsigned char command) {
    GPIOB->BSRR = (RS | RW) << 16;  /* RS = 0, R/W = 0 */
    GPIOC->ODR = command;           /* put command on data bus */
    GPIOB->BSRR = EN;               /* pulse E high */
    delayMs(0);
    GPIOB->BSRR = EN << 16;         /* clear E */

    if (command < 4)
        delayMs(2);         /* command 1 and 2 needs up to 1.64ms */
    else
        delayMs(1);         /* all others 40 us */
}

void LCD_data(char data) {
    GPIOB->BSRR = RS;               /* RS = 1 */
    GPIOB->BSRR = RW << 16;         /* R/W = 0 */
    GPIOC->ODR = data;              /* put data on data bus */
    GPIOB->BSRR = EN;               /* pulse E high */
    delayMs(0);
    GPIOB->BSRR = EN << 16;         /* clear E */

    delayMs(1);
}

/* 16 MHz SYSCLK */
void delayMs(int n) {
    int i;

    /* Configure SysTick */
    SysTick->LOAD = 16000;  /* reload with number of clocks per millisecond */
    SysTick->VAL = 0;       /* clear current value register */
    SysTick->CTRL = 0x5;    /* Enable the timer */

    for(i = 0; i < n; i++) {
        while((SysTick->CTRL & 0x10000) == 0) /* wait until the COUNTFLAG is set */
            { }
    }
    SysTick->CTRL = 0;      /* Stop the timer (Enable = 0) */
}