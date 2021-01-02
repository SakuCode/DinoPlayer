/*
 * DinoPlayer
 * Chrome Dino game
 * Servo with LDR sensor using ATmega4809
 * Author: Saku Linnankoski sjlinn@utu.fi
 * v1.4
*/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

/* Standard PWM period 20ms*/
#define SERVO_PWM_PERIOD (0x1046)
/* Neutral 0 degree angle*/
#define SERVO_PWM_DUTY_NEUTRAL (0x0138)
/* Adjustable angle for servo down to space key*/
#define SERVO_DOWN (0x0104)
/* Adjustable threshold for LDR detection*/
#define LIGHT_THRESHOLD (0x258)

/* LDR value*/
uint16_t lightLevel;

void ADC0init(void)
{
    /* Disable digital input buffer*/
    PORTD.DIRCLR = PIN0_bm;
    PORTD.PIN0CTRL |= PORT_ISC_INPUT_DISABLE_gc;
 
    /* Vdd reference voltage and prescaler of 16*/
    ADC0.CTRLC |= ADC_PRESC_DIV16_gc | ADC_REFSEL_VDDREF_gc;
    
    /* Enable (power up) ADC (10-bit resolution is default)*/
    ADC0.CTRLA |= ADC_ENABLE_bm;
}

uint16_t ADC0_read(void)
{
    /* Clear the interrupt flag by writing 1*/
    ADC0.INTFLAGS = ADC_RESRDY_bm;
    
    return ADC0.RES;
}

void ADC0_start(void)
{
    /* Start conversion*/
    ADC0.COMMAND = ADC_STCONV_bm;
}

void SERVOinit(void)
{
    /* Route TCA0 PWM waveform to PORTB*/
    PORTMUX.TCAROUTEA |= PORTMUX_TCA0_PORTB_gc;
 
    /* Set 0-WO2 (PB2) as digital output*/
    PORTB.DIRSET = PIN2_bm;
 
    /* Set TCA0 prescaler value to 16 (~208 kHz)*/
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc;
 
    /* Set single-slop PWM generation mode*/
    TCA0.SINGLE.CTRLB |= TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
 
    /* Using double-buffering, set PWM period (20 ms)*/
    TCA0.SINGLE.PERBUF = SERVO_PWM_PERIOD;
 
    /* Set initial servo arm position as neutral (0 deg)*/
    TCA0.SINGLE.CMP2BUF = SERVO_PWM_DUTY_NEUTRAL;
 
    /* Enable Compare Channel 2*/
    TCA0.SINGLE.CTRLB |= TCA_SINGLE_CMP2EN_bm;
 
    /* Enable TCA0 peripheral*/
    TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
}

int main(void)
{
    /* Clear the global interrupt flag*/
    cli();
    
    /* Initialize LDR ADC, SERVO and LED*/
    ADC0init();
    SERVOinit();
    PORTF.DIRSET = PIN5_bm;
    
    /* Enable interrupts*/
    sei();
    
    while (1)
    {
        /* Start LDR ADC*/
        ADC0_start();
        
        /* Read the LDR ADC value*/
        lightLevel = ADC0_read();
        
        while (!(ADC0.INTFLAGS & ADC_RESRDY_bm))
        {
            /* Blink LED if LDR threshold detected*/
            PORTF.OUTTGL = PIN5_bm;
            
            if(lightLevel < LIGHT_THRESHOLD)
            {
                /* SERVO in up position when LDR under threshold*/
                TCA0.SINGLE.CMP2BUF = SERVO_PWM_DUTY_NEUTRAL;
            }
            
            else
            {
                /* If LDR value over threshold put servo in down position*/
                TCA0.SINGLE.CMP2BUF = SERVO_DOWN;
                _delay_ms(500);
            }
        }
        
        /* Must clear ADC interrupt flag for next ADC read*/
        ADC0.INTFLAGS = ADC_RESRDY_bm;
    }
}