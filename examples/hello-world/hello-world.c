/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         A very simple Contiki application showing how Contiki programs look
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "board.h"
#include "prcm.h"
#include "hw_sysctl.h"
#include "timer.h"
#include "ioc.h"
#include "lpm.c"
#include "lpm.h"
#include "dev/leds.h"
#include "button-sensor.h"
#include "board-peripherals.h"
#include "dev/adc-sensor.h"
#include "lib/sensors.h"
#include <stdio.h> /* For printf() */
/*---------------------------------------------------------------------------*/
//#define BOARD_IOID_DIO27          IOID_27
#define CC26XX_DEMO_SENSOR_1     &button_left_sensor
#define CC26XX_DEMO_SENSOR_2     &button_right_sensor
static struct etimer et_adc;
//static struct etimer et_gpio

#define CC26XX_DEMO_LOOP_INTERVAL       (CLOCK_SECOND * 20)

PROCESS(gpio_process, "Hello world process");
PROCESS(pwm_process,"pwm process");
PROCESS(adc_process, "adc process");
//AUTOSTART_PROCESSES(&gpio_process,&pwm_process, &adc_process);
AUTOSTART_PROCESSES(&adc_process);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
uint8_t pwm_request_max_pm(void)
{
return LPM_MODE_DEEP_SLEEP;
}

void sleep_enter(void)
{
leds_on(LEDS_RED);
}

void sleep_leave(void)
{
leds_off(LEDS_RED);
}

int16_t pwminit(int32_t freq)
{
    LPM_MODULE(pwmdrive_module, pwm_request_max_pm, sleep_enter, sleep_leave, LPM_DOMAIN_PERIPH);
    uint32_t load=0;

    IOCPinTypeGpioOutput(IOID_21);
    leds_off(LEDS_RED);

    /* Enable GPT0 clocks under active, sleep, deep sleep */
    ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_TIMER0);
    ti_lib_prcm_peripheral_sleep_enable(PRCM_PERIPH_TIMER0);
    ti_lib_prcm_peripheral_deep_sleep_enable(PRCM_PERIPH_TIMER0);
    ti_lib_prcm_load_set();
    while(!ti_lib_prcm_load_get());

    /*
     * Register ourself with LPM. This will keep the PERIPH PD powered on
     * during deep sleep, allowing the pwm to keep working while the chip is
     * being power-cycled
     */
    //supostamente, mas pelo jeito não está implementado no driver
    //a solução foi desabilitar os Low Power Modes da implementação
    //arquivo /contiki/cpu/cc26xx-cc13xx/lpm.h, substituir a linha
   // #define LPM_MODE_MAX_SUPPORTED LPM_MODE_DEEP_SLEEP
    //#define LPM_MODE_MAX_SUPPORTED LPM_MODE_AWAKE
    lpm_register_module(&pwmdrive_module);



    /* Drive the I/O ID with GPT0 / Timer A */
    ti_lib_ioc_port_configure_set(IOID_21, IOC_PORT_MCU_PORT_EVENT0, IOC_STD_OUTPUT);

    /* GPT0 / Timer A: PWM, Interrupt Enable */
    //HWREG(GPT0_BASE + GPT_O_TAMR) = (TIMER_CFG_A_PWM & 0xFF) | GPT_TAMR_TAPWMIE;
    ti_lib_timer_configure(GPT0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);


    /* Stop the timer */
    ti_lib_timer_disable(GPT0_BASE, TIMER_A);
    ti_lib_timer_disable(GPT0_BASE, TIMER_B);

    if(freq > 0)
    {
        load = (GET_MCU_CLOCK / freq);

        ti_lib_timer_load_set(GPT0_BASE, TIMER_A, load);
        ti_lib_timer_match_set(GPT0_BASE, TIMER_A, load-1);

        /* Start */
        ti_lib_timer_enable(GPT0_BASE, TIMER_A);
    }
    return load;

}



void pwmdeinit()
{
    /*
       * Unregister the buzzer module from LPM. This will effectively release our
       * lock for the PERIPH PD allowing it to be powered down (unless some other
       * module keeps it on)
       */
      //lpm_unregister_module(&pwmdrive_module);

      /* Stop the timer */
      ti_lib_timer_disable(GPT0_BASE, TIMER_A);

      /*
       * Stop the module clock:
       *
       * Currently GPT0 is in use by clock_delay_usec (GPT0/TB) and by this
       * module here (GPT0/TA).
       *
       * clock_delay_usec
       * - is definitely not running when we enter here and
       * - handles the module clock internally
       *
       * Thus, we can safely change the state of module clocks here.
       */
      ti_lib_prcm_peripheral_run_disable(PRCM_PERIPH_TIMER0);
      //ti_lib_prcm_peripheral_sleep_disable(PRCM_PERIPH_TIMER0);
      //ti_lib_prcm_peripheral_deep_sleep_disable(PRCM_PERIPH_TIMER0);
      ti_lib_prcm_load_set();
      while(!ti_lib_prcm_load_get());

      /* Un-configure the pin */
      IOCPinTypeGpioInput(IOID_21);
      IOCIOInputSet(IOID_21, IOC_INPUT_DISABLE);
}
/*----adc-----------------------------------------------------------------------*/

PROCESS_THREAD(adc_process, ev, data)
{
    static struct sensors_sensor *sensor;
    sensor = sensors_find(ADC_SENSOR);
    SENSORS_ACTIVATE(*sensor);
    sensor->configure(ADC_SENSOR_SET_CHANNEL,ADC_COMPB_IN_AUXIO7);

    PROCESS_BEGIN();
    etimer_set(&et_adc, CLOCK_SECOND * 3);

    while(1) {

        PROCESS_WAIT_EVENT();

        if(ev == PROCESS_EVENT_TIMER){

                   int valor = sensor->value(ADC_SENSOR_VALUE);
                   printf("valor:%d....\n",valor);

            etimer_reset(&et_adc);
        }
        SENSORS_DEACTIVATE(*sensor);
    }

    PROCESS_END();
}

/*adc----------------------------------------------------------------*/


/*
PROCESS_THREAD(gpio_process, ev, data)
{
    static int8_t counter = 0;
    PROCESS_BEGIN();
    etimer_set(&et_gpio, CLOCK_SECOND * 1);


    //ti_lib_rom_ioc_pin_type_gpio_output(IOID_27);
    IOCPinTypeGpioOutput(IOID_27);
    //ti_lib_gpio_clear_multi_dio(1<<IOID_27);
    GPIO_clearMultiDio(1<<IOID_27);
    IOCPinTypeGpioOutput(IOID_28);
    //ti_lib_gpio_clear_multi_dio(1<<IOID_27);
    GPIO_clearMultiDio(1<<IOID_28);


    while(1) {

        PROCESS_WAIT_EVENT();

        if(ev == PROCESS_EVENT_TIMER){
            counter=(counter+1)%2;

            if(counter)
            {
                //ti_lib_gpio_set_dio(IOID_27);
                GPIO_setDio(IOID_27);
                GPIO_clearDio(IOID_28);
            }
            else
            {
                //ti_lib_gpio_clear_dio(IOID_27);
                GPIO_clearDio(IOID_27);
                GPIO_setDio(IOID_28);

            }
            etimer_reset(&et_gpio);
        }
    }

    PROCESS_END();
}
*/
/*

PROCESS_THREAD(pwm_process, ev, data)
{
    static int16_t current_duty = 0;
    static int16_t loadvalue;
    static int16_t ticks=0;
    //static int16_t contador=0;

    PROCESS_BEGIN();sensor
    loadvalue = pwminit(5000);
    SENSORS_ACTIVATE(button_left_sensor);
    SENSORS_ACTIVATE(button_right_sensor);


    while(1) {

        PROCESS_YIELD();
        if(ev == sensors_event)
        {
            if(data == CC26XX_DEMO_SENSOR_1)
            {
               // contador = contador + 400;
                current_duty=current_duty+10;

            }
            else if(data == CC26XX_DEMO_SENSOR_2)
            {
                //contador = contador - 400;
                current_duty=current_duty-10;

             }
            //if(contador>4000){contador=4000;}
            //if(contador<0){contador=0;}
            if(current_duty>=100){current_duty=100;}
            if(current_duty<=0){current_duty=0;}

            //printf("\n%d\n",current_duty);

            ticks = (current_duty * loadvalue)/100;

            ti_lib_timer_load_set(GPT0_BASE, TIMER_A,loadvalue);
            //ti_lib_timer_match_set(GPT0_BASE, TIMER_A,loadvalue- contador);
            ti_lib_timer_match_set(GPT0_BASE, TIMER_A,loadvalue- ticks);

         }
    }

    PROCESS_END();
}




*/


/*---------------------------------------------------------------------------*/
