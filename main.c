/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "log.h"

/* TRUE means that DMA-accessible buffers are placed in a non-cached RAM
   area and that no cache management is required.*/
#define DMA_BUFFERS_COHERENCE TRUE

#include "shell.h"
#include "chprintf.h"

/*===========================================================================*/
/* GPT related.                                                              */
/*===========================================================================*/

static GPTConfig gpt3_config = {
  .frequency = 1000000, // 1MHz
  .callback = NULL,
  .cr2 = TIM_CR2_MMS_1,
  .dier = 0,
};


/*===========================================================================*/
/* PWM related.                                                              */
/*===========================================================================*/
#define LINE_PWM PAL_LINE(GPIOA, GPIOE_PIN8)

static PWMConfig pwm_config = {
  42000000, // 42MHz
  420, // 50KHz
  NULL,
  {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL}
  },
  0,
  0,
  0
};

/*===========================================================================*/
/* ADC related.                                                              */
/*===========================================================================*/
#define LINE_ADC PAL_LINE(GPIOA, GPIOE_PIN0)
#define ADC_NUM_CHANNELS 1
#define ADC_BUF_DEPTH 1

static adcsample_t adc_samples[ADC_NUM_CHANNELS * ADC_BUF_DEPTH];

static ADCConversionGroup adc_groupConfig = {
  FALSE,
  ADC_NUM_CHANNELS,
  NULL,
  NULL,
  0,
  ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(8),
  0,
  ADC_SMPR2_SMP_AN0(ADC_SAMPLE_15),
  0,
  0,
  0,
  0,
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0),
};



/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/
#define SHELL_WA_SIZE THD_WORKING_AREA_SIZE(2048)

void cmd_version(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)chp;
  (void)argc;
  (void)argv;
  chprintf(chp, "Version: 1.0.0\r\n");
}

void cmd_adc(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)chp;
  (void)argc;
  (void)argv;
  chprintf(chp, "ADC: %.1fv\r\n", (adc_samples[0] / 4095.0f) * 3.3f);
}

static const ShellCommand shell_commands[] = {
  {"version", cmd_version},
  {"adc", cmd_adc},
  {NULL, NULL}
};

char shell_history[SHELL_MAX_HIST_BUFF];
char *shell_completions[SHELL_MAX_COMPLETIONS];

const SerialConfig sd2_config = {
    .speed = 115200,
    .cr1 = 0,
    .cr2 = 0,
    .cr3 = 0,
};

static const ShellConfig shell_cfg = {
    (BaseSequentialStream *)&SD2,
    shell_commands,
    shell_history,
    sizeof(shell_history),
    shell_completions,
};


/*
 * Green LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palClearPad(GPIOA, GPIOA_LED_GREEN);
    chThdSleepMilliseconds(10000);
    palSetPad(GPIOA, GPIOA_LED_GREEN);
    chThdSleepMilliseconds(10000);
  }
}


/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  shellInit();

  palSetLineMode(LINE_PWM, PAL_MODE_ALTERNATE(1));
  palClearLine(LINE_PWM);

  palSetLineMode(LINE_ADC, PAL_MODE_INPUT_ANALOG);

  pwmStart(&PWMD1, &pwm_config);
  pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 5000));

  /*
   * Activates the serial driver 2 for logging
   */
  sdStart(&SD2, &sd2_config);

  /*
   * Activates the GPT driver
   */
  gptStart(&GPTD3, &gpt3_config);

   /*
   * Activates the ADC driver
   */
  adcStart(&ADCD1, NULL); // 10.5 MHz

  /*
   * Start the ADC conversion
   */
  adcStartConversion(&ADCD1, &adc_groupConfig, adc_samples, ADC_BUF_DEPTH);

  /*
   * Start the GPT timer in continuous mode
   */
  gptStartContinuous(&GPTD3, 100); // update frequency = 1M / 100 = 10KHz

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (true) {
    LOG("main", "Hello, World!\r\n");
    thread_t *shell_thd = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
                                                "shell", NORMALPRIO + 1,
                                                shellThread, (void *)&shell_cfg);
    chThdWait(shell_thd); /* Wait for the shell thread to exit */
    chThdSleepMilliseconds(500);
  }
}
