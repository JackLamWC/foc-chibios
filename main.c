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
#include <stdlib.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* TRUE means that DMA-accessible buffers are placed in a non-cached RAM
   area and that no cache management is required.*/
#define DMA_BUFFERS_COHERENCE TRUE

#include "shell.h"
#include "chprintf.h"

#define LINE_DRV_ENABLE             PAL_LINE(GPIOC, GPIOC_PIN8)
#define LINE_DRV_N_FAULT            PAL_LINE(GPIOC, GPIOC_PIN10) 
#define LINE_DRV_N_OCTW             PAL_LINE(GPIOC, GPIOC_PIN11)
#define LINE_DRV_M_OC               PAL_LINE(GPIOC, GPIOC_PIN12)
#define LINE_DRV_OC_ADJ             PAL_LINE(GPIOD, GPIOD_PIN2)
#define LINE_DRV_M_PWM              PAL_LINE(GPIOC, GPIOD_PIN9)

/*===========================================================================*/
/* GPT related.                                                              */
/*===========================================================================*/
static uint32_t last_isr_cycles = 0;
static uint32_t debug_isr_duration_cycles = 0;


static GPTConfig gpt3_config = {
  .frequency = 1000000, // 1MHz
  .callback = NULL,
  .cr2 = TIM_CR2_MMS_1,
  .dier = 0,
};


/*===========================================================================*/
/* PWM related.                                                              */
/*===========================================================================*/

/* the mode of pwm which is either 6 pwm or 3 pwm. to select the mode, set or reset the M_PWM, HIGH = 3 pwm, LOW = 6 pwm */
/* normal mode: 3 pwm */
#define LINE_MTR_PWM_PHASE_AH PAL_LINE(GPIOA, GPIOA_ARD_D7)
#define LINE_MTR_PWM_PHASE_AL PAL_LINE(GPIOA, GPIOA_ARD_D11)
#define LINE_MTR_PWM_PHASE_BH PAL_LINE(GPIOA, GPIOA_ARD_D8)
#define LINE_MTR_PWM_PHASE_BL PAL_LINE(GPIOB, GPIOB_ARD_A3)
#define LINE_MTR_PWM_PHASE_CH PAL_LINE(GPIOA, GPIOA_ARD_D2)
#define LINE_MTR_PWM_PHASE_CL PAL_LINE(GPIOB, GPIOB_PIN1)


#define MTR_PWM_CHANNELS 3
#define MTR_PHASE_A 0
#define MTR_PHASE_B 1
#define MTR_PHASE_C 2

#define MTR_DEFAULT_DUTY 20





static PWMConfig mtr_pwm_config = {
  42000000, // 42MHz
  420, // 50KHz
  NULL,
  {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_DISABLED, NULL}
  },
  0,
  0,
  0
};

static void mtr_pwm_init(void) {
  /*
   * Set the PWM lines to alternate function mode
   */
  palSetLineMode(LINE_MTR_PWM_PHASE_AH, PAL_MODE_ALTERNATE(1));
  palSetLineMode(LINE_MTR_PWM_PHASE_BH, PAL_MODE_ALTERNATE(1));
  palSetLineMode(LINE_MTR_PWM_PHASE_CH, PAL_MODE_ALTERNATE(1));

  palSetLineMode(LINE_MTR_PWM_PHASE_AL, PAL_MODE_ALTERNATE(1));
  palSetLineMode(LINE_MTR_PWM_PHASE_BL, PAL_MODE_ALTERNATE(1));
  palSetLineMode(LINE_MTR_PWM_PHASE_CL, PAL_MODE_ALTERNATE(1));


  palClearLine(LINE_MTR_PWM_PHASE_AH);
  palClearLine(LINE_MTR_PWM_PHASE_BH);
  palClearLine(LINE_MTR_PWM_PHASE_CH);

  palClearLine(LINE_MTR_PWM_PHASE_AL);
  palClearLine(LINE_MTR_PWM_PHASE_BL);
  palClearLine(LINE_MTR_PWM_PHASE_CL);

  /*
   * Start the PWM driver
   */
  pwmStart(&PWMD1, &mtr_pwm_config);
  pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
  pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0)); 
  pwmEnableChannel(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
}

inline static void mtr_pwm_write_duty(uint8_t channel, uint16_t duty) {
  pwmEnableChannel(&PWMD1, channel, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, duty));
}

/*
 * Commutation state machine
 */

static uint8_t mtr_commutation_state = 0;

static const uint8_t mtr_commutation_valid_states_mask = 0b01111110; // 0b001, 0b010, 0b011, 0b100, 0b101, 0b110

static bool mtr_commutation_is_vaild_state(uint8_t state) {
    return (mtr_commutation_valid_states_mask & (1 << (state & 0x07))) != 0;
}

static uint16_t mtr_duty = MTR_DEFAULT_DUTY * 10;

typedef enum {
  MTR_COMMUTATION_PASHE_IN, // up side close
  MTR_COMMUTATION_PASHE_OUT, // low side close
  MTR_COMMUTATION_PASHE_FLOATING, // up and low side open
} mtr_commutation_phase;

static mtr_commutation_phase mtr_commutation_phase_table[8][3] = {
  {MTR_COMMUTATION_PASHE_FLOATING, MTR_COMMUTATION_PASHE_FLOATING, MTR_COMMUTATION_PASHE_FLOATING}, // 000 - invalid state
  {MTR_COMMUTATION_PASHE_OUT     , MTR_COMMUTATION_PASHE_FLOATING, MTR_COMMUTATION_PASHE_IN}, // 001
  {MTR_COMMUTATION_PASHE_FLOATING, MTR_COMMUTATION_PASHE_IN     , MTR_COMMUTATION_PASHE_OUT}, // 010
  {MTR_COMMUTATION_PASHE_OUT     , MTR_COMMUTATION_PASHE_IN     , MTR_COMMUTATION_PASHE_FLOATING}, // 011
  {MTR_COMMUTATION_PASHE_IN      , MTR_COMMUTATION_PASHE_OUT    , MTR_COMMUTATION_PASHE_FLOATING}, // 100
  {MTR_COMMUTATION_PASHE_FLOATING, MTR_COMMUTATION_PASHE_OUT    , MTR_COMMUTATION_PASHE_IN}, // 101
  {MTR_COMMUTATION_PASHE_IN      , MTR_COMMUTATION_PASHE_FLOATING, MTR_COMMUTATION_PASHE_OUT}, // 110
  {MTR_COMMUTATION_PASHE_FLOATING, MTR_COMMUTATION_PASHE_FLOATING, MTR_COMMUTATION_PASHE_FLOATING}, // 111 - invalid state
};


static uint8_t last_state = 0;
inline static void mtr_commutation_set_state(uint8_t state) {
  mtr_commutation_state = state;
  if(state == last_state) {
    return;
  }
  last_state = state;
    
  uint16_t phases_duty[MTR_PWM_CHANNELS] = {0, 0, 0};
  pwmStop(&PWMD1);
  for(int i = 0; i < MTR_PWM_CHANNELS; i++) {
    switch(mtr_commutation_phase_table[state][i]) {
      case MTR_COMMUTATION_PASHE_IN: {
        phases_duty[i] = 5000 + mtr_duty / 2;
        mtr_pwm_config.channels[i].mode = PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH;
        break;
      }
      case MTR_COMMUTATION_PASHE_OUT: {
        mtr_pwm_config.channels[i].mode = PWM_OUTPUT_ACTIVE_LOW | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW;
        phases_duty[i] = 10000 - (5000 - mtr_duty / 2);
        break;
      }
      case MTR_COMMUTATION_PASHE_FLOATING: {
        phases_duty[i] = 0;
        mtr_pwm_config.channels[i].mode = PWM_OUTPUT_DISABLED;
        break;
      }
    }
  }
  pwmStart(&PWMD1, &mtr_pwm_config);
  for(int i = 0; i < MTR_PWM_CHANNELS; i++) {
    mtr_pwm_write_duty(i, phases_duty[i]);
  }
}

void mtr_commutation_transit_to_state(void) {
  switch(mtr_commutation_state) {
    case 0b100:
      mtr_commutation_set_state(0b110);
      break;
    case 0b110:
      mtr_commutation_set_state(0b010);
      break;
    case 0b010:
      mtr_commutation_set_state(0b011);
      break;
    case 0b011:
      mtr_commutation_set_state(0b001);
      break;
    case 0b001:
      mtr_commutation_set_state(0b101);
      break;
    case 0b101:
      mtr_commutation_set_state(0b100);
      break;
    default:
      mtr_commutation_set_state(0b000);
      break;
  }
}

/*
 * Motor control state machine
 */

typedef enum {
  MTR_CONTROL_STATE_IDLE,
  MTR_CONTROL_STATE_RAMP_UP,
  MTR_CONTROL_STATE_CONST_SPEED,
  MTR_CONTROL_STATE_BACK_EMF_COMMUTATION,
} mtr_control_state_t;

static mtr_control_state_t mtr_control_state = MTR_CONTROL_STATE_IDLE;

/*
 * Linear ramp profile for PWM duty cycle
 */
typedef struct {
  uint16_t start_duty;      // Starting duty cycle (0-10000 for 0-100%)
  uint16_t target_duty;     // Target duty cycle (0-10000 for 0-100%)
  uint32_t ramp_time_ms;    // Total ramp time in milliseconds
  uint32_t start_time;      // Start time in system ticks
  bool active;              // Whether ramp is currently active
} mtr_ramp_profile_t;

static mtr_ramp_profile_t mtr_ramp_profile = {
  .start_duty = 0,
  .target_duty = 0,
  .ramp_time_ms = 1000,
  .start_time = 0,
  .active = false
};

/**
 * @brief Start a linear ramp profile
 * @param start_duty Starting duty cycle (0-10000 for 0-100%)
 * @param target_duty Target duty cycle (0-10000 for 0-100%)
 * @param ramp_time_ms Ramp time in milliseconds
 */
void mtr_ramp_start(uint16_t start_duty, uint16_t target_duty, uint32_t ramp_time_ms) {
  if(mtr_control_state != MTR_CONTROL_STATE_IDLE) {
    return;
  }
  mtr_ramp_profile.start_duty = start_duty;
  mtr_ramp_profile.target_duty = target_duty;
  mtr_ramp_profile.ramp_time_ms = ramp_time_ms;
  mtr_ramp_profile.start_time = chVTGetSystemTime();
  mtr_control_state = MTR_CONTROL_STATE_RAMP_UP;
}

/**
 * @brief Get current duty cycle value from linear ramp profile
 * @return Current duty cycle (0-10000 for 0-100%)
 */
uint16_t mtr_ramp_get_duty(void) {
  if(mtr_control_state != MTR_CONTROL_STATE_RAMP_UP) {
    return 0;
  }

  uint32_t current_time = chVTGetSystemTime();
  uint32_t elapsed_time = current_time - mtr_ramp_profile.start_time;
  uint32_t ramp_time_ticks = TIME_MS2I(mtr_ramp_profile.ramp_time_ms);
  
  // Check if ramp is complete
  if (elapsed_time >= ramp_time_ticks) {
    mtr_control_state = MTR_CONTROL_STATE_CONST_SPEED;
    return mtr_ramp_profile.target_duty;
  }
  
  // Calculate linear interpolation
  float progress = (float)elapsed_time / (float)ramp_time_ticks;
  uint16_t current_duty = mtr_ramp_profile.start_duty + 
                         (uint16_t)((mtr_ramp_profile.target_duty - mtr_ramp_profile.start_duty) * progress);
  
  return current_duty;
}

void mtr_control_start(uint16_t duty, uint32_t ramp_time_ms) {
  mtr_commutation_set_state(0b100);
  mtr_ramp_start(0, duty, ramp_time_ms);
}

void mtr_control_stop(void) {
  mtr_control_state = MTR_CONTROL_STATE_IDLE;
  mtr_duty = 0;
  mtr_commutation_set_state(0b000);
}

/*===========================================================================*/
/* ADC related.                                                              */
/*===========================================================================*/
#define LINE_MTR_VOLTAGE_PHASE_A PAL_LINE(GPIOA, GPIOA_ARD_A0)
#define LINE_MTR_VOLTAGE_PHASE_B PAL_LINE(GPIOA, GPIOA_ARD_A1)
#define LINE_MTR_VOLTAGE_PHASE_C PAL_LINE(GPIOA, LINE_ARD_A2)
#define LINE_MTR_VOLTAGE_VPDD PAL_LINE(GPIOC, LINE_ARD_A4)

#define MTR_ADC_NUM_CHANNELS 4
#define MTR_ADC_BUF_DEPTH 1

#define MTR_ADC_BACKEMF_THRESHOLD 0.5f
#define MTR_ADC_BACKEMF_PHASE_A 0
#define MTR_ADC_BACKEMF_PHASE_B 1
#define MTR_ADC_BACKEMF_PHASE_C 2
#define MTR_ADC_BACKEMF_VPDD 3

static adcsample_t mtr_voltage_adc_samples[MTR_ADC_NUM_CHANNELS * MTR_ADC_BUF_DEPTH];

float mtr_voltage_adc_get_voltage(uint8_t channel) {
  return (mtr_voltage_adc_samples[channel] / 4095.0f) * 3.3f;
}

// EMA filtered ADC samples
static float mtr_voltage_adc_filtered[MTR_ADC_NUM_CHANNELS] = {0};
// Alpha for EMA filter, can be set at runtime
static float ema_alpha = 0.611f; // Default for 600Hz cutoff at 4kHz sample rate

// Function to get filtered ADC value (in volts)
float mtr_voltage_adc_get_filtered(uint8_t channel) {
  if (channel >= MTR_ADC_NUM_CHANNELS) {
    return 0.0f;
  }
  return mtr_voltage_adc_filtered[channel];
}

/**
 * @brief Calculate EMA alpha for a given cutoff frequency and sample rate
 * @param cutoff_freq Cutoff frequency in Hz
 * @param sample_rate Sample rate in Hz
 * @return Alpha value for EMA filter
 *
 * Usage: float alpha = calculate_ema_alpha(600.0f, 4000.0f);
 */
static float calculate_ema_alpha(float cutoff_freq, float sample_rate) {
    return 1.0f - expf(-2.0f * (float)M_PI * cutoff_freq / sample_rate);
}

void mtr_voltage_adc_init(void) {
  palSetLineMode(LINE_MTR_VOLTAGE_PHASE_A, PAL_MODE_INPUT_ANALOG);
  palSetLineMode(LINE_MTR_VOLTAGE_PHASE_B, PAL_MODE_INPUT_ANALOG);
  palSetLineMode(LINE_MTR_VOLTAGE_PHASE_C, PAL_MODE_INPUT_ANALOG);
  palSetLineMode(LINE_MTR_VOLTAGE_VPDD, PAL_MODE_INPUT_ANALOG);
}

// Add this lookup table before the callback function
static uint8_t mtr_backemf_phase_lookup[8] = {
  0xFF, // 0b000 - invalid state
  0xFF, // 0b001 - Phase B
  0xFF, // 0b010 - Phase A  
  0xFF, // 0b011 - Phase C
  0xFF, // 0b100 - Phase C
  0xFF, // 0b101 - Phase A
  0xFF, // 0b110 - Phase B
  0xFF  // 0b111 - invalid state
};

static void mtr_backemf_init(void) {
  mtr_backemf_phase_lookup[0b100] = MTR_ADC_BACKEMF_PHASE_C;
  mtr_backemf_phase_lookup[0b110] = MTR_ADC_BACKEMF_PHASE_B;
  mtr_backemf_phase_lookup[0b010] = MTR_ADC_BACKEMF_PHASE_A;
  mtr_backemf_phase_lookup[0b011] = MTR_ADC_BACKEMF_PHASE_C;
  mtr_backemf_phase_lookup[0b001] = MTR_ADC_BACKEMF_PHASE_B;
  mtr_backemf_phase_lookup[0b101] = MTR_ADC_BACKEMF_PHASE_A;
}

uint16_t mtr_backemf_voltage_samples[MTR_ADC_NUM_CHANNELS * MTR_ADC_BUF_DEPTH];
void mtr_commutation_adc_backemf_cb(ADCDriver *adcp) {
  (void)adcp;
#ifndef JACK_DEBUG
  for(int i = 0; i < MTR_ADC_NUM_CHANNELS; i++) {
    mtr_backemf_voltage_samples[i] = mtr_voltage_adc_samples[i];
    // Update EMA filtered value
    float new_sample = (mtr_voltage_adc_samples[i] / 4095.0f) * 3.3f;
    mtr_voltage_adc_filtered[i] = ema_alpha * new_sample + (1.0f - ema_alpha) * mtr_voltage_adc_filtered[i];
  }

  if(!mtr_commutation_is_vaild_state(mtr_commutation_state)) {
    return;
  }

  uint32_t end_cycles = DWT->CYCCNT;
  uint32_t duration_cycles = end_cycles - last_isr_cycles;
  last_isr_cycles = end_cycles;
  debug_isr_duration_cycles = duration_cycles;

  if(mtr_control_state == MTR_CONTROL_STATE_BACK_EMF_COMMUTATION) {
    uint8_t backemf_phase = mtr_backemf_phase_lookup[mtr_commutation_state];
    if(backemf_phase != 0xFF) {
      float backemf_voltage = mtr_voltage_adc_get_filtered(backemf_phase);
      float vpdd_voltage = mtr_voltage_adc_get_filtered(MTR_ADC_BACKEMF_VPDD);
      if(backemf_voltage >= vpdd_voltage * MTR_ADC_BACKEMF_THRESHOLD) {
        mtr_commutation_transit_to_state();
      }
    }
  }
#else
  static systime_t last_time = 0;
  systime_t current_time = chVTGetSystemTime();
  if(current_time - last_time >TIME_MS2I(1000)) {
    LOG("Hello\r\n");
    last_time = current_time;
    mtr_commutation_transit_to_state();
  }
#endif
}

static ADCConversionGroup mtr_voltage_adc_groupConfig = {
  true,
  MTR_ADC_NUM_CHANNELS,
  mtr_commutation_adc_backemf_cb,
  NULL,
  0,
  ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(8),
  ADC_SMPR1_SMP_AN11(ADC_SAMPLE_3),
  ADC_SMPR2_SMP_AN0(ADC_SAMPLE_3) | ADC_SMPR2_SMP_AN1(ADC_SAMPLE_3) | ADC_SMPR2_SMP_AN4(ADC_SAMPLE_3),
  0,
  0,
  0,
  0,
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN1) | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN4) | ADC_SQR3_SQ4_N(ADC_CHANNEL_IN11),
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
  for(int i = 0; i < MTR_ADC_NUM_CHANNELS; i++) {
    chprintf(chp, "ADC%d: %.1fv\r\n", i, (mtr_voltage_adc_samples[i] / 4095.0f) * 3.3f);
  }
}

void cmd_mtr_run(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)chp;
  (void)argc;
  (void)argv;
  // the first argument is the duty
  if(argc == 1) {
    mtr_duty = atoi(argv[0]) * 100;
    LOG("mtr_duty: %d\r\n", mtr_duty);
    if(mtr_duty != 0) {
      mtr_control_start(mtr_duty, 5000);
    }
    else {
      mtr_control_stop();
    }
  }
  else {
    chprintf(chp, "Usage: mtr-run <duty>\r\n");
  }
  
}

void cmd_motor_enable(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)chp;
  (void)argc;
  (void)argv;
  if(argc == 1) {
    if(argv[0][0] == '1') {
      palSetLine(LINE_DRV_ENABLE);
    } else {
      palClearLine(LINE_DRV_ENABLE);
    }
  }
  else {
    chprintf(chp, "Usage: motor-enable <1|0>\r\n");
  }
}

void cmd_motor_fault(BaseSequentialStream *chp, int argc, char *argv[]) {

  (void)chp;
  (void)argc;
  (void)argv;
  chprintf(chp, "Motor fault: %d\r\n", palReadLine(LINE_DRV_N_FAULT));
  chprintf(chp, "Motor octw: %d\r\n", palReadLine(LINE_DRV_N_OCTW));
}

void cmd_mtr_debug(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)chp;
  (void)argc;
  (void)argv;
  chprintf(chp, "ISR duration: %d\r\n", debug_isr_duration_cycles);
  chprintf(chp, "Control state: %d\r\n", mtr_control_state);
  for(int i = 0; i < MTR_ADC_NUM_CHANNELS; i++) {
    chprintf(chp, "ADC%d: %.1fv\r\n", i, (mtr_backemf_voltage_samples[i] / 4095.0f) * 3.3f);
    chprintf(chp, "ADC%d filtered: %.1fv\r\n", i, mtr_voltage_adc_get_filtered(i));
  }
  chprintf(chp, "Commutation state: %d\r\n", mtr_commutation_state);
}

void cmd_mtr_set_state(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void)chp;
  (void)argc;
  (void)argv;
  if(argc == 2) {
    uint8_t state = atoi(argv[0]);
    uint16_t duty = atoi(argv[1]);
    mtr_duty = duty * 100;
    chprintf(chp, "Setting state to %d, duty to %d\r\n", state, duty);
    mtr_commutation_set_state(state);
  }
  else {
    chprintf(chp, "Usage: mtr-set-state <state> <duty>\r\n");
  }
}

static const ShellCommand shell_commands[] = {
  {"version", cmd_version},
  {"adc", cmd_adc},
  {"mtr-run", cmd_mtr_run},
  {"mtr-enable", cmd_motor_enable},
  {"mtr-fault", cmd_motor_fault},
  {"mtr-debug", cmd_mtr_debug},
  {"mtr-set-state", cmd_mtr_set_state},
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

static THD_WORKING_AREA(waMotorControlThread, 1024);
static THD_FUNCTION(MotorControlThread, arg) {
  (void)arg;
  chRegSetThreadName("motor-control");
  while (true) {
    switch(mtr_control_state) {
      case MTR_CONTROL_STATE_RAMP_UP:
        mtr_duty = mtr_ramp_get_duty();
        mtr_commutation_transit_to_state();
        break;
      case MTR_CONTROL_STATE_CONST_SPEED:
        mtr_control_state = MTR_CONTROL_STATE_BACK_EMF_COMMUTATION;
        mtr_commutation_transit_to_state();
        break;
      case MTR_CONTROL_STATE_BACK_EMF_COMMUTATION:
        break;
      case MTR_CONTROL_STATE_IDLE:
        break;
    }
    chThdSleepMilliseconds(10);
  }
}


/*
 * Green LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palClearPad(GPIOA, GPIOA_LED_GREEN);
    chThdSleepMilliseconds(100);
    palSetPad(GPIOA, GPIOA_LED_GREEN);
    chThdSleepMilliseconds(100);
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

  /*
   * Activates the serial driver 2 for logging
   */
  sdStart(&SD2, &sd2_config);

  /*
   * Activates the GPT driver
   */
  gptStart(&GPTD3, &gpt3_config);

  /*
   * Initialize the PWM driver
   */
  mtr_pwm_init();

  /*
   * Initialize back-EMF detection
   */
  mtr_backemf_init();

  mtr_voltage_adc_init();

  // Set EMA filter alpha for desired cutoff frequency and sample rate
  // Example: 600 Hz cutoff, 4000 Hz sample rate
  ema_alpha = calculate_ema_alpha(600.0f, 4000.0f);

   /*
   * Activates the ADC driver
   */
  adcStart(&ADCD1, NULL); // 10.5 MHz

  /*
   * Start the ADC conversion
   */
  adcStartConversion(&ADCD1, &mtr_voltage_adc_groupConfig, mtr_voltage_adc_samples, MTR_ADC_BUF_DEPTH);

  /*
   * Start the GPT timer in continuous mode
   */
  gptStartContinuous(&GPTD3, 250); // update frequency = 1M / 250 = 4000Hz

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Start the motor control thread
   */
  chThdCreateStatic(waMotorControlThread, sizeof(waMotorControlThread), NORMALPRIO, MotorControlThread, NULL);

  for(int i = 0; i < 4; i++) {
    mtr_pwm_write_duty(i, 0);
  }


  palSetLineMode(LINE_DRV_N_FAULT, PAL_MODE_INPUT_PULLDOWN);
  palSetLineMode(LINE_DRV_N_OCTW, PAL_MODE_INPUT_PULLDOWN);
  palSetLineMode(LINE_DRV_ENABLE, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LINE_DRV_M_OC, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LINE_DRV_OC_ADJ, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LINE_DRV_M_PWM, PAL_MODE_OUTPUT_PUSHPULL);

  palClearLine(LINE_DRV_M_OC);
  palSetLine(LINE_DRV_OC_ADJ);
  palClearLine(LINE_DRV_M_PWM);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (true) {
    LOG("Hello, World!\r\n");
    thread_t *shell_thd = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
                                                "shell", NORMALPRIO + 1,
                                                shellThread, (void *)&shell_cfg);
    chThdWait(shell_thd); /* Wait for the shell thread to exit */
    chThdSleepMilliseconds(500);
  }
}
