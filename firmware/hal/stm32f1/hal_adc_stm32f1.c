/* hal_adc_stm32f1.c — the Phase 1 analog channel: a timer's TRGO triggers ADC1, DMA1
 * Channel 1 fills a circular half-word buffer, and every completed half-buffer is handed
 * to the application from the DMA interrupt (FR-1.1).
 *
 *   TIM3 (PSC 71, ARR 999) --TRGO/update--> ADC1 (PA0/IN0, 71.5-cycle sample, 12-bit)
 *        --DMA1 Channel 1, circular--> s_buffer[2 * block_samples] (static, half-words)
 *        --half/full transfer complete--> on_block()
 *
 * WHY TIM3 AND NOT TIM2, since design-spec.md §6.1 and the Phase 1 brief both say "TIM2
 * TRGO -> ADC1 external trigger" and that combination does not exist on this part. On the
 * STM32F103 (RM0008, ADC_CR2 EXTSEL for the *regular* group of ADC1/ADC2) the selectable
 * triggers are T1_CC1, T1_CC2, T1_CC3, T2_CC2, T3_TRGO, T4_CC4 and EXTI11. TIM2's TRGO is
 * available to the *injected* group (JEXTSEL = 010) and to nothing else, and the injected
 * group is not the path this firmware uses: it has four dedicated result registers and no
 * DMA request, so it could not fill a circular buffer at all. The property the decision
 * asks for — the trigger is the timer's hardware output, not a CPU write — is delivered by
 * TIM3's update event on TRGO, which is the same signal on a different timer. TIM3 is free
 * (TIM4 is the HAL timebase), so nothing else had to move; the report of this change says
 * so rather than leaving a reader to wonder why the code and the spec disagree.
 *
 * Three properties are worth reading the code for rather than taking on trust:
 *
 *   1. NOTHING IN THE ACQUISITION PATH POLLS. The only functions called after start() are
 *      the DMA interrupt handler and the two HAL callbacks it reaches, and neither reads a
 *      status flag to decide anything: the interrupt *is* the notification. The one place a
 *      flag is waited on is the ADC calibration in start(), which waits for the
 *      calibration-complete bit of CR2 — a self-test the part runs with the converter
 *      disabled and no conversion involved — and the only alternative is to sample
 *      uncalibrated and call the result a measurement.
 *
 *   2. THE CALLBACK RUNS IN INTERRUPT CONTEXT and is passed a buffer the DMA engine is
 *      still writing into. It does not copy, allocate, lock or yield: it calls the
 *      application's callback, which copies (AcquisitionService::on_block) and returns.
 *      The handover is safe because it is a *half*-buffer: while the callback runs, the
 *      converter is filling the other half, so no sample is missed and no conversion has
 *      to be restarted.
 *
 *   3. THE REQUESTED RATE IS CHECKED AGAINST BOTH CLOCKS THAT PRODUCE IT, and start()
 *      returns false rather than starting something slower than asked for. A control
 *      period that is a count of blocks is only as good as the rate those blocks arrive
 *      at, so silently achieving 999 Hz where 1000 was requested would show up as an
 *      unexplained bias in every control period.
 *
 * The numbers behind the configuration — ADCCLK, the sampling time, and the arithmetic
 * that sizes the sampling time for a ~10 kohm source — live in app/board.h with the rest of
 * the Phase 1 constants, and are restated in hal_stm32f1.h's hardware assignment. They are
 * repeated here where the HAL can check them against the register encodings it actually
 * writes, so a change on one side of the duplication fails the build instead of drifting.
 */

#include "hal_stm32f1.h"

#include "stm32f1xx_hal.h"

#include <stddef.h>

/* PA0 = ADC1_IN0. The probe's analog output arrives here: through the AFE buffer and RC
 * when that exists (FR-1.2, FR-3.10), and straight from the probe for first light — which
 * is why the sampling time is sized for the source impedance rather than for a buffered
 * one. */
#define ADC_GPIO_PORT GPIOA
#define ADC_GPIO_PIN GPIO_PIN_0
#define ADC_GPIO_CHANNEL ADC_CHANNEL_0

/* The 72 MHz timer clock (APB1 prescaler is 2, so the F1 doubles it). PSC 71 -> a 1 MHz
 * counter; ARR is solved from the requested rate. */
#define ADC_TRIGGER_TIM TIM3
#define ADC_TRIGGER_CLOCK_HZ 72000000U

/* The prescaler this file uses, chosen so the counter is exactly 1 MHz. A finer counter
 * would buy nothing at the rates this system measures, and a coarser one would make the
 * achievable-rate check below reject values the timer could otherwise hit. */
#define ADC_TRIGGER_PRESCALER (ADC_TRIGGER_CLOCK_HZ / 1000000U)

/* The largest block the static DMA buffer can hold. This exists because hal/adc.h requires
 * the buffer to be static (the part has no heap budget for it) while block_samples is a
 * run-time argument, so the implementation has to have a ceiling and refuse anything above
 * it. 256 samples per half is 512 half-words = 1 KB, 5 % of this part's RAM; the
 * application asks for 100, so the ceiling guards a future caller rather than limiting
 * anything today. A refused request returns false with nothing started, as hal/adc.h
 * documents, rather than truncating to a size the caller did not ask for. */
#define ADC_MAX_BLOCK_SAMPLES 256U

/* AT ADCCLK = PCLK2 / 6 = 12 MHz and 71.5 sampling cycles, one conversion is
 * 71.5 + 12.5 = 84 ADCCLK cycles = 7.0 us, so the converter's ceiling with this sampling
 * time is 142.8 kSPS. Both figures are asserted against the encodings written below. */
#define ADC_CYCLES_PER_CONVERSION 84U
#define ADC_ADCCLK_DIVISOR 6U

/* The register encoding of the sampling time, so the comment above and the code below
 * cannot disagree: SMP = 6 is "71.5 cycles" in RM0008's table (1.5, 7.5, 13.5, 28.5, 41.5,
 * 55.5, 71.5, 239.5 for SMP = 0..7), and the HAL names that encoding
 * ADC_SAMPLETIME_71CYCLES_5. */
#define ADC_SAMPLING_CYCLES 71U
_Static_assert(ADC_SAMPLETIME_71CYCLES_5 == 6U,
               "the HAL's 71.5-cycle sampling time must still be SMP = 6");
_Static_assert(ADC_SAMPLETIME_1CYCLE_5 == 0U && ADC_SAMPLETIME_7CYCLES_5 == 1U,
               "the sampling-time encodings moved: re-derive the cycle counts above");
_Static_assert(ADC_CYCLES_PER_CONVERSION > ADC_SAMPLING_CYCLES,
               "a conversion must take longer than its sampling phase");
_Static_assert(ADC_EXTERNALTRIGCONV_T3_TRGO != ADC_SOFTWARE_START,
               "the external trigger must not be the software start");

/* Interrupt priority. The block callback takes no RTOS lock, so it could legally sit above
 * configMAX_SYSCALL_INTERRUPT_PRIORITY; 6 is the conservative choice, matching the other
 * peripheral ISRs, so that an edit which reaches for a kernel API from the callback is
 * caught by the priority rule rather than by corrupting a critical section. The cost is
 * that a handover can be delayed by the length of a kernel critical section — microseconds
 * against a block period of 100 ms. */
#define ADC_IRQ_PRIORITY 6U

/* Owned by this file. Half-words because the ADC's data register is 16 bits and the DMA is
 * configured for half-word transfers; two halves because that is what makes the handover
 * atomic. */
static uint16_t s_buffer[2U * ADC_MAX_BLOCK_SAMPLES];
static uint32_t s_buffer_half_words = 0U;

static ADC_HandleTypeDef s_adc1;
static DMA_HandleTypeDef s_dma_adc;
static TIM_HandleTypeDef s_trigger_tim;

static hal_adc_block_fn s_on_block = NULL;
static void *s_on_block_ctx = NULL;

/* Counters the interface reports. `s_blocks` is written in interrupt context and read by a
 * task, so it is volatile: a 32-bit aligned store is atomic on this core, and this value is
 * an accounting aid (hal/adc.h is explicit that nothing decides anything from it) rather
 * than a synchronisation primitive. */
static volatile uint32_t s_blocks = 0U;
static volatile bool s_running = false;

/* Which half of the buffer the last interrupt covered. The HAL's DMA callbacks do not say
 * where the data landed — they say *that* a half or the whole buffer completed — so the
 * pairing is tracked here: half-complete hands over the first half, transfer-complete the
 * second, and the next half-complete starts again. The DMA's own ordering makes this
 * unambiguous, and it is why the length passed to HAL_ADC_Start_DMA is 2 * block_samples
 * rather than a ring of blocks. */
static volatile bool s_first_half_taken = false;

/* ---------------------------------------------------------------------------
 * Rate arithmetic
 * ---------------------------------------------------------------------------
 * The trigger rate is 72 MHz / (PSC + 1) / (ARR + 1), so the timer can produce any rate
 * that divides its 1 MHz counter tick exactly. The converter imposes the other limit: one
 * conversion is 84 ADCCLK cycles, so it cannot be triggered faster than ADCCLK / 84.
 *
 * The achievable range is therefore 15.3 Hz (1 MHz / 65536, the 16-bit auto-reload at this
 * prescaler) to 142.8 kSPS (12 MHz / 84), and only rates that divide 1 MHz exactly are
 * accepted inside it. 1 kHz divides exactly: ARR = 999 and PSC = 71 give
 * 72 MHz / 72 / 1000 = 1000.000 Hz, so kAdcSampleRateHz needs no correction and the block
 * period is exactly 100 ms.
 *
 * The divisibility rule is deliberate rather than a "nearest achievable rate" search: the
 * request is for a fixed rate, and 1 kHz is what makes ten blocks one second. A caller that
 * wants a rate this clock tree cannot hit exactly is told no, which is the answer
 * hal/adc.h's false return is for.
 */
static bool adc_rate_achievable(uint32_t sample_rate_hz, uint32_t *out_arr) {
  if (sample_rate_hz == 0U || out_arr == NULL) {
    return false;
  }

  const uint32_t adcclk_hz = ADC_TRIGGER_CLOCK_HZ / ADC_ADCCLK_DIVISOR;
  if (sample_rate_hz > (adcclk_hz / ADC_CYCLES_PER_CONVERSION)) {
    return false; /* faster than the converter can convert */
  }

  const uint32_t counter_hz = ADC_TRIGGER_CLOCK_HZ / ADC_TRIGGER_PRESCALER; /* 1 MHz */
  if (sample_rate_hz < (counter_hz / 65536U)) {
    return false; /* slower than the auto-reload register can express */
  }
  if ((counter_hz % sample_rate_hz) != 0U) {
    return false; /* not a whole number of counter ticks per trigger */
  }

  *out_arr = (counter_hz / sample_rate_hz) - 1U;
  return true;
}

/* Arm the DMA and the ADC's external trigger. The F1 HAL takes a uint32_t* for the buffer
 * even though the DMA is set up for half-word transfers, a known quirk of this family's
 * driver: the pointer is only ever used as an address. The cast goes through uintptr_t so
 * the mismatch is stated at one place rather than hidden in an implicit conversion that
 * -Wconversion would flag anyway. */
static bool adc_arm_dma(uint32_t half_words) {
  // NOLINTNEXTLINE(performance-no-int-to-ptr) -- the address of a real array, see above
  uint32_t *as_words = (uint32_t *)(uintptr_t)s_buffer;
  return HAL_ADC_Start_DMA(&s_adc1, as_words, 2U * half_words) == HAL_OK;
}

/* ---------------------------------------------------------------------------
 * Bring-up: MspInit (called by HAL_ADC_Init) and the module init
 * ---------------------------------------------------------------------------
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance != ADC1) {
    return;
  }

  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Analog mode: a digital input's buffer and Schmitt trigger would load the source, and
   * the Schmitt threshold would make the pin a comparator rather than an ADC input. This is
   * the only mode in which PA0 is ADC1_IN0. */
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = ADC_GPIO_PIN;
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC_GPIO_PORT, &gpio);

  __HAL_RCC_ADC1_CLK_ENABLE();

  /* DMA1 Channel 1 is hardwired to the ADC1 request on this family — there is no
   * channel-selection register to configure, unlike the DMA controllers in later parts.
   * Circular, because the whole point is a free-running double buffer: the channel must
   * never stop at the end of the buffer, or one half-buffer per lap would be dropped and
   * the DMA request would be lost with it. Very high priority: nothing else in this image
   * moves data behind the CPU's back at 1 kHz, and the I2C channel's transfers are short
   * and its own transfers are already serialised by the arbiter. */
  __HAL_RCC_DMA1_CLK_ENABLE();

  s_dma_adc.Instance = DMA1_Channel1;
  s_dma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
  s_dma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
  s_dma_adc.Init.MemInc = DMA_MINC_ENABLE;
  s_dma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  s_dma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  s_dma_adc.Init.Mode = DMA_CIRCULAR;
  s_dma_adc.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  (void)HAL_DMA_Init(&s_dma_adc);
  __HAL_LINKDMA(hadc, DMA_Handle, s_dma_adc);

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, ADC_IRQ_PRIORITY, 0U);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance != ADC1) {
    return;
  }

  HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
  (void)HAL_DMA_DeInit(&s_dma_adc);

  __HAL_RCC_DMA1_CLK_DISABLE();
  __HAL_RCC_ADC1_CLK_DISABLE();
}

/* The peripheral setup that does not depend on the requested rate or block size. Called
 * from hal_stm32f1_init(), before the scheduler: the handle exists from here on, but
 * nothing converts until start() arms the trigger.
 *
 * The F1's ADC_InitTypeDef is smaller than the later families': there is no EOCSelection
 * and no DMAContinuousRequests, because on this part the DMA request is raised once per
 * conversion by the hardware and ADC_CR2_DMA stays set until the channel is stopped. There
 * is also no clock prescaler here — ADCCLK comes from RCC_CFGR.ADCPRE, set in the clock
 * setup — and no resolution field, because a 12-bit part has no other resolution to
 * choose. */
void hal_adc_stm32f1_init(void) {
  s_adc1.Instance = ADC1;
  s_adc1.Init.ScanConvMode = ADC_SCAN_DISABLE; /* one channel, so no sequencer */
  s_adc1.Init.ContinuousConvMode = DISABLE;    /* the trigger decides when, not the ADC */
  s_adc1.Init.DiscontinuousConvMode = DISABLE;
  s_adc1.Init.NbrOfDiscConversion = 0U;
  s_adc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  s_adc1.Init.DataAlign = ADC_DATAALIGN_RIGHT; /* 0..4095, unsigned */
  s_adc1.Init.NbrOfConversion = 1;

  (void)HAL_ADC_Init(&s_adc1);
}

/* ---------------------------------------------------------------------------
 * The interface's operations
 * ---------------------------------------------------------------------------
 */
static void adc_stop(void *ctx);

static bool adc_start(void *ctx, uint32_t sample_rate_hz, size_t block_samples,
                      hal_adc_block_fn on_block, void *on_block_ctx) {
  (void)ctx;

  if (on_block == NULL || block_samples == 0U ||
      block_samples > (size_t)ADC_MAX_BLOCK_SAMPLES) {
    return false;
  }

  uint32_t arr = 0U;
  if (!adc_rate_achievable(sample_rate_hz, &arr)) {
    return false;
  }

  /* Reconfiguring a running acquisition is allowed by hal/adc.h ("calling it twice
   * reconfigures and restarts"), so the previous run is stopped first: a DMA channel still
   * armed while the ADC is re-initialised is how a half-buffer of the old configuration
   * gets handed over as if it belonged to the new one. */
  if (s_running) {
    adc_stop(NULL);
  }

  s_on_block = on_block;
  s_on_block_ctx = on_block_ctx;
  s_blocks = 0U;
  s_first_half_taken = false;
  s_buffer_half_words = (uint32_t)block_samples;

  for (size_t i = 0; i < (2U * block_samples); ++i) {
    s_buffer[i] = 0U;
  }

  /* The trigger timer. Started at the very end, after the DMA is armed: the first update
   * event must arrive when there is somewhere for the conversion's DMA request to go. */
  __HAL_RCC_TIM3_CLK_ENABLE();
  s_trigger_tim.Instance = ADC_TRIGGER_TIM;
  s_trigger_tim.Init.Prescaler = ADC_TRIGGER_PRESCALER - 1U; /* 1 MHz counter */
  s_trigger_tim.Init.Period = arr;
  s_trigger_tim.Init.ClockDivision = 0U;
  s_trigger_tim.Init.CounterMode = TIM_COUNTERMODE_UP;
  s_trigger_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&s_trigger_tim) != HAL_OK) {
    return false;
  }

  /* TRGO = the timer's update event. This is the line that makes the sample instants
   * hardware-timed: the ADC is triggered by the timer's own output signal, so the jitter of
   * the instants is the timer's, not the CPU's, and no software touches a register per
   * sample. */
  TIM_MasterConfigTypeDef master = {0};
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  master.MasterOutputTrigger = TIM_TRGO_UPDATE;
  if (HAL_TIMEx_MasterConfigSynchronization(&s_trigger_tim, &master) != HAL_OK) {
    return false;
  }

  /* Calibration. RM0008 requires it after each power-up: RSTCAL is written, then CAL, and
   * the hardware clears each when it is done. The HAL bounds both waits, so a part that
   * never completes returns HAL_ERROR here instead of hanging the boot — which matters
   * because this runs inside Node::init(), and a hang there would present as a board that
   * never boots with no clue why. */
  if (HAL_ADCEx_Calibration_Start(&s_adc1) != HAL_OK) {
    return false;
  }

  ADC_ChannelConfTypeDef channel = {0};
  channel.Channel = ADC_GPIO_CHANNEL;
  channel.Rank = 1U;
  /* 71.5 ADCCLK cycles = 5.96 us at 12 MHz. The arithmetic that makes this the right
   * number for a ~10 kohm source — 11 kohm x 8 pF x ln(2^14) = 854 ns — is in app/board.h
   * with the rest of the acquisition decisions; the margin is 7x because the probe's own
   * output impedance is unknown until the AFE exists. */
  channel.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&s_adc1, &channel) != HAL_OK) {
    return false;
  }

  if (!adc_arm_dma(s_buffer_half_words)) {
    return false;
  }

  if (HAL_TIM_Base_Start(&s_trigger_tim) != HAL_OK) {
    (void)HAL_ADC_Stop_DMA(&s_adc1);
    return false;
  }

  s_running = true;
  return true;
}

static void adc_stop(void *ctx) {
  (void)ctx;

  /* Timer first: stopping the DMA while the trigger still fires would leave conversions
   * running with no destination, and the ADC's overrun flag set. */
  (void)HAL_TIM_Base_Stop(&s_trigger_tim);
  (void)HAL_ADC_Stop_DMA(&s_adc1);

  s_running = false;
  s_on_block = NULL;
  s_on_block_ctx = NULL;
  s_buffer_half_words = 0U;
}

static uint32_t adc_blocks(void *ctx) {
  (void)ctx;
  return s_blocks;
}

static hal_adc_iface_t s_adc = {
    .ctx = NULL,
    .start = adc_start,
    .stop = adc_stop,
    .blocks = adc_blocks,
};

hal_adc_iface_t *hal_adc_stm32f1(void) {
  return &s_adc;
}

/* ---------------------------------------------------------------------------
 * The handover
 * ---------------------------------------------------------------------------
 * One block: call the application's callback with the half of the buffer the DMA engine has
 * finished with, then count it. `count` is always exactly the requested block size, because
 * the DMA channel's length is 2 * block_samples and both interrupts therefore cover a whole
 * half; a short block is not something this implementation can produce, and hal/adc.h's
 * contract is that an implementation which could produce one reports it rather than
 * pretending the block is complete.
 *
 * The counter is incremented after the callback, so a consumer that compares its own
 * received-block count against blocks() during the callback (AcquisitionService does not,
 * but the callback is public and a future one might) sees the count of blocks *before* this
 * one rather than a number that includes data it has not been handed.
 */
static void adc_hand_over(uint32_t first_sample) {
  const uint32_t half_words = s_buffer_half_words;
  if (half_words == 0U) {
    return;
  }

  if (s_on_block != NULL) {
    s_on_block(s_on_block_ctx, &s_buffer[first_sample], (size_t)half_words, s_blocks);
  }
  s_blocks++;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance != ADC1 || !s_running) {
    return;
  }

  /* The first half-buffer is complete: samples [0, block_samples). This runs while the
   * converter fills the second half, which is what makes the handover atomic. */
  s_first_half_taken = true;
  adc_hand_over(0U);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance != ADC1 || !s_running) {
    return;
  }

  /* The second half-buffer is complete: samples [block_samples, 2 * block_samples). The
   * circular channel is already pointing back at the start of the buffer, so the next
   * interrupt is a half-complete one again. */
  if (s_first_half_taken) {
    adc_hand_over(s_buffer_half_words);
    s_first_half_taken = false;
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance != ADC1) {
    return;
  }

  /* An overrun means the DMA did not collect the data register in time. There is nothing
   * useful to do from interrupt context, and hal/adc.h has no error channel to report it
   * on, so the evidence is left where it is readable — the HAL's error code, kept by
   * HAL_ADC_ErrorCallback's caller — and acquisition continues: one overrun costs one
   * sample, while stopping the channel would cost every sample until someone noticed. */
}

void DMA1_Channel1_IRQHandler(void) {
  HAL_DMA_IRQHandler(&s_dma_adc);
}
