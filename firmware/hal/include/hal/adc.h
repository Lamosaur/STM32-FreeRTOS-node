/* adc.h — the analog acquisition interface the application layer is written against.
 *
 * FR-1.1 does not say "read the ADC". It says the conversion is triggered by a hardware
 * timer at a fixed rate and transferred by DMA into a double buffer, *with no CPU polling
 * of the conversion-complete flag*. That last clause is a statement about what the code
 * must not contain, so it is made structural here rather than left to review: this
 * interface has no "is a conversion ready?" entry and no "read the result register"
 * entry. There is no way to write polling code against it, and therefore no way for a
 * later change to reintroduce polling without changing this header, which is the point at
 * which someone would have to argue for it.
 *
 * The only way samples arrive is a completed block handed over from interrupt context:
 * the DMA half/full-transfer-complete interrupt fires when one half of the circular
 * buffer is filled, and the implementation invokes the callback registered by start().
 * Half-buffer transfers are what make the handover atomic — the timer keeps converting
 * into the other half while the callback is running, so no sample is ever missed and no
 * conversion has to be restarted.
 *
 * The application side of that contract (copy out of the hardware buffer, no work in the
 * ISR) lives in firmware/app/src/acquisition.cpp, per design-spec.md §6: callbacks only
 * signal, and no processing happens in interrupt context, which is what keeps NFR-1.9's
 * latency defensible.
 */
#ifndef HAL_ADC_H
#define HAL_ADC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One completed block, handed over from interrupt context.
 *
 * The callback's contract, all of which a conforming implementation must guarantee and
 * every implementation of the callback must honour:
 *
 *   1. It runs in interrupt context, at the DMA interrupt's priority. It must not block,
 *      must not allocate, must not take a mutex, and must not call anything that does any
 *      of those. On this system a mutex here would be a priority inversion on the
 *      control path, which is the failure FR-1.7 exists to measure elsewhere and which
 *      must not be introduced here.
 *
 *   2. `samples` points to a HARDWARE-OWNED buffer: the DMA channel is still pointed at
 *      it and the conversion is still running. The contents are valid for the duration of
 *      the call and only for the duration of the call — the instant the callback returns,
 *      the DMA engine may overwrite them with the next block. A consumer that needs the
 *      data after returning must copy it out, which is what the application layer does.
 *
 *   3. `count` is the number of samples in this block and equals the block_samples passed
 *      to start(). It is passed rather than assumed so that an implementation which
 *      cannot deliver a partial half-buffer reports that rather than handing over a short
 *      block that silently changes the averaging window.
 *
 *   4. `block_index` counts blocks since start(), starting at 0, and is the producer's own
 *      numbering. It is not a sequence number of accepted data: blocks can be dropped on
 *      the consumer side, so a consumer uses it to notice that the producer restarted
 *      (the index returns to 0).
 *
 *   5. It must return promptly. The next half-buffer completes one block period later —
 *      100 ms at the 1 kHz / 100-sample configuration in app/board.h — and if the callback
 *      is still running then, that block is lost. Nothing in this design is close to that
 *      budget, but the budget is the reason the rule is "signal, do not process".
 */
typedef void (*hal_adc_block_fn)(void *ctx, const uint16_t *samples, size_t count,
                                 uint32_t block_index);

typedef struct hal_adc_iface {
  void *ctx;

  /* Start free-running acquisition: a timer's TRGO output drives the ADC's external
   * trigger at `sample_rate_hz`, conversions land in a circular DMA buffer of
   * 2 * block_samples half-words, and every completed half-buffer is handed to
   * `on_block` with `on_block_ctx` as its first argument.
   *
   * `block_samples` must be at least 1. The DMA buffer is 2 * block_samples * 2 bytes and
   * is owned by the implementation (static storage — the part has no heap budget for
   * this); the caller never sees it, only copies handed to the callback.
   *
   * Returns false if the rate is not achievable, in which case nothing is started and no
   * callback will arrive.
   *
   * The achievable range, so a caller knows a false return means "impossible" rather than
   * "busy": on the STM32F103 with the 72 MHz clock tree, ADCCLK is PCLK2/6 = 12 MHz and a
   * 12-bit conversion takes 14 to 20 ADCCLK cycles depending on the sample time, so the
   * ceiling is roughly 600 kSPS (12 MHz / 19.5 cycles with the 7.5-cycle sample time).
   * The floor is set by the trigger timer, which can divide 72 MHz down to about 17 mHz
   * with its prescaler and auto-reload register, so any rate that matters here is
   * reachable; the practical floor is the rate below which nothing is being measured.
   * These figures come from the reference manual and the datasheet, not from a
   * measurement — docs/measurements.md records what is still unmeasured.
   *
   * There is no "already running" state to query, deliberately: this is called once at
   * start-up, and calling it twice reconfigures and restarts.
   */
  bool (*start)(void *ctx, uint32_t sample_rate_hz, size_t block_samples,
                hal_adc_block_fn on_block, void *on_block_ctx);

  /* Stop acquisition and release the DMA channel and the trigger timer. Safe to call when
   * not started. After this returns, no further callbacks will arrive. */
  void (*stop)(void *ctx);

  /* Blocks handed over since start(). For accounting, not for flow control: nothing in
   * the application decides what to do next based on this number, it exists so a consumer
   * can compare its own received-block count against what the peripheral claims it
   * delivered and see whether a callback was lost rather than merely late.
   */
  uint32_t (*blocks)(void *ctx);
} hal_adc_iface_t;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HAL_ADC_H */
