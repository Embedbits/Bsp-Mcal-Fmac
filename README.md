# FMAC Peripheral Driver

This module provides an abstraction layer for the **filter math accelerator** (`FMAC`) of STM32 MCUs: the hardware execution of the
**FIR** and **IIR** digital filters and of the vector functions (dot product, convolution, correlation) on 16-bit fixed-point
data, with the 16 x 16-bit multiplier, the wide accumulator, the 256-word local memory and the circular buffers. It offers the
configuration of a filter, the preload of the state, a single sample / block processing and the streaming of the data by the DMA. It is a low-level
MCAL module: it owns the FMAC hardware, the application and the other modules never touch the registers.

The public interface (`Fmac_Port.h`, `Fmac_Types.h`) is **identical for all supported families**. The implementation
for a family lives in its own branch (`Dev/<Family>` for development, `Releases/<Family>` for the released versions), so
a module or an application written against the interface does not change when the MCU family changes.

This README is the landing page and the **specification of the interface** (master). The family branches contain their
own README with everything that is specific to the family: the instances of the device, their settings and the status of
the implementation.

> **Status: design.** No implementation exists yet, the interface below is a proposal. Stories and branches follow.

---

## Intent and purpose

- A digital filter is a loop of multiplications and additions that runs at the sample rate for ever: an anti-alias or a smoothing FIR on the
  samples of the ADC, an IIR notch for a mains hum, an equalizer in the audio chain, the correlation for the detection of a pattern. On the Cortex-M the loop
  costs the CPU a good part of its time; the FMAC does it **in the background**: the samples go in, the filtered samples come out, by the DMA, without the CPU.
- The accelerator has a **local memory of 256 words** divided into the areas (the coefficients `B` of the feedforward part, the coefficients `A` of the
  feedback part, the input data, the output data) that are set by the base pointers and the sizes, with the circular buffers for the input and
  the output. The user thinks in *"a 32-tap FIR with these coefficients"* or *"a biquad IIR with these coefficients and this gain"*; the module lays out the memory, loads the
  coefficients, preloads the state (the initial delay line values), sets the gain and the clipping and starts the filter.
- Two ways to feed the filter: the **block** (a buffer in, a buffer out, `Fmac_Set_FilterProcess`) and the **stream** (`Fmac_Set_StreamStart`: the DMA continuously
  moves the samples from the input buffer to the filter and the results to the output buffer, with the half and the complete callbacks like the audio streams of the Sai module).
- The module hides the differences between the families: the size of the local memory, the supported functions, the DMA structure and the
  interrupts differ, the interface stays the same.

What the module is **not**: it does not design the filter (the calculation of the coefficients is done offline or by the application; the module takes the
`q1.15` coefficients), it does not convert the float to the fixed point beyond the helpers, and it does not process the data beyond 16 bits.

---

## Concept

| Item | Meaning |
|---|---|
| **Function** (`fmac_Function_t`) | `FMAC_FUNC_FIR`, `FMAC_FUNC_IIR` (direct form 1), `FMAC_FUNC_DOT_PRODUCT`, `FMAC_FUNC_CONVOLUTION`, `FMAC_FUNC_CORRELATION`. |
| **Coefficients** | The `B` taps (feedforward) and, for the IIR, the `A` taps (feedback) in the `q1.15` format; the count of each. |
| **Gain** | The output shift (a power of two scaling of the accumulator), `FMAC_GAIN_1` ... `_128`. |
| **Clipping** | `FMAC_CLIP_SATURATE` or `FMAC_CLIP_NONE` of the result to the 16-bit range. |
| **Memory layout** | The areas `X1` (the input and the state), `X2` (the coefficients), `Y` (the output and the feedback) with the sizes and the optional watermark of the circular buffers; calculated by the module from the filter. |
| **Preload** | The initial values of the input and the output history (zero by default). |

Rules:

1. `Fmac_Set_FilterConfig( filterConfig )` sets the function, the coefficients, the gain and the clipping, lays out the memory and **preloads** it; it fails when the
   filter does not fit the local memory (the total of the coefficients, the delay line and the buffers is checked before a register is written).
2. The filter is started by `Fmac_Set_FilterStart`; the module then accepts the samples. A reconfiguration while the filter runs is rejected.
3. `Fmac_Set_FilterProcess( input, inputCnt, output, outputCnt )` pushes a block and collects the results; the number of the outputs follows from the
   function (one output per input for the FIR / IIR, one result for the dot product). The DMA / interrupt / polling mode is selected in the configuration; the same request
   and the same callbacks are used in all modes.
4. `Fmac_Set_StreamStart` / `Fmac_Set_StreamStop` run the continuous streaming with the circular buffers and the half / complete callbacks.
5. The saturation of the accumulator (overflow) and the underflow / overflow of the buffers (the data not supplied or not read in time) are reported by the
   error callback.
6. All parameters are checked before a register is written; the coefficient and the sample types are the `q1.15` fixed-point numbers.

---

## Supported families

Every family gets its own story, its own tasks and its own branches `Dev/<Family>` and `Releases/<Family>` cut from this
master.

| Family | Branch | Accelerator | Status |
|---|---|---|---|
| STM32H5 | `Dev/STM32H5` | `FMAC` on STM32H562, H563, H573 and the H5E / H5F lines: 16 x 16-bit multiplier, 24+2-bit accumulator, 256 x 16-bit local memory, FIR and IIR (direct form 1), dot product, convolution, correlation, circular buffers, DMA read and write channels; not on STM32H503 / H523 / H533 / H543 / H553 | planned (first) |
| STM32G4 | `Dev/STM32G4` | `FMAC` | planned |
| STM32H7 | `Dev/STM32H7` | `FMAC` on the lines that have it | planned |
| STM32U5, STM32U3 | `Dev/STM32U5`, `Dev/STM32U3` | device dependent | planned |
| STM32F4, STM32F7, STM32L4 | `Dev/STM32F4` ... | no FMAC on the device - the module is a stub that returns `FMAC_REQUEST_ERROR` | planned |

A function without a meaning in a family stays in the interface, returns `FMAC_REQUEST_ERROR` and suppresses the unused
parameters (for example a function the accelerator of the device does not have).

---

## Public interface in short (proposal)

| Group | Functions |
|---|---|
| Module management | `Fmac_Get_ModuleVersion`, `Fmac_Init`, `Fmac_Deinit`, `Fmac_Task`, `Fmac_Get_DefaultConfig` |
| Filter | `Fmac_Set_FilterConfig`, `Fmac_Set_FilterStart`, `Fmac_Set_FilterStop`, `Fmac_Set_FilterPreload`, `Fmac_Get_FilterState` |
| Processing | `Fmac_Set_FilterProcess`, `Fmac_Set_StreamStart`, `Fmac_Set_StreamStop`, `Fmac_Get_XferState`, `Fmac_Get_XferError` |
| Conversion | `Fmac_Get_FromFloat`, `Fmac_Get_ToFloat` |
| Interrupts | `Fmac_Set_IrqPriority` / `Get`, the callbacks are a part of the configuration |

```c
fmac_ModuleVersion_t Fmac_Get_ModuleVersion  ( void );

fmac_RequestState_t  Fmac_Init               ( const fmac_Config_t * const fmacConfig );
fmac_RequestState_t  Fmac_Deinit             ( void );
void                 Fmac_Task               ( void );
fmac_RequestState_t  Fmac_Get_DefaultConfig  ( fmac_Config_t * const fmacConfig );

fmac_RequestState_t  Fmac_Set_FilterConfig   ( const fmac_FilterConfig_t * const filterConfig );
fmac_RequestState_t  Fmac_Set_FilterStart    ( void );
fmac_RequestState_t  Fmac_Set_FilterStop     ( void );
fmac_RequestState_t  Fmac_Set_FilterPreload  ( const fmac_Sample_t * const inputHistory, fmac_SampleCnt_t inputCnt, const fmac_Sample_t * const outputHistory, fmac_SampleCnt_t outputCnt );

fmac_RequestState_t  Fmac_Set_FilterProcess  ( const fmac_Sample_t * const input, fmac_SampleCnt_t inputCnt, fmac_Sample_t * const output, fmac_SampleCnt_t outputCnt );
fmac_RequestState_t  Fmac_Set_StreamStart    ( const fmac_StreamRequest_t * const streamRequest );
fmac_RequestState_t  Fmac_Set_StreamStop     ( void );
fmac_RequestState_t  Fmac_Get_FromFloat      ( float value, fmac_Sample_t * const sample );
```

Every function returns `FMAC_REQUEST_OK` / `FMAC_REQUEST_ERROR` and checks its parameters and the state of the module
before it touches a register.

| Type | Meaning |
|---|---|
| `fmac_Config_t` | Transfer mode (`FMAC_XFER_MODE_POLL` / `ISR` / `DMA`), DMA channels, callbacks, interrupt priority |
| `fmac_FilterConfig_t` | Function, the `B` and `A` coefficient arrays and their counts, the gain, the clipping, the circular buffer watermarks |
| `fmac_StreamRequest_t` | The input buffer, the output buffer, the sizes, circular / one shot |
| `fmac_Sample_t` | A `q1.15` sample or coefficient (16 bit signed) |
| Callbacks | `XferHalfCallback`, `XferCompleteCallback`, `ErrorCallback( errorMask )` |

---

## Usage

### A 16-tap low-pass FIR on the stream of the ADC samples

```c
#include "Fmac_Port.h"

static const fmac_Sample_t lowPassTaps[ 16u ] = { /* q1.15 coefficients from the filter design */ };
static fmac_Sample_t       filtered[ 512u ];

fmac_Config_t       fmacConfig;
fmac_FilterConfig_t firConfig;

(void)Fmac_Get_DefaultConfig( &fmacConfig );
fmacConfig.XferMode = FMAC_XFER_MODE_DMA;
(void)Fmac_Init( &fmacConfig );

firConfig.Function  = FMAC_FUNC_FIR;
firConfig.B         = lowPassTaps;
firConfig.BCount    = 16u;
firConfig.Gain      = FMAC_GAIN_1;
firConfig.Clipping  = FMAC_CLIP_SATURATE;

(void)Fmac_Set_FilterConfig( &firConfig );
(void)Fmac_Set_FilterStart();

const fmac_StreamRequest_t stream =
{
    .Input = adcSamples, .InputSize = 512u, .Output = filtered, .OutputSize = 512u, .BufferMode = FMAC_BUFFER_CIRCULAR,
};

(void)Fmac_Set_StreamStart( &stream );                      /* the half / complete callbacks hand over the filtered blocks */
```

---

## Behaviour and limits

- **Memory.** The taps, the delay line and the buffers share 256 words; a long FIR (more than about 100 taps with the buffers) does not fit. The module
  tells how much memory the configuration needs and rejects an oversized one.
- **Numbers.** The data are 16-bit fixed point; the accumulator is wider but the result is clipped or wrapped to 16 bits after the gain - the gain and the coefficient scaling
  are the filter designer's task (the guidance on the overflow is in the family README).
- **Latency.** The filter has a start-up transient of the length of the filter; the preload removes the transient when the history is known.
- **Throughput.** The FMAC computes a tap per clock cycle; the DMA and the memory limit the rate at the high sample rates.
- **Not part of the first version:** the cascade of the several filters in one configuration (the application chains the requests), the IIR in the form other than
  direct form 1, the automatic coefficient design.

---

## Dependencies

- **Rcc** - peripheral clock and reset
- **Nvic** - interrupt
- **Gpdma** - DMA transfers of the input and the output data
- **RAL (Register Abstraction Layer)** - used internally to access the registers

All mandatory RCC, NVIC and DMA configurations are handled internally.

---

## Testing

- **Unit tests** (host, Unity + CMock + RegMem): the memory layout calculation for the filter sizes (the maximum, the overflow of the local memory), the coefficient
  loading and the preload, the gain and the clipping, the process and the stream flow with the callbacks, the error paths (the overflow, the buffer underflow), the rejected
  reconfiguration of a running filter.
- **Integration tests** (target, probe-rs): the FIR and the IIR impulse and step responses compared with a software reference, the dot product and the
  correlation, the DMA stream of a generated sine through a low-pass filter measured for the attenuation.

---

## Interface conventions

The module follows the common rules of the MCAL layer, so that it behaves like the other modules:

- **Lists instead of free members.** Everything that the hardware restricts (the pins, the kernel clock sources, the DMA
  channels, the triggers and the other selectable sources) is a `typedef enum` list with exactly the items that are valid
  for the device line; a pin is an item `FMAC_<SIGNAL>_PIN_<INSTANCE>_P<port><pin>` and there is an `*_UNUSED` item. A choice
  that is not in the list cannot be written.
- **Own peripheral only.** The module calls only the registers / LL functions of its own peripheral. The clocks, the
  interrupts, the pins, the DMA and the EXTI lines go through the API of the Rcc, Nvic, Gpio, Gpdma and Exti modules; a
  missing function is added to that module, it is not bypassed.
- **Init verifies itself.** `Fmac_Init` checks the result at the end; when it is not as required, the module is
  deinitialized and an error is returned. The waiting for the hardware is a loop on the register state with a timeout,
  never a blind delay.
- **Transfer modes.** Where the module moves data, the POLL (served by `Fmac_Task`), ISR and DMA modes have the same
  configuration structure, the same request and the same callbacks (error, half transfer, transfer complete).
- **Flags and states are enums of the module** (`fmac_FunctionState_t`, `fmac_FlagState_t`), the limits and the constants are
  named, there are no magic numbers in the interface.
- **Unsupported functions stay.** A function without a meaning in a family keeps its signature, suppresses the unused
  parameters and returns `FMAC_REQUEST_ERROR`.
- **Documentation.** Every function and type has a Doxygen description, the main header has the `\defgroup`.

---

## Repository layout and branches

The master holds this README and the license. The family branches (`Dev/<Family>`, `Releases/<Family>`) hold:

| Path | Content |
|---|---|
| `README.md` | README of the family: the instances of the device, specifics, errata, status |
| `Fmac_Port.h`, `Fmac_Types.h` | Public interface (identical in all families) |
| `Fmac*.c`, `Fmac.h` | Implementation of the family |
| `CMakeLists.txt` | Library `Fmac_Lib` of the EmBi platform |
| `Tests/UnitTests` | Unit tests (Unity + CMock + RegMem) |
| `Tests/IntegrationTests` | Integration tests on the target (probe-rs) |

A change of the public interface is made in all families.

---

## CMake integration

```cmake
target_link_libraries( User_Lib PRIVATE Fmac_Lib )
```

1. Link `Fmac_Lib` to your CMake library.
2. Include `Fmac_Port.h` in your project.
3. Configure the module as needed for your hardware.

---

## License

This project is licensed under the **Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)**.

You are free to use, modify, and share this work for **non-commercial purposes**, provided appropriate credit is given.

See [LICENSE.md](LICENSE.md) for full terms or visit [creativecommons.org/licenses/by-nc/4.0](https://creativecommons.org/licenses/by-nc/4.0/).

---

## Authors

- **Mr.Nobody** - [embedbits.com](https://embedbits.com)

Contributions are welcome! Please open a pull request.
