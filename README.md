# sapf~

A Max/MSP external that embeds the [SAPF](https://github.com/lfnoise/sapf) (Sound As Pure Form) functional audio programming language, enabling real-time audio synthesis and processing within Max.

## Overview

SAPF is a stack-based, functional programming language designed for audio synthesis and signal processing. This external allows you to write SAPF code directly in Max, combining the power of functional audio programming with Max's patching environment.

**Key Capabilities:**
- Generate audio using oscillators, noise, and other signal sources
- Process incoming audio with filters, delays, and effects
- Use functional programming constructs (closures, higher-order functions)
- Multichannel expansion for polyphonic/spatial audio

## Features

- Direct SAPF code interpretation via Max messages
- Audio input processing via `adc`/`adcn` primitives
- Configurable output channels (1-8)
- Thread-safe operation
- Stack inspection and manipulation
- Prelude with standard library functions

## Building

Requires macOS and the Max SDK.

```sh
make
```

The external will be built to `externals/sapf~.mxo`.

## Installation

Copy `externals/sapf~.mxo` to your Max externals folder:
- `~/Documents/Max 8/Packages/sapf/externals/`
- Or any folder in Max's search path

Copy `source/projects/sapf_tilde/sapf-prelude.txt` to Max's search path for the standard library.

## Usage

### Creating the Object

```
[sapf~]           // 2 output channels (default)
[sapf~ 4]         // 4 output channels
[sapf~ 8]         // 8 output channels (maximum)
```

### Inlets and Outlets

- **Inlet 1**: Audio input + SAPF code messages
- **Outlets 1-N**: Audio outputs (configurable)
- **Last Outlet**: Text output (stack values, status)

## Message API

### SAPF Code Execution

Send SAPF code directly as messages:

```
440 0 sinosc play              // Play 440Hz sine wave
0 adc 1000 lpf play            // Low-pass filter input at 1kHz
stop                           // Stop all audio
```

### Commands

| Command | Description |
|---------|-------------|
| `stop` | Stop all audio playback |
| `clear` | Clear the stack |
| `stack` | Print stack contents |
| `status` | Show engine status |
| `help` | Display help information |

## SAPF Language Basics

SAPF uses reverse Polish notation (RPN) - arguments come before operations.

### Numbers and Arithmetic

```
5 3 +                 // 8 (addition)
10 2 /                // 5 (division)
440 2 *               // 880 (multiplication)
```

### Oscillators

```
// sinosc: freq phase -- signal
440 0 sinosc play              // Sine wave at 440Hz

// saw: freq phase -- signal
220 0 saw play                 // Sawtooth at 220Hz

// pulse: freq phase width -- signal
330 0 0.3 pulse play           // Pulse wave, 30% duty cycle
```

### Audio Input (adc/adcn)

```
// adc: channel -- signal
0 adc play                     // Pass through input channel 0
1 adc play                     // Pass through input channel 1

// adcn: numChannels -- [signals...]
2 adcn play                    // Pass through stereo input
```

### Filters

```
// lpf: input freq -- signal (low-pass filter)
0 adc 1000 lpf play

// hpf: input freq -- signal (high-pass filter)
0 adc 200 hpf play

// bpf: input freq q -- signal (band-pass filter)
0 adc 1000 10 bpf play
```

### Delays

```
// delayl: input delaytime maxdelay -- signal (linear interpolation)
0 adc aa 0.25 0.5 delayl + play       // Quarter-second echo (max 0.5s buffer)
0 adc aa 0.25 0 delayl + play         // Same (0 maxdelay uses delaytime)

// delayc: input delaytime maxdelay -- signal (cubic interpolation)
0 adc aa 0.1 0.5 delayc 0.5 * + play  // With wet/dry mix
```

### Amplitude and Mixing

```
// Multiply for amplitude control
440 0 sinosc 0.5 * play        // Sine at 50% volume

// Add signals together
440 0 sinosc 550 0 sinosc + 0.5 * play   // Mix two sines
```

### Envelopes

```
// perc: attack release curve -- envelope
0.01 0.5 -4 perc               // Percussive envelope

// Apply envelope to oscillator
440 0 sinosc 0.01 0.5 -4 perc * play
```

### Multichannel Expansion

Lists automatically expand across channels:

```
[440 550 660] 0 sinosc play    // Three-channel output

// Stereo panning
440 0 sinosc [0.7 0.3] * play  // Pan left
```

### Variables and Functions

```
// Define a variable
440 =freq

// Use the variable
freq 0 sinosc play

// Define a function
{ 0 sinosc play } =playsine

// Call the function
440 playsine
```

### Higher-Order Functions

```
// Map over a list
[1 2 3 4 5] { 100 * } map      // [100 200 300 400 500]

// Generate frequencies and play
[1 2 3 4] { 110 * } map 0 sinosc play
```

## Examples

### Simple Sine Wave
```
440 0 sinosc play
```

### Stereo Detuned Oscillators
```
[440 442] 0 sinosc play
```

### Filtered Noise
```
whitenoise 800 lpf play
```

### Audio Pass-Through with Filter
```
0 adc 2000 lpf play
```

### Echo/Delay Effect
```
0 adc aa 0.3 0 delayl 0.5 * + play
```

### Ring Modulation
```
0 adc 440 0 sinosc * play
```

### Tremolo
```
440 0 sinosc 5 0 sinosc 0.5 * 0.5 + * play
```

### FM Synthesis
```
440 200 0 sinosc 100 * + 0 sinosc play
```

### Additive Synthesis (Harmonic Series)
```
[1 2 3 4 5 6 7 8] { 110 * } map 0 sinosc
[1 0.5 0.33 0.25 0.2 0.17 0.14 0.12] *
mix play
```

### Karplus-Strong Pluck
```
0.01 whitenoise 0.005 0 delayl 200 lpf play
```

## Platform Support

Currently macOS only.

## Credits

- SAPF language by James McCartney
- Max/MSP external wrapper by [shakfu](https://github.com/shakfu)

## License

GPL-3.0 (following SAPF's license)

## See Also

- [SAPF Language Repository](https://github.com/lfnoise/sapf)
- [CHANGELOG.md](CHANGELOG.md) - Recent changes
- [NEXT_STEPS.md](NEXT_STEPS.md) - Development roadmap
