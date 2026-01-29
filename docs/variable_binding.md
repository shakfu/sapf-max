# Signal Rate Variable Binding for sapf~

Implementation proposal for adding signal-rate parameter modulation to the sapf~ Max/MSP external.

---

## Problem Statement

Currently, all parameter control in sapf~ happens via Max messages (text commands). While audio signals can be accessed via `adc`/`adcn` primitives, there is no mechanism for:

1. **Automatic variable binding** - Signal values automatically available as named variables
2. **Control-rate separation** - Distinguishing control signals from audio signals
3. **Efficient parameter modulation** - Avoiding per-sample `adc` calls for simple parameter control

### Current Workaround

Users can use audio inlets as control signals:

```
// Use inlet 1 as filter frequency modulator
0 adc 1 adc 1000 * 200 + lpf play
```

Limitations:
- Requires explicit `adc` call in every expression
- No semantic distinction between audio and control
- Verbose for simple parameter modulation

---

## Proposed Solution

Add dedicated **control inlets** that automatically bind to SAPF variables, updated at configurable rates.

### Object Syntax

```
[sapf~ num_outputs num_inputs num_controls]

[sapf~ 2]           // 2 out, 2 in, 0 control (current default)
[sapf~ 2 2 4]       // 2 out, 2 in, 4 control inlets
[sapf~ 4 1 2]       // 4 out, 1 in, 2 control inlets
```

### Variable Access in SAPF

Control values accessible via indexed or named variables:

**Option A: Indexed access (recommended)**
```
$0              // First control inlet value
$1              // Second control inlet value
$3 1000 * lpf   // Use fourth control as filter freq multiplier
```

**Option B: Named access**
```
ctrl0           // First control inlet
ctrl1           // Second control inlet
```

**Option C: Function access**
```
0 ctrl          // Mirrors adc syntax
1 ctrl
```

### Recommendation

**Option A (indexed `$n`)** provides:
- Concise syntax familiar from shell/Max conventions
- Clear distinction from audio (`adc`) and user variables
- No namespace pollution

---

## Technical Implementation

### 1. Struct Changes

```cpp
typedef struct _sapf {
    t_pxobject ob;

    // Existing fields...
    int numInputChannels;
    int numOutputChannels;

    // New fields for control
    int numControlChannels;           // Number of control inlets (0-8)
    double* controlValues;            // Current control values
    t_symbol** controlSymbols;        // Pre-interned symbols: $0, $1, etc.
    long controlUpdateRate;           // 0=block, 1=sample (attribute)

    // ...rest of struct
} t_sapf;
```

### 2. Object Creation

```cpp
void* sapf_new(t_symbol* s, long argc, t_atom* argv)
{
    t_sapf* x = (t_sapf*)object_alloc(sapf_class);

    // Parse arguments: outputs, inputs, controls
    x->numOutputChannels = (argc > 0) ? atom_getlong(argv) : 2;
    x->numInputChannels = (argc > 1) ? atom_getlong(argv + 1) : 2;
    x->numControlChannels = (argc > 2) ? atom_getlong(argv + 2) : 0;

    // Clamp values
    x->numOutputChannels = CLAMP(x->numOutputChannels, 1, 8);
    x->numInputChannels = CLAMP(x->numInputChannels, 1, 8);
    x->numControlChannels = CLAMP(x->numControlChannels, 0, 8);

    // Total signal inlets = audio + control
    int totalInlets = x->numInputChannels + x->numControlChannels;
    dsp_setup((t_pxobject*)x, totalInlets);

    // Allocate control storage
    if (x->numControlChannels > 0) {
        x->controlValues = new double[x->numControlChannels]();
        x->controlSymbols = new t_symbol*[x->numControlChannels];

        // Pre-intern symbols for efficiency
        char symName[8];
        for (int i = 0; i < x->numControlChannels; i++) {
            snprintf(symName, sizeof(symName), "$%d", i);
            x->controlSymbols[i] = gensym(symName);
        }
    }

    // ...rest of initialization
}
```

### 3. DSP Setup

```cpp
void sapf_dsp64(t_sapf* x, t_object* dsp64, short* count,
                double samplerate, long maxvectorsize, long flags)
{
    // Existing setup...

    // Register perform method
    object_method(dsp64, gensym("dsp_add64"), x, sapf_perform64, 0, nullptr);
}
```

### 4. Perform Routine

```cpp
void sapf_perform64(t_sapf* x, t_object* dsp64, double** ins, long numins,
                    double** outs, long numouts, long sampleframes, long flags, void* userparam)
{
    // Update control variables at block rate
    if (x->numControlChannels > 0) {
        pthread_mutex_lock(&x->threadMutex);

        for (int i = 0; i < x->numControlChannels; i++) {
            // Sample first value of control signal (block rate)
            int inletIndex = x->numInputChannels + i;
            double value = ins[inletIndex][0];
            x->controlValues[i] = value;

            // Bind to SAPF global variable
            // This requires adding a method to Thread or using existing setGlobal
            x->mainThread->setGlobalValue(x->controlSymbols[i]->s_name, V(value));
        }

        pthread_mutex_unlock(&x->threadMutex);
    }

    // Existing audio processing...
    // Copy audio inputs to SAPF buffers (only audio inlets, not control)
    for (int ch = 0; ch < x->numInputChannels; ch++) {
        for (long i = 0; i < sampleframes; i++) {
            x->audioBuffers[ch][i] = (float)ins[ch][i];
        }
    }

    // ...rest of perform routine
}
```

### 5. SAPF Engine Integration

Two approaches for making control variables accessible:

**Approach A: Global Variable Injection**

Add method to `Thread` class:

```cpp
// In Thread.hpp
void setGlobalValue(const char* name, V value) {
    Symbol sym = getSymbol(name);
    mGlobals[sym] = value;
}
```

Pros: Simple, uses existing infrastructure
Cons: Modifies SAPF engine, variables persist after code execution

**Approach B: Prelude-Based Binding**

Define control accessors in prelude:

```
// sapf-prelude.txt additions
{ 0 _ctrl@ } =$0
{ 1 _ctrl@ } =$1
{ 2 _ctrl@ } =$2
// etc.
```

With `_ctrl@` as a new primitive that reads from the control buffer.

Pros: No engine changes for variable binding
Cons: Requires new primitive, indirect access

**Approach C: Special Symbol Resolution (Recommended)**

Modify symbol lookup to check control values for `$n` patterns:

```cpp
// In Thread::getGlobal or symbol resolution
V Thread::resolveSymbol(Symbol sym) {
    const char* name = sym.name();

    // Check for $n pattern
    if (name[0] == '$' && isdigit(name[1])) {
        int index = name[1] - '0';
        if (index < numControls) {
            return V(controlValues[index]);
        }
    }

    // Fall back to normal lookup
    return mGlobals[sym];
}
```

Pros: Clean syntax, lazy evaluation, no prelude changes
Cons: Requires engine modification, slight lookup overhead

---

## Update Rate Options

### Block Rate (Default)

Sample control signal once per audio block (e.g., every 64 or 512 samples).

```cpp
// In perform64, before audio loop
for (int i = 0; i < x->numControlChannels; i++) {
    x->controlValues[i] = ins[x->numInputChannels + i][0];
}
```

- Sufficient for most modulation (LFOs, envelopes)
- Minimal CPU overhead
- ~689 Hz update rate at 44.1kHz/64 samples

### Sample Rate (Optional)

Update control values every sample within SAPF's audio generation.

Requires deeper integration:
- Control buffer passed to SAPF generators
- Per-sample lookup in UGen code

Not recommended for initial implementation due to complexity.

### Attribute Control

```cpp
CLASS_ATTR_LONG(c, "ctrlrate", 0, t_sapf, controlUpdateRate);
CLASS_ATTR_ENUMINDEX(c, "ctrlrate", 0, "block sample");
CLASS_ATTR_LABEL(c, "ctrlrate", 0, "Control Update Rate");
```

---

## Inlet Assist Strings

```cpp
void sapf_assist(t_sapf* x, void* b, long io, long index, char* s)
{
    if (io == ASSIST_INLET) {
        if (index < x->numInputChannels) {
            if (index == 0) {
                snprintf(s, ASSIST_MAX_STRING_LEN,
                    "(signal) Audio input %ld, SAPF messages", index);
            } else {
                snprintf(s, ASSIST_MAX_STRING_LEN,
                    "(signal) Audio input %ld", index);
            }
        } else {
            int ctrlIndex = index - x->numInputChannels;
            snprintf(s, ASSIST_MAX_STRING_LEN,
                "(signal) Control $%d", ctrlIndex);
        }
    }
    // ...outlets
}
```

---

## Usage Examples

### Basic Parameter Modulation

```
// sapf~ 2 1 2  (stereo out, mono in, 2 controls)
// Control 0: filter frequency (0-1 signal)
// Control 1: resonance (0-1 signal)

0 adc $0 5000 * 100 + $1 10 * 0.5 + bpf play
```

### LFO-Controlled Panning

```
// sapf~ 2 1 1  (stereo out, mono in, 1 control)
// Control 0: pan position from [cycle~ 0.5]

0 adc dup $0 * swap 1 $0 - * play
```

### Envelope Following

```
// sapf~ 2 2 1  (stereo out, stereo in, 1 control)
// Control 0: envelope from [peakamp~]

0 adc 1 adc + 0.5 * $0 * play
```

### Multi-Parameter FM

```
// sapf~ 2 0 3  (stereo out, no audio in, 3 controls)
// Control 0: carrier freq
// Control 1: mod freq
// Control 2: mod depth

$0 $1 0 sinosc $2 * + 0 sinosc play
```

---

## Implementation Phases

### Phase 1: Basic Control Inlets
- Add `numControlChannels` argument parsing
- Create additional signal inlets
- Store control values at block rate
- Update inlet assist strings

### Phase 2: Variable Binding
- Implement global variable injection in SAPF Thread
- Bind `$0`-`$7` to control values before code execution
- Test with simple modulation examples

### Phase 3: Documentation
- Update README with control inlet syntax
- Add examples to help patcher
- Document `$n` variable convention

### Phase 4: Optimizations (Optional)
- Sample-rate control option
- Smoothing/interpolation for control signals
- Control signal monitoring via outlet

---

## Alternatives Considered

### 1. Message-Based Parameter Setting

```
[sapf~]
|
[r freq] -> [prepend setvar freq]
```

Rejected: Not signal-rate, adds latency, clutters patch

### 2. Buffer~ Reference

Store control data in `buffer~` objects, read via SAPF.

Rejected: Overcomplicated for simple parameter control

### 3. Named Inlets via Attributes

```
@ctrl1name "freq" @ctrl2name "res"
```

Allows `freq` and `res` as variable names.

Considered for Phase 2: Nice UX but adds complexity

### 4. Lua/JS-Style Closure Capture

Capture Max signals in SAPF closures at definition time.

Rejected: Doesn't match SAPF's evaluation model

---

## Open Questions

1. **Maximum control channels**: 8 matches audio limit, but is this sufficient?

2. **Default values**: Should unconnected control inlets default to 0.0 or be undefined?

3. **Signal vs float inlets**: Should control inlets accept both signals and floats?
   - Pro: More flexible patching
   - Con: Additional complexity

4. **Namespace collision**: What if user defines `$0` as a variable?
   - Option: Control values shadow user definitions
   - Option: Error on collision
   - Option: Different namespace (`_$0`, `%0`)

5. **Thread safety**: Current design updates controls in perform thread, SAPF code runs in main thread. Need synchronization strategy.

---

## Summary

| Aspect | Recommendation |
|--------|----------------|
| Syntax | `[sapf~ outs ins ctrls]` |
| Variable names | `$0`, `$1`, ... `$7` |
| Update rate | Block rate (default) |
| Max controls | 8 |
| Implementation | Global variable injection |

This feature would significantly improve sapf~'s integration with Max's signal flow paradigm while maintaining SAPF's functional programming model.
