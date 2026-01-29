# MIDI Support for sapf~

Implementation proposal for enabling MIDI functionality in the sapf~ Max/MSP external.

---

## Current State

SAPF has a complete MIDI subsystem that is currently disabled in the Max context:

### SAPF MIDI Architecture

```
CoreMidiBackend          MidiRouter              gMidiState[16][16]
     |                       |                         |
     | raw bytes             | parsed messages         | UGens read
     +---------------------> +-----------------------> +-------------> MCtl, MBend, etc.
```

**Components:**

| File | Purpose |
|------|---------|
| `MidiBackend.hpp` | Abstract interface for MIDI I/O |
| `CoreMidiBackend.cpp` | macOS Core MIDI implementation |
| `MidiRouter.cpp` | Parses MIDI bytes, updates `gMidiState` |
| `Midi.cpp` | SAPF ops that read from `gMidiState` |

### Available SAPF MIDI Ops

**Control:**
- `midiStart`, `midiStop`, `midiRestart`, `midiList`
- `midiConnectInput`, `midiDisconnectInput`
- `midiDebug`

**Instantaneous Values:**
- `mctl1`, `mpoly1`, `mtouch1`, `mbend1` - Linear mapping
- `xmctl1`, `xmpoly1`, `xmtouch1`, `xmbend1` - Exponential mapping
- `mprog1`, `mgate1`, `mlastkey1`, `mlastvel1`

**Signal UGens (smoothed):**
- `mctl`, `mpoly`, `mtouch`, `mbend` - Linear mapping
- `xmctl`, `xmpoly`, `xmtouch`, `xmbend` - Exponential mapping
- `mprog`, `mgate`, `mlastkey`, `mlastvel`

### Global State Structure

```cpp
// MidiRouter.hpp
struct MidiChanState {
    uint8_t control[128];     // CC values
    uint8_t keyvel[128];      // Note velocities (0 = off)
    uint8_t polytouch[128];   // Polyphonic aftertouch
    uint8_t touch;            // Channel pressure
    uint8_t program;          // Program change
    int16_t bend;             // Pitch bend (-8192 to 8191)
    uint8_t lastkey;          // Most recent note on
    uint8_t lastvel;          // Most recent velocity
    int numKeysDown;          // Active note count
};

extern MidiChanState gMidiState[kMaxMidiPorts][16];  // 16 ports x 16 channels
```

---

## Problem Statement

In Max/MSP, MIDI is typically handled through dedicated objects:
- `[midiin]`, `[midiout]` - Raw MIDI
- `[notein]`, `[noteout]` - Note messages
- `[ctlin]`, `[ctlout]` - Control change
- `[bendin]`, `[bendout]` - Pitch bend

The current sapf~ external cannot use SAPF's MIDI ops because:
1. Core MIDI backend is not initialized
2. No way to inject Max MIDI messages into SAPF's state
3. SAPF's direct Core MIDI access would bypass Max's MIDI infrastructure

---

## Proposed Solution

### Approach: Max MIDI Bridge

Create a bridge that allows Max MIDI messages to populate `gMidiState`, enabling all existing SAPF MIDI ops.

**Benefits:**
- Integrates with Max's MIDI paradigm
- Users can use familiar Max objects for device selection/routing
- All existing SAPF MIDI ops work unchanged
- No conflicts with system MIDI devices
- Flexible routing (filter channels, merge sources, etc.)

### Message API

```
// Note messages
midi note <srcIndex> <channel> <pitch> <velocity>

// Control change
midi cc <srcIndex> <channel> <controller> <value>

// Pitch bend (14-bit: 0-16383, center=8192)
midi bend <srcIndex> <channel> <value>

// Channel pressure (aftertouch)
midi touch <srcIndex> <channel> <value>

// Polyphonic aftertouch
midi polytouch <srcIndex> <channel> <pitch> <value>

// Program change
midi program <srcIndex> <channel> <program>

// Raw MIDI bytes (parsed by MidiRouter)
midi raw <srcIndex> <byte1> [byte2] [byte3]

// Control
midi start          // Initialize MIDI state
midi stop           // Reset MIDI state
midi debug <0|1>    // Enable/disable debug output
```

### Parameter Conventions

| Parameter | Range | Notes |
|-----------|-------|-------|
| srcIndex | 0-15 | MIDI source/port index |
| channel | 1-16 | MIDI channel (1-based, matches SAPF) |
| pitch | 0-127 | MIDI note number |
| velocity | 0-127 | 0 = note off |
| controller | 0-127 | CC number |
| value | 0-127 | CC/pressure value |
| bend | 0-16383 | 14-bit, 8192 = center |
| program | 0-127 | Program number |

---

## Technical Implementation

### 1. Message Handler

```cpp
void sapf_midi(t_sapf* x, t_symbol* s, long argc, t_atom* argv)
{
    if (argc < 1) {
        object_error((t_object*)x, "midi: requires subcommand");
        return;
    }

    t_symbol* cmd = atom_getsym(argv);

    if (cmd == gensym("note")) {
        sapf_midi_note(x, argc - 1, argv + 1);
    }
    else if (cmd == gensym("cc")) {
        sapf_midi_cc(x, argc - 1, argv + 1);
    }
    else if (cmd == gensym("bend")) {
        sapf_midi_bend(x, argc - 1, argv + 1);
    }
    else if (cmd == gensym("touch")) {
        sapf_midi_touch(x, argc - 1, argv + 1);
    }
    else if (cmd == gensym("polytouch")) {
        sapf_midi_polytouch(x, argc - 1, argv + 1);
    }
    else if (cmd == gensym("program")) {
        sapf_midi_program(x, argc - 1, argv + 1);
    }
    else if (cmd == gensym("raw")) {
        sapf_midi_raw(x, argc - 1, argv + 1);
    }
    else if (cmd == gensym("start")) {
        MidiRouter::instance().resetState();
        post("sapf~: MIDI state initialized");
    }
    else if (cmd == gensym("stop")) {
        MidiRouter::instance().resetState();
        post("sapf~: MIDI state reset");
    }
    else if (cmd == gensym("debug")) {
        gMidiDebug = (argc > 1) ? atom_getlong(argv + 1) != 0 : true;
        post("sapf~: MIDI debug %s", gMidiDebug ? "on" : "off");
    }
    else {
        object_error((t_object*)x, "midi: unknown subcommand '%s'", cmd->s_name);
    }
}
```

### 2. Individual Message Handlers

```cpp
// midi note <srcIndex> <channel> <pitch> <velocity>
void sapf_midi_note(t_sapf* x, long argc, t_atom* argv)
{
    if (argc < 4) {
        object_error((t_object*)x, "midi note: requires srcIndex channel pitch velocity");
        return;
    }

    int srcIndex = CLAMP((int)atom_getlong(argv), 0, 15);
    int chan = CLAMP((int)atom_getlong(argv + 1) - 1, 0, 15);  // Convert to 0-based
    int pitch = CLAMP((int)atom_getlong(argv + 2), 0, 127);
    int vel = CLAMP((int)atom_getlong(argv + 3), 0, 127);

    MidiChanState& state = gMidiState[srcIndex][chan];

    if (vel > 0) {
        state.lastkey = pitch;
        state.lastvel = vel;
        state.numKeysDown++;
    } else {
        if (state.numKeysDown > 0) state.numKeysDown--;
    }
    state.keyvel[pitch] = vel;

    if (gMidiDebug) {
        post("sapf~ midi note: src=%d ch=%d pitch=%d vel=%d", srcIndex, chan+1, pitch, vel);
    }
}

// midi cc <srcIndex> <channel> <controller> <value>
void sapf_midi_cc(t_sapf* x, long argc, t_atom* argv)
{
    if (argc < 4) {
        object_error((t_object*)x, "midi cc: requires srcIndex channel controller value");
        return;
    }

    int srcIndex = CLAMP((int)atom_getlong(argv), 0, 15);
    int chan = CLAMP((int)atom_getlong(argv + 1) - 1, 0, 15);
    int cc = CLAMP((int)atom_getlong(argv + 2), 0, 127);
    int val = CLAMP((int)atom_getlong(argv + 3), 0, 127);

    gMidiState[srcIndex][chan].control[cc] = val;

    // Handle special CCs
    if (cc == 120 || (cc >= 123 && cc <= 127)) {
        // All notes off
        memset(gMidiState[srcIndex][chan].keyvel, 0, 128);
        gMidiState[srcIndex][chan].numKeysDown = 0;
    } else if (cc == 121) {
        // Reset all controllers
        memset(gMidiState[srcIndex][chan].control, 0, 128);
        gMidiState[srcIndex][chan].bend = 0;
    }

    if (gMidiDebug) {
        post("sapf~ midi cc: src=%d ch=%d cc=%d val=%d", srcIndex, chan+1, cc, val);
    }
}

// midi bend <srcIndex> <channel> <value>
void sapf_midi_bend(t_sapf* x, long argc, t_atom* argv)
{
    if (argc < 3) {
        object_error((t_object*)x, "midi bend: requires srcIndex channel value");
        return;
    }

    int srcIndex = CLAMP((int)atom_getlong(argv), 0, 15);
    int chan = CLAMP((int)atom_getlong(argv + 1) - 1, 0, 15);
    int val = CLAMP((int)atom_getlong(argv + 2), 0, 16383);

    gMidiState[srcIndex][chan].bend = val - 8192;  // Convert to signed

    if (gMidiDebug) {
        post("sapf~ midi bend: src=%d ch=%d val=%d", srcIndex, chan+1, val);
    }
}

// midi touch <srcIndex> <channel> <value>
void sapf_midi_touch(t_sapf* x, long argc, t_atom* argv)
{
    if (argc < 3) {
        object_error((t_object*)x, "midi touch: requires srcIndex channel value");
        return;
    }

    int srcIndex = CLAMP((int)atom_getlong(argv), 0, 15);
    int chan = CLAMP((int)atom_getlong(argv + 1) - 1, 0, 15);
    int val = CLAMP((int)atom_getlong(argv + 2), 0, 127);

    gMidiState[srcIndex][chan].touch = val;

    if (gMidiDebug) {
        post("sapf~ midi touch: src=%d ch=%d val=%d", srcIndex, chan+1, val);
    }
}

// midi polytouch <srcIndex> <channel> <pitch> <value>
void sapf_midi_polytouch(t_sapf* x, long argc, t_atom* argv)
{
    if (argc < 4) {
        object_error((t_object*)x, "midi polytouch: requires srcIndex channel pitch value");
        return;
    }

    int srcIndex = CLAMP((int)atom_getlong(argv), 0, 15);
    int chan = CLAMP((int)atom_getlong(argv + 1) - 1, 0, 15);
    int pitch = CLAMP((int)atom_getlong(argv + 2), 0, 127);
    int val = CLAMP((int)atom_getlong(argv + 3), 0, 127);

    gMidiState[srcIndex][chan].polytouch[pitch] = val;

    if (gMidiDebug) {
        post("sapf~ midi polytouch: src=%d ch=%d pitch=%d val=%d", srcIndex, chan+1, pitch, val);
    }
}

// midi program <srcIndex> <channel> <program>
void sapf_midi_program(t_sapf* x, long argc, t_atom* argv)
{
    if (argc < 3) {
        object_error((t_object*)x, "midi program: requires srcIndex channel program");
        return;
    }

    int srcIndex = CLAMP((int)atom_getlong(argv), 0, 15);
    int chan = CLAMP((int)atom_getlong(argv + 1) - 1, 0, 15);
    int prog = CLAMP((int)atom_getlong(argv + 2), 0, 127);

    gMidiState[srcIndex][chan].program = prog;

    if (gMidiDebug) {
        post("sapf~ midi program: src=%d ch=%d prog=%d", srcIndex, chan+1, prog);
    }
}

// midi raw <srcIndex> <byte1> [byte2] [byte3]
void sapf_midi_raw(t_sapf* x, long argc, t_atom* argv)
{
    if (argc < 2) {
        object_error((t_object*)x, "midi raw: requires srcIndex and at least one byte");
        return;
    }

    int srcIndex = CLAMP((int)atom_getlong(argv), 0, 15);

    uint8_t data[3];
    int length = 0;
    for (int i = 1; i < argc && i < 4; i++) {
        data[length++] = (uint8_t)atom_getlong(argv + i);
    }

    MidiRouter::instance().handleIncomingMessage(srcIndex, data, length);
}
```

### 3. Registration

```cpp
// In ext_main()
class_addmethod(c, (method)sapf_midi, "midi", A_GIMME, 0);
```

### 4. Include Headers

```cpp
// In sapf_tilde.cpp
#include "MidiRouter.hpp"
```

---

## Max Patching Examples

### Basic Note Input

```
[notein]
|    |    |
[pack 0 0 0 0]  // srcIndex, channel, pitch, velocity
|
[prepend midi note]
|
[sapf~]
```

In SAPF:
```
0 1 60 0 127 mgate play    // Gate signal for middle C on src 0, channel 1
0 1 0 127 mlastvel play    // Velocity of last note
0 1 mlastkey 69 - 12 / 2 ** 440 * 0 sinosc play  // Play last note as pitch
```

### CC Modulation

```
[ctlin 1]               // CC #1 (mod wheel)
|        |
[pack 0 1 1 0]          // srcIndex 0, channel 1, CC 1, value
|
[prepend midi cc]
|
[sapf~]
```

In SAPF:
```
0 1 1 200 2000 mctl play   // Map CC1 to filter freq 200-2000Hz smoothed signal
```

### Pitch Bend

```
[bendin]
|       |
[pack 0 0 0]            // srcIndex 0, channel, bend value (0-16383)
|
[prepend midi bend]
|
[sapf~]
```

In SAPF:
```
0 1 -2 2 mbend play        // Pitch bend mapped to -2 to +2 semitones
```

### Full MIDI Keyboard Setup

```
                    [midiinfo]
                         |
                    [umenu]
                         |
[loadbang]---[0]    [midiparse]
             |       /  |  |  \
         [midiin]  [pack...]  [pack...]
                      |          |
                 [prepend midi note] [prepend midi cc]
                      \          /
                       \        /
                        [sapf~]
```

---

## Alternative Approach: Enable Core MIDI Directly

Instead of bridging Max messages, enable SAPF's native Core MIDI:

### Implementation

```cpp
void sapf_midi_native(t_sapf* x, t_symbol* s, long argc, t_atom* argv)
{
    if (argc < 1) return;

    t_symbol* cmd = atom_getsym(argv);

    if (cmd == gensym("start")) {
        EnsureDefaultMidiBackend();
        GetMidiBackend().initialize(16, 16);
        post("sapf~: Native MIDI started");
    }
    else if (cmd == gensym("stop")) {
        if (HasMidiBackend()) {
            GetMidiBackend().cleanup();
        }
        post("sapf~: Native MIDI stopped");
    }
    else if (cmd == gensym("list")) {
        EnsureDefaultMidiBackend();
        GetMidiBackend().listDevices();
    }
    else if (cmd == gensym("connect")) {
        // midi connect <uid> <portIndex>
        if (argc < 3) return;
        int uid = atom_getlong(argv + 1);
        int port = atom_getlong(argv + 2);
        EnsureDefaultMidiBackend();
        GetMidiBackend().connectInput(uid, port);
    }
    else if (cmd == gensym("disconnect")) {
        // midi disconnect <uid> <portIndex>
        if (argc < 3) return;
        int uid = atom_getlong(argv + 1);
        int port = atom_getlong(argv + 2);
        if (HasMidiBackend()) {
            GetMidiBackend().disconnectInput(uid, port);
        }
    }
}
```

### Pros
- No Max patching required for simple setups
- Direct access to all MIDI devices
- Lower latency

### Cons
- Bypasses Max's MIDI infrastructure
- Device selection less flexible
- May conflict with other Max MIDI objects
- Users unfamiliar with SAPF's device UID system

---

## Recommendation

**Implement the Max MIDI Bridge (primary approach)** with optional native MIDI as a secondary mode.

```cpp
// Max bridge (recommended)
midi note 0 1 60 127        // Use Max's [notein] etc.
midi cc 0 1 1 64

// Native Core MIDI (optional, for advanced users)
midi native start
midi native list
midi native connect <uid> <port>
```

This provides:
1. **Familiar workflow** for Max users
2. **Flexible routing** through Max objects
3. **Optional direct access** for low-latency needs
4. **Full SAPF MIDI op compatibility**

---

## Implementation Phases

### Phase 1: Max MIDI Bridge
- Implement `sapf_midi()` message handler
- Individual handlers for note, cc, bend, touch, polytouch, program
- Raw byte handler for advanced use
- Debug output support

### Phase 2: Documentation
- Update README with MIDI section
- Add MIDI examples to help patcher
- Document all SAPF MIDI ops with Max patching examples

### Phase 3: Native MIDI (Optional)
- Add `midi native` subcommand
- Expose CoreMidiBackend methods
- Device listing and connection via messages

### Phase 4: MIDI Output (Future)
- Implement `midi send` for MIDI output
- Enable SAPF's MIDI output ops
- Two-way MIDI communication

---

## Usage Examples in SAPF

### Velocity-Sensitive Synth

```
// srcIndex 0, channel 1
0 1 mlastkey 69 - 12 / 2 ** 440 *   // Frequency from last key
0 sinosc                             // Oscillator
0 1 0 1 mlastvel *                   // Scale by velocity
0 1 60 mgate *                       // Gate by middle C
play
```

### CC-Controlled Filter

```
0 adc                               // Audio input
0 1 74 100 5000 mctl               // CC74 (filter cutoff) mapped to 100-5000Hz
lpf play
```

### Pitch Bend + Modulation

```
// Base frequency with pitch bend
440 0 1 0.5 2 mbend * *            // Bend range: +/- 1 octave

// Apply vibrato from mod wheel
0 1 1 0 10 mctl                    // CC1 -> 0-10Hz vibrato depth
0 sinosc 0.02 * 1 +                // LFO
*                                   // Apply to frequency

0 sinosc play
```

### MPE-Style Polyphony

```
// 4-voice MPE on channels 2-5
[2 3 4 5] { =ch
    0 ch mlastkey 69 - 12 / 2 ** 440 *
    0 ch -1 1 mbend 12 / 2 ** *    // Per-note bend
    0 sinosc
    0 ch 0 1 mlastvel *
    0 ch mlastkey mgate *
} map
mix play
```

---

## Open Questions

1. **Thread safety**: MIDI messages arrive asynchronously. Should we use mutex protection for `gMidiState` access?
   - Currently SAPF reads are atomic (single byte/word)
   - May need protection for multi-field updates

2. **Latency**: Max message scheduling adds latency. Is this acceptable?
   - Typical: 1-3ms additional
   - For timing-critical use, recommend native MIDI

3. **Channel convention**: SAPF uses 1-16, Max uses 1-16. Maintain consistency.

4. **Source index mapping**: How should users think about srcIndex?
   - Simple: Always use 0 unless multiplexing sources
   - Advanced: Use different srcIndex for different physical inputs

5. **MIDI output**: Should this be Phase 3 or Phase 4?
   - Input is more immediately useful
   - Output requires additional Max objects for routing

---

## Summary

| Feature | Approach | Complexity | Priority |
|---------|----------|------------|----------|
| MIDI input bridge | Message handlers | Low | High |
| Debug output | `midi debug` | Low | High |
| Raw bytes | `midi raw` | Low | Medium |
| Native MIDI | CoreMidiBackend | Medium | Low |
| MIDI output | Future | High | Low |

This proposal enables all of SAPF's MIDI functionality within Max's ecosystem while maintaining flexibility for advanced users who need direct Core MIDI access.
