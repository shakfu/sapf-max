#include "ErrorCodes.hpp"
#include "Manta.h"
#include "VM.hpp"
#include "Play.hpp"
#include "primes.hpp"
#include <CoreFoundation/CoreFoundation.h>
#include <algorithm>
#include <complex>
#include <dispatch/dispatch.h>
#include <histedit.h>
#include <os/lock.h>
#include <stdio.h>
#include <sys/stat.h>
#include <vector>

#define CODE_BUFFER_SIZE 4096

// enums for inlets / outlets
enum INLETS { I_INPUT, NUM_INLETS };
enum OUTLETS { O_OUTPUT, NUM_OUTLETS };

// Forward declarations for sapf builtin initialization functions
extern void AddCoreOps();
extern void AddMathOps();
extern void AddStreamOps();
extern void AddUGenOps();
extern void AddSetOps();
extern void AddRandomOps();
extern void AddMidiOps();

#include "ext.h"           // standard Max include, always required (except in Jitter)
#include "ext_atomic.h"    // Max's atomic operations for lock-free threading
#include "ext_obex.h"      // required for "new" style objects
#include "ext_systhread.h" // Max's thread-safe primitives
#include "z_dsp.h"         // required for MSP objects


// Threading Model:
// - sapfThread: Used in main thread for compilation and execution of sapf code
// - audioThread: Used in audio callback (sapf_perform64) for thread-safe audio
// generation
// - audioStateLock: Protects shared audio state between main and audio threads
// - ZIn audioExtractor: Copied to audio thread to avoid cross-thread access

// struct to represent the object's state
typedef struct _sapf {
    t_pxobject ob; // the object itself (Max MSP object)

    // Legacy field (can be removed later)
    double offset; // the value of a property of our object

    // Sapf execution context
    Thread* sapfThread;  // Main sapf execution thread (used in main thread for
                         // compilation/execution)
    Thread* audioThread; // Separate thread for audio processing (used in audio
                         // callback)

    // Compiled sapf code cache
    P<Fun> compiledFunction; // Currently compiled sapf function
    char* lastSapfCode;      // Last compiled sapf code string for change detection

    // Audio extraction interface (lock-free access via atomics)
    ZIn audioExtractor;     // Interface for extracting audio from sapf results
                            // (single channel)
    ZIn audioExtractors[8]; // Multi-channel audio extractors (up to 8
                            // channels)

    // Atomic state variables for lock-free audio thread access
    t_int32_atomic numAudioChannels;  // Number of active audio channels
    t_int32_atomic hasValidAudio;     // Flag indicating if audioExtractor contains
                                      // valid audio data (0/1)
    t_int32_atomic audioStateVersion; // Version counter for audio state changes

    // Error handling and status
    bool compilationError;  // True if last compilation failed
    char errorMessage[256]; // Last error message for debugging

    // Sample rate synchronization
    double currentSampleRate; // Current sample rate from Max
    bool sampleRateChanged;   // Flag to trigger VM reconfiguration

    // Intermediate audio buffers (like ChucK example)
    float* out_sapf_buffer;     // Intermediate sapf output buffer
    long bufferSize;           // Current buffer size (for reallocation detection)

    // non-audio outlet
    void * text_outlet;
} t_sapf;

// method prototypes
void* sapf_new(t_symbol* s, long argc, t_atom* argv);
void sapf_free(t_sapf* x);
void sapf_assist(t_sapf* x, void* b, long m, long a, char* s);
void sapf_float(t_sapf* x, double f);
void sapf_dsp64(t_sapf* x, t_object* dsp64, short* count, double samplerate, long maxvectorsize, long flags);
void sapf_perform64(t_sapf* x, t_object* dsp64, double** ins, long numins, double** outs, long numouts,
                    long sampleframes, long flags, void* userparam);
void sapf_code(t_sapf* x, t_symbol* s, long argc, t_atom* argv);
void sapf_status(t_sapf* x);
void sapf_help(t_sapf* x);
void sapf_stack(t_sapf* x);
void sapf_clear(t_sapf* x);

// Max-specific audio functions
void addMaxSpecificOps();
void outputStackToTextOutlet(t_sapf* x);
void sapf_fill(t_sapf* x, long numFrames, float* outBuffer);

// possibly useful funcs
void initSapfBuiltins();
void reportSapfError(t_sapf* x, const char* codeBuffer, const std::exception& e);

// global class pointer variable
static t_class* sapf_class = NULL;

// Global reference to current sapf object for Max audio integration
static t_sapf* gCurrentSapfObject = nullptr;

// Flag to track if sapf builtins have been initialized globally
static bool gSapfBuiltinsInitialized = false;

// Initialize all sapf built-in functions
void initSapfBuiltins()
{
    if (gSapfBuiltinsInitialized)
        return; // Already initialized

    post("sapf~: Initializing sapf built-in functions...");

    try {
        AddCoreOps();
        AddMathOps();
        AddStreamOps();
        AddRandomOps();
        AddUGenOps(); // This includes AddOscilUGenOps() which has sinosc
        AddMidiOps();
        AddSetOps();

        // Add Max-specific overrides after standard functions
        addMaxSpecificOps();

        gSapfBuiltinsInitialized = true;
        post("sapf~: Built-in functions initialized successfully");

    } catch (const std::exception& e) {
        error("sapf~: Error initializing built-ins: %s", e.what());
    } catch (...) {
        error("sapf~: Unknown error initializing built-ins");
    }
}


// Max-specific play primitive that uses Max audio instead of AudioUnit
static void playMax_(Thread& th, Prim* prim)
{
    post("sapf~: DEBUG - playMax_ called (Max audio route)");

    if (!gCurrentSapfObject) {
        error("sapf~: No current sapf object for audio playback");
        return;
    }

    V v = th.popList("play : list");

    // Directly capture audio generator without calling AudioUnit code
    try {
        t_sapf* x = gCurrentSapfObject;

        if (v.isList()) {
            if (v.isZList()) {
                // Single channel
                x->audioExtractor.set(v);
                x->numAudioChannels = 1;
                x->hasValidAudio = 1;
                post("sapf~: ✓ Single-channel audio generator captured for Max");
            } else {
                // Multi-channel - use the first channel for now
                if (!v.isFinite()) {
                    post("sapf~: Error: Infinite lists not supported for multi-channel audio");
                    return;
                }

                P<List> s = (List*)v.o();
                s = s->pack(th, 32); // Max 32 channels
                if (!s()) {
                    post("sapf~: Error: Too many channels");
                    return;
                }

                Array* a = s->mArray();
                int numChannels = std::min((int)a->size(), 32);

                // For now, just use the first channel
                if (numChannels > 0) {
                    x->audioExtractor.set(a->at(0));
                    x->numAudioChannels = 1; // TODO: Implement multi-channel properly
                    x->hasValidAudio = 1;
                    post("sapf~: ✓ Multi-channel audio generator captured (using channel 0)");
                }
            }
        } else {
            post("sapf~: Error: play requires a list argument");
        }

    } catch (const std::exception& e) {
        error("sapf~: Error in playMax_: %s", e.what());
        if (gCurrentSapfObject) {
            gCurrentSapfObject->hasValidAudio = 0;
        }
    } catch (...) {
        error("sapf~: Unknown error in playMax_");
        if (gCurrentSapfObject) {
            gCurrentSapfObject->hasValidAudio = 0;
        }
    }
}

// Max-specific stop primitive that stops Max audio instead of AudioUnit
static void stopMax_(Thread& th, Prim* prim)
{
    post("sapf~: DEBUG - stopMax_ called (Max audio route)");

    if (gCurrentSapfObject) {
        gCurrentSapfObject->hasValidAudio = 0;
        gCurrentSapfObject->numAudioChannels = 0;
        post("sapf~: ✓ Max audio generation stopped");
    } else {
        post("sapf~: Warning - No current sapf object to stop");
    }
}

// Add Max-specific primitives that override standard ones
void addMaxSpecificOps()
{
    post("sapf~: Adding Max-specific primitives (overriding 'play' and 'stop')");

    try {
        // Override the 'play' primitive to use Max audio instead of AudioUnit
        // This must be called after AddStreamOps() to override the standard play
        vm.def("play", 1, 0, playMax_, "(channels -->) plays the audio to Max outputs.");

        // Override the 'stop' primitive to stop Max audio instead of AudioUnit
        vm.def("stop", 0, 0, stopMax_, "() stops audio playback.");

        post("sapf~: ✓ Max-specific 'play' and 'stop' primitives installed");

    } catch (const std::exception& e) {
        error("sapf~: Error adding Max-specific primitives: %s", e.what());
    } catch (...) {
        error("sapf~: Unknown error adding Max-specific primitives");
    }
}

// Output current stack contents to Max text outlet (like sapf REPL)
void outputStackToTextOutlet(t_sapf* x)
{
    if (!x || !x->sapfThread || !x->text_outlet) {
        return;
    }

    try {
        size_t stackDepth = x->sapfThread->stackDepth();

        if (stackDepth == 0) {
            // Output empty stack indicator (like REPL prompt)
            t_atom emptyAtom;
            atom_setsym(&emptyAtom, gensym("stack_empty"));
            outlet_anything(x->text_outlet, gensym("stack"), 1, &emptyAtom);
        } else {
            // Output each stack item
            for (size_t i = 0; i < stackDepth; i++) {
                try {
                    // Get stack item (from bottom to top to match REPL order)
                    V stackItem = x->sapfThread->stack[x->sapfThread->stackBase + i];

                    // Convert sapf value to Max-compatible output
                    if (stackItem.isReal()) {
                        t_atom valueAtom;
                        atom_setfloat(&valueAtom, (float)stackItem.asFloat());
                        outlet_anything(x->text_outlet, gensym("value"), 1, &valueAtom);

                    } else if (stackItem.isList()) {
                        // For lists, output a formatted representation
                        P<List> list = (List*)stackItem.o();
                        if (list && list->isFinite()) {
                            Array* arr = list->mArray.get();
                            if (arr && arr->size() <= 10) { // Limit output size
                                t_atom listAtoms[10];
                                int atomCount = 0;

                                for (size_t j = 0; j < (size_t)arr->size() && atomCount < 10; j++) {
                                    V item = arr->at(j);
                                    if (item.isReal()) {
                                        atom_setfloat(&listAtoms[atomCount], (float)item.asFloat());
                                        atomCount++;
                                    }
                                }

                                if (atomCount > 0) {
                                    outlet_anything(x->text_outlet, gensym("list"), atomCount, listAtoms);
                                } else {
                                    t_atom listSymbol;
                                    atom_setsym(&listSymbol, gensym("[complex_list]"));
                                    outlet_anything(x->text_outlet, gensym("value"), 1, &listSymbol);
                                }
                            } else {
                                t_atom listSymbol;
                                atom_setsym(&listSymbol, gensym("[large_list]"));
                                outlet_anything(x->text_outlet, gensym("value"), 1, &listSymbol);
                            }
                        } else {
                            t_atom listSymbol;
                            atom_setsym(&listSymbol, gensym("[infinite_list]"));
                            outlet_anything(x->text_outlet, gensym("value"), 1, &listSymbol);
                        }

                    } else if (stackItem.isObject()) {
                        // For other objects, output their type
                        Object* obj = stackItem.o();
                        if (obj) {
                            const char* typeName = obj->TypeName();
                            if (typeName) {
                                t_atom typeAtom;
                                atom_setsym(&typeAtom, gensym(typeName));
                                outlet_anything(x->text_outlet, gensym("object"), 1, &typeAtom);
                            } else {
                                t_atom unknownAtom;
                                atom_setsym(&unknownAtom, gensym("[unknown_object]"));
                                outlet_anything(x->text_outlet, gensym("value"), 1, &unknownAtom);
                            }
                        }
                    }

                } catch (const std::exception& e) {
                    post("sapf~: Error outputting stack item %zu: %s", i, e.what());
                    t_atom errorAtom;
                    atom_setsym(&errorAtom, gensym("[error]"));
                    outlet_anything(x->text_outlet, gensym("value"), 1, &errorAtom);
                }
            }
        }

    } catch (const std::exception& e) {
        post("sapf~: Error outputting stack contents: %s", e.what());
    }
}

// Fill audio buffer using sapf audio generators (like chuck->run())
void sapf_fill(t_sapf* x, long numFrames, float* outBuffer)
{
    if (!x || !outBuffer || numFrames <= 0) {
        return;
    }

    // Initialize buffer with silence
    memset(outBuffer, 0.f, sizeof(float) * numFrames);

    // Only generate audio if we have valid audio generators
    if (!x->hasValidAudio || !x->audioThread) {
        return; // Buffer remains silent
    }

    try {
        int localChannels = x->numAudioChannels;

        if (localChannels >= 1) {
            if (localChannels == 1) {
                // Single channel audio generation
                int frameCount = (int)numFrames;

                // Use ZIn::fill to generate audio directly into our buffer
                // This is the key - we use our own buffer instead of AudioUnit
                bool isDone = x->audioExtractor.fill(*x->audioThread, frameCount, outBuffer, 1);

                if (frameCount != (int)numFrames) {
                    // Fill remaining frames with silence if generator produced fewer frames
                    for (long i = frameCount; i < numFrames; i++) {
                        outBuffer[i] = 0.0f;
                    }
                }

                // If generator is done, mark audio as invalid
                if (isDone) {
                    x->hasValidAudio = 0;
                    post("sapf~: Audio generator completed");
                }

            } else {
                // Multi-channel: for now, just use first channel
                // TODO: Implement proper multi-channel support with separate buffers
                int frameCount = (int)numFrames;
                bool isDone = x->audioExtractors[0].fill(*x->audioThread, frameCount, outBuffer, 1);

                if (frameCount != (int)numFrames) {
                    for (long i = frameCount; i < numFrames; i++) {
                        outBuffer[i] = 0.0f;
                    }
                }

                if (isDone) {
                    x->hasValidAudio = 0;
                    post("sapf~: Multi-channel audio generator completed");
                }
            }
        }

    } catch (const std::exception& e) {
        post("sapf~: Error generating audio: %s", e.what());
        // Fill buffer with silence on error
        memset(outBuffer, 0.f, sizeof(float) * numFrames);
        x->hasValidAudio = 0;
    }
}

// Forward declarations for refactored sapf_code functions
struct ValidationResult {
    bool success;
    std::string codeBuffer;
    std::string errorMessage;
};

struct CompilationResult {
    bool success;
    P<Fun> compiledFunction;
    std::string errorMessage;
};

struct ExecutionResult {
    bool success;
    V audioResult;
    size_t stackDepth;
    std::string errorMessage;
};

struct AudioProcessingResult {
    bool success;
    bool hasValidAudio;
    int numChannels;
    std::string errorMessage;
};

ValidationResult sapf_validateInput(t_sapf* x, t_symbol* s, long argc, t_atom* argv);
CompilationResult sapf_compileCode(t_sapf* x, const std::string& codeBuffer, bool needsRecompilation);
ExecutionResult sapf_executeCode(t_sapf* x, P<Fun> compiledFunction);
AudioProcessingResult sapf_processAudioResult(t_sapf* x, const V& audioResult);
AudioProcessingResult sapf_handleMultiChannelAudio(t_sapf* x, const V& audioResult, const char* resultType);
void sapf_reportStatus(t_sapf* x, bool compilationError, bool hasValidAudio, P<Fun> compiledFunction);

// Enhanced error reporting with specific error type detection and user
// guidance
void reportSapfError(t_sapf* x, const char* codeBuffer, const std::exception& e)
{
    const char* errorMsg = e.what();

    // Check for specific error types and provide contextual guidance
    if (strstr(errorMsg, "Undefined")) {
        error("sapf~: ✗ Undefined symbol in: \"%s\"", codeBuffer);
        post("sapf~: Error: %s", errorMsg);
        post("sapf~: Hint: Check function names - available: sinosc, play, +, "
             "-, *, /, etc.");
        post("sapf~: Try: '440 0 sinosc 0.3 *' or send 'status' for VM info");

    } else if (strstr(errorMsg, "stack underflow") || strstr(errorMsg, "Stack underflow")) {
        error("sapf~: ✗ Stack underflow in: \"%s\"", codeBuffer);
        post("sapf~: Error: %s", errorMsg);
        post("sapf~: Hint: Not enough arguments for operation");
        post("sapf~: Example: '440 sinosc' needs frequency argument first");

    } else if (strstr(errorMsg, "stack overflow") || strstr(errorMsg, "Stack overflow")) {
        error("sapf~: ✗ Stack overflow in: \"%s\"", codeBuffer);
        post("sapf~: Error: %s", errorMsg);
        post("sapf~: Hint: Too many values on stack - simplify expression");

    } else if (strstr(errorMsg, "syntax") || strstr(errorMsg, "Syntax")) {
        error("sapf~: ✗ Syntax error in: \"%s\"", codeBuffer);
        post("sapf~: Error: %s", errorMsg);
        post("sapf~: Hint: Check parentheses, quotes, and operators");
        post("sapf~: Valid: '440 0 sinosc' Invalid: '440 sinosc('");

    } else if (strstr(errorMsg, "type") || strstr(errorMsg, "Type")) {
        error("sapf~: ✗ Type error in: \"%s\"", codeBuffer);
        post("sapf~: Error: %s", errorMsg);
        post("sapf~: Hint: Wrong argument type - check number vs audio vs "
             "array");

    } else if (strstr(errorMsg, "range") || strstr(errorMsg, "Range")) {
        error("sapf~: ✗ Range error in: \"%s\"", codeBuffer);
        post("sapf~: Error: %s", errorMsg);
        post("sapf~: Hint: Value out of valid range - check array indices, "
             "frequencies");

    } else if (strstr(errorMsg, "memory") || strstr(errorMsg, "alloc")) {
        error("sapf~: ✗ Memory error in: \"%s\"", codeBuffer);
        post("sapf~: Error: %s", errorMsg);
        post("sapf~: Hint: Out of memory - try simpler code or restart Max");

    } else {
        // Generic error with basic guidance
        error("sapf~: ✗ Compilation error: %s", errorMsg);
        post("sapf~: Code: \"%s\"", codeBuffer);
        post("sapf~: Hint: Try simpler expressions like '440 sinosc' or send "
             "'status'");
    }

    // Always provide general help
    post("sapf~: For help: send 'status' for VM info, or try basic examples:");
    post("sapf~: '440 0 sinosc' (sine wave) or '220 330 + 0 sinosc' (math)");
}

//***********************************************************************************************

void ext_main(void* r)
{
    // object initialization, note the use of dsp_free for the freemethod,
    // which is required unless you need to free allocated memory, in which
    // case you should call dsp_free from your custom free function.

    t_class* c = class_new("sapf~", (method)sapf_new, (method)sapf_free, (long)sizeof(t_sapf), 0L, A_GIMME, 0);

    class_addmethod(c, (method)sapf_float, "float", A_FLOAT, 0);
    class_addmethod(c, (method)sapf_dsp64, "dsp64", A_CANT, 0);
    class_addmethod(c, (method)sapf_assist, "assist", A_CANT, 0);
    class_addmethod(c, (method)sapf_code, "code", A_GIMME, 0);
    class_addmethod(c, (method)sapf_status, "status", 0);
    class_addmethod(c, (method)sapf_help, "help", 0);
    class_addmethod(c, (method)sapf_stack, "stack", 0);
    class_addmethod(c, (method)sapf_clear, "clear", 0);

    class_dspinit(c);
    class_register(CLASS_BOX, c);
    sapf_class = c;
}

void* sapf_new(t_symbol* s, long argc, t_atom* argv)
{
    t_sapf* x = (t_sapf*)object_alloc(sapf_class);

    if (x) {
        // MSP inlets: arg is # of inlets and is REQUIRED!
        dsp_setup((t_pxobject*)x, 1);
        
        // general (non-audio) outlet
        x->text_outlet = outlet_new((t_object *)x, NULL);

        // audio (signal) outlet
        outlet_new(x, "signal"); // signal outlet (note "signal" rather than NULL)

        // Legacy field initialization
        x->offset = 0.0;

        // Initialize sapf VM components
        try {
            // Initialize sapf built-in functions (only once globally)
            initSapfBuiltins();

            // Create sapf execution thread for main thread
            // (compilation/execution)
            x->sapfThread = new Thread();

            // Create separate thread for audio processing (thread-safe audio
            // generation)
            x->audioThread = new Thread();

            // Load prelude file from its known location in the project
            const char* preludePath = "sapf-prelude.txt";
            try {
                post("sapf~: Loading prelude file: %s", preludePath);
                loadFile(*x->sapfThread, preludePath);
                post("sapf~: Prelude loaded successfully");
            } catch (const std::exception& e) {
                post("sapf~: Warning - Error loading prelude from %s: %s", preludePath, e.what());
                post("sapf~: Continuing without prelude (some functions may not be available)");
            } catch (...) {
                post("sapf~: Warning - Unknown error loading prelude from %s", preludePath);
                post("sapf~: Continuing without prelude (some functions may not be available)");
            }

            // Initialize compiled function storage
            x->compiledFunction = P<Fun>(); // Initialize empty smart pointer
            x->lastSapfCode = nullptr;      // No cached code yet

            // Initialize audio extraction interface
            x->audioExtractor = ZIn(); // Initialize empty ZIn (legacy single
                                       // channel)
            for (int i = 0; i < 8; i++) {
                x->audioExtractors[i] = ZIn(); // Initialize multi-channel
                                               // extractors
            }

            // Initialize atomic variables for lock-free thread communication
            x->numAudioChannels = 0;  // No channels active initially
            x->hasValidAudio = 0;     // False (atomic: 0 = false, 1 = true)
            x->audioStateVersion = 0; // Start with version 0

            // Initialize error handling
            x->compilationError = false;
            x->errorMessage[0] = '\0'; // Empty error message

            // Initialize sample rate tracking
            x->currentSampleRate = sys_getsr(); // Default until sapf_dsp64
                                                // sets it
            x->sampleRateChanged = true;        // Force initial configuration

            // Initialize intermediate audio buffers
            x->out_sapf_buffer = nullptr;
            x->bufferSize = 0;

            // Set global reference for Max audio integration
            gCurrentSapfObject = x;

            post("sapf~: Initialized with sapf language interpreter");

        } catch (const std::exception& e) {
            post("sapf~: Error initializing sapf VM: %s", e.what());

            // Cleanup on failure
            if (x->sapfThread) {
                delete x->sapfThread;
                x->sapfThread = nullptr;
            }
            if (x->audioThread) {
                delete x->audioThread;
                x->audioThread = nullptr;
            }
            if (x->lastSapfCode) {
                free(x->lastSapfCode);
                x->lastSapfCode = nullptr;
            }

            // Mark as failed
            x->compilationError = true;
            strncpy_zero(x->errorMessage, "VM initialization failed", sizeof(x->errorMessage) - 1);
            x->errorMessage[sizeof(x->errorMessage) - 1] = '\0';
        }
    }
    return (x);
}

void sapf_free(t_sapf* x)
{
    if (!x)
        return; // Safety check

    post("sapf~: Cleaning up sapf VM resources");

    // Clear global reference if this object was the current one
    if (gCurrentSapfObject == x) {
        gCurrentSapfObject = nullptr;
    }

    // Clean up main sapf Thread (manually allocated)
    if (x->sapfThread) {
        try {
            delete x->sapfThread;
            x->sapfThread = nullptr;
        } catch (const std::exception& e) {
            post("sapf~: Error cleaning up main Thread: %s", e.what());
        }
    }

    // Clean up audio sapf Thread (manually allocated)
    if (x->audioThread) {
        try {
            delete x->audioThread;
            x->audioThread = nullptr;
        } catch (const std::exception& e) {
            post("sapf~: Error cleaning up audio Thread: %s", e.what());
        }
    }

    // Clean up cached sapf code string (manually allocated)
    if (x->lastSapfCode) {
        free(x->lastSapfCode);
        x->lastSapfCode = nullptr;
    }

    // Clean up intermediate audio buffers
    if (x->out_sapf_buffer) {
        delete[] x->out_sapf_buffer;
        x->out_sapf_buffer = nullptr;
    }

    // Smart pointers (P<Fun>) clean up automatically via destructor
    // ZIn objects clean up automatically via destructor
    // Primitive types (bool, double, char[]) clean up automatically

    // must call dsp_free here
    dsp_free((t_pxobject*)x);

    post("sapf~: Cleanup complete");
}

void sapf_code(t_sapf* x, t_symbol* s, long argc, t_atom* argv)
{
    post("sapf~: DEBUG - sapf_code entry, argc=%ld", argc);

    // Set global object reference for Max audio integration
    gCurrentSapfObject = x;

    // Phase 1: Input validation and code string construction
    ValidationResult validation = sapf_validateInput(x, s, argc, argv);
    if (!validation.success) {
        error("sapf~: FATAL - %s", validation.errorMessage.c_str());
        return;
    }

    post("sapf~: DEBUG - x pointer valid (%p)", x);
    post("sapf~: DEBUG - sapfThread valid (%p)", x->sapfThread);
    post("sapf~: DEBUG - audioThread valid (%p)", x->audioThread);
    post("sapf~: DEBUG - All critical pointers validated successfully");

    std::string codeBuffer = validation.codeBuffer;

    // Check if code has changed (for caching)
    bool needsRecompilation = true;
    if (x->lastSapfCode) {
        needsRecompilation = (codeBuffer != std::string(x->lastSapfCode));
    }

    // Phase 2: Compilation and caching logic
    CompilationResult compilation = sapf_compileCode(x, codeBuffer, needsRecompilation);
    if (!compilation.success) {
        if (!compilation.errorMessage.empty()) {
            error("sapf~: ✗ Compilation error: %s", compilation.errorMessage.c_str());
        }
        return;
    }

    // Check if this code contains 'play' command
    bool containsPlay = (codeBuffer.find("play") != std::string::npos);

    // Phase 3: Function execution and stack management (only for new compilations)
    if (needsRecompilation && compilation.compiledFunction) {
        ExecutionResult execution = sapf_executeCode(x, compilation.compiledFunction);
        if (!execution.success) {
            if (!execution.errorMessage.empty()) {
                error("sapf~: ✗ %s", execution.errorMessage.c_str());
            }
        } else {
            // Successful execution - handle results based on whether this was a 'play' command
            if (containsPlay) {
                post("sapf~: ✓ Audio command executed - audio routed to Max output");
                // Don't output stack contents for 'play' commands - they consume the stack
            } else {
                // Non-play command: output stack contents to text outlet (like sapf REPL)
                post("sapf~: ✓ Stack expression executed - output to text outlet");
                outputStackToTextOutlet(x);
            }
        }
        // Note: Audio processing is now handled by the 'play' primitive itself
        // We don't need to process stack results for audio here anymore
    } else if (!containsPlay) {
        // Even if we didn't recompile, output stack for non-play commands
        outputStackToTextOutlet(x);
    }

    // Phase 6: Final status reporting
    sapf_reportStatus(x, x->compilationError, x->hasValidAudio, x->compiledFunction);
}

void sapf_status(t_sapf* x)
{
    // if (!x) {
    //     error("sapf~: Invalid object pointer");
    //     return;
    // }

    post("sapf~: === STATUS REPORT ===");

    // VM Initialization Status
    if (x->sapfThread && x->audioThread) {
        post("sapf~: VM: ✓ Both main and audio threads initialized and ready");
    } else if (x->sapfThread || x->audioThread) {
        post("sapf~: VM: ⚠ Partially initialized - main:%s audio:%s", x->sapfThread ? "OK" : "FAIL",
             x->audioThread ? "OK" : "FAIL");
    } else {
        post("sapf~: VM: ✗ Not initialized");
        return; // No point in checking other status if VM is not initialized
    }

    // Compilation Status
    if (x->compilationError) {
        post("sapf~: Compilation: ✗ ERROR - %s", x->errorMessage);
    } else if (x->compiledFunction) {
        post("sapf~: Compilation: ✓ Function compiled and loaded");
    } else {
        post("sapf~: Compilation: ○ No function compiled yet");
    }

    // Code Cache Status
    if (x->lastSapfCode) {
        post("sapf~: Last Code: \"%s\"", x->lastSapfCode);
    } else {
        post("sapf~: Last Code: (none)");
    }

    // Audio Status (thread-safe read)
    bool audioStatus;
    int audioChannels;
    // Lock-free atomic reads
    audioStatus = x->hasValidAudio;
    audioChannels = x->numAudioChannels;

    if (audioStatus) {
        if (audioChannels == 1) {
            post("sapf~: Audio: ✓ Single-channel ready for generation "
                 "(thread-safe)");
        } else if (audioChannels > 1) {
            post("sapf~: Audio: ✓ %d-channel ready for generation "
                 "(thread-safe)",
                 audioChannels);
        } else {
            post("sapf~: Audio: ✓ Ready but no channels configured "
                 "(thread-safe)");
        }
    } else {
        post("sapf~: Audio: ○ No audio data generated yet (thread-safe)");
    }

    // Sample Rate Status
    post("sapf~: Sample Rate: %.1f Hz %s", x->currentSampleRate,
         x->sampleRateChanged ? "(changed, needs VM update)" : "(synchronized)");

    // Stack Status
    if (x->sapfThread) {
        size_t stackDepth = x->sapfThread->stackDepth();
        if (stackDepth == 0) {
            post("sapf~: Stack: ✓ Empty (clean state)");
        } else {
            post("sapf~: Stack: ⚠ %zu items present", stackDepth);
            post("sapf~: Hint: Send 'stack' to inspect or 'clear' to empty");
        }
    }

    // Memory Status
    post("sapf~: Memory: Function=%s, CodeCache=%s", x->compiledFunction ? "allocated" : "null",
         x->lastSapfCode ? "cached" : "empty");

    post("sapf~: === END STATUS ===");
}


void sapf_assist(t_sapf* x, void* b, long io, long idx, char* s)
{
    /* Document inlet functions */
    if (io == ASSIST_INLET) {
        switch (idx) {
        case I_INPUT:
            snprintf_zero(s, ASSIST_MAX_STRING_LEN, "%ld: input", idx);
            break;
        }
    } 

    /* Document outlet functions */
    else if (io == ASSIST_OUTLET) {
        switch (idx) {
        case O_OUTPUT:
            snprintf_zero(s, ASSIST_MAX_STRING_LEN, "%ld: output", idx);
            break;
        }
    }
}


void sapf_float(t_sapf* x, double f) { x->offset = f; }

// registers a function for the signal chain in Max
void sapf_dsp64(t_sapf* x, t_object* dsp64, short* count, double samplerate, long maxvectorsize, long flags)
{
    if (!x) {
        error("sapf~: Invalid object in dsp64");
        return;
    }

    post("sapf~: Max sample rate: %.1f Hz, vector size: %ld", samplerate, maxvectorsize);

    // Check if sample rate has actually changed
    bool rateChanged = (x->currentSampleRate != samplerate);
    x->currentSampleRate = samplerate;

    if (rateChanged || x->sampleRateChanged) {
        try {
            // Configure global sapf VM with Max's sample rate
            vm.setSampleRate(samplerate);

            // Clear the sample rate changed flag
            x->sampleRateChanged = false;

            post("sapf~: ✓ Configured sapf VM with sample rate: %.1f Hz", samplerate);

            // Update Thread rate context if threads exist
            if (x->sapfThread && x->audioThread) {
                // Both threads automatically get the updated rate from vm.ar
                // when needed
                post("sapf~: ✓ Both main and audio threads will use updated "
                     "rate context");
            } else {
                post("sapf~: ⚠ Some threads not initialized - rate update may "
                     "not apply fully");
            }

        } catch (const std::exception& e) {
            error("sapf~: Error configuring VM sample rate: %s", e.what());
            x->sampleRateChanged = true; // Keep flag set to retry later

            // Mark VM as in error state for diagnostics
            if (x) {
                x->compilationError = true;
                snprintf_zero(x->errorMessage, sizeof(x->errorMessage), "VM sample rate config failed: %s", e.what());
            }
        } catch (...) {
            error("sapf~: Unknown error configuring VM sample rate");
            x->sampleRateChanged = true; // Keep flag set to retry later
        }
    } else {
        post("sapf~: Sample rate unchanged (%.1f Hz)", samplerate);
    }

    // Allocate intermediate audio buffers (like ChucK example)
    post("sapf~: Allocating audio buffers: %ld frames", maxvectorsize);

    // Clean up old buffers if they exist
    if (x->out_sapf_buffer) {
        delete[] x->out_sapf_buffer;
        x->out_sapf_buffer = nullptr;
    }

    // Allocate new buffers with current vector size
    // For now, supporting single channel - can expand to multichannel later
    x->out_sapf_buffer = new float[maxvectorsize];
    x->bufferSize = maxvectorsize;

    // Initialize buffer with silence
    memset(x->out_sapf_buffer, 0.f, sizeof(float) * maxvectorsize);

    post("sapf~: ✓ Audio buffers allocated (%ld samples)", maxvectorsize);

    object_method(dsp64, gensym("dsp_add64"), x, sapf_perform64, 0, NULL);
}

// this is the 64-bit perform method audio vectors (buffer-based like ChucK)
void sapf_perform64(t_sapf* x, t_object* dsp64, double** ins, long numins, double** outs, long numouts,
                    long sampleframes, long flags, void* userparam)
{
    t_double* outL = outs[0];
    float* out_ptr = x->out_sapf_buffer;
    long n = sampleframes; // n = 64

    if (!x || !x->out_sapf_buffer) {
        // No buffer allocated - output silence
        for (long chan = 0; chan < numouts; chan++) {
            for (long i = 0; i < n; i++) {
                outs[chan][i] = 0.0;
            }
        }
        return;
    }

    // Step 1: Generate audio using sapf (like chuck->run())
    sapf_fill(x, n, out_ptr);


    // Debug output (first few calls only)
    static int debugCallCount = 0;
    if (debugCallCount < 3) {
        bool hasNonZero = false;
        for (long i = 0; i < std::min(n, 8L); i++) {
            if (x->out_sapf_buffer[i] != 0.0f) {
                hasNonZero = true;
                break;
            }
        }
        post("sapf~: Audio callback: %ld frames, hasValidAudio=%s, buffer has audio=%s",
             n, x->hasValidAudio ? "true" : "false", hasNonZero ? "true" : "false");
        if (hasNonZero) {
            post("sapf~: Sample values: [0]=%.6f [1]=%.6f [2]=%.6f",
                 x->out_sapf_buffer[0], x->out_sapf_buffer[1], x->out_sapf_buffer[2]);
        }
        debugCallCount++;
    }

    // Step 2: Copy from our intermediate buffer to Max outputs (like ChucK)
    while (n--) {
        *outL++ = (double)(*out_ptr++);
    }

    // for (long i = 0; i < n; i++) {
    //     for (long chan = 0; chan < numouts; chan++) {
    //         outs[chan][i] = (double)(*out_ptr++);
    //     }
    //     // out_ptr++; // Move to next sample
    // }
}

void sapf_help(t_sapf* x)
{
    post("sapf~: === SAPF LANGUAGE HELP ===");
    post("");

    post("sapf~ is a Max external that embeds the sapf language interpreter");
    post("SAPF (Sound As Pure Form) is a functional, stack-based audio "
         "programming language");
    post("");

    post("Basic Usage:");
    post("  Send 'code <expression>' messages to compile and execute sapf "
         "code");
    post("  Example: [code 440 0 sinosc 0.3 *(");
    post("");

    post("Common Commands:");
    post("  status  - Show VM status and current state");
    post("  help    - Show this help message");
    post("  stack   - Inspect current sapf stack contents");
    post("  clear   - Clear sapf stack (removes all values)");
    post("  Note: Stack values are preserved after code execution for "
         "debugging");
    post("");

    post("Basic Examples:");
    post("  440 0 sinosc                - 440Hz sine wave");
    post("  440 0 sinosc 0.3 *          - Sine wave at 30% volume");
    post("  220 330 + 0 sinosc          - Math: (220+330)Hz sine wave");
    post("  440 0 sawtooth               - 440Hz sawtooth wave");
    post("  100 300 linterp 0 sinosc     - Linear interpolation between "
         "100-300Hz");
    post("");

    post("Key Concepts:");
    post("  • Stack-based: Arguments come before functions");
    post("  • Postfix notation: '2 3 +' means 2+3");
    post("  • Audio-rate: Most operations work on audio signals");
    post("  • Functional: Pure functions with no side effects");
    post("");

    post("Common Functions:");
    post("  Oscillators: sinosc, sawtooth, square, pulse, noise");
    post("  Math: +, -, *, /, sin, cos, exp, log");
    post("  Audio: *, +, mix, pan, delay, reverb");
    post("  Control: linterp, clip, wrap, fold");
    post("");

    post("Error Help:");
    post("  • Stack underflow: Not enough arguments (try '440 0 sinosc' not "
         "'sinosc')");
    post("  • Undefined symbol: Function name not found (check spelling)");
    post("  • Type error: Wrong argument type (number vs audio signal)");
    post("  • Stack debugging: Use 'stack' to see values, 'clear' to empty");
    post("");

    post("For more info: Send 'status' to check VM state");
    post("sapf~ version with full sapf language interpreter embedded");
}

void sapf_stack(t_sapf* x)
{
    if (!x) {
        error("sapf~: Invalid object pointer");
        return;
    }

    if (!x->sapfThread) {
        error("sapf~: VM thread not initialized");
        return;
    }

    post("sapf~: === STACK INSPECTION ===");

    size_t stackDepth = x->sapfThread->stackDepth();
    post("sapf~: Current stack depth: %zu items", stackDepth);

    if (stackDepth == 0) {
        post("sapf~: Stack is empty");
    } else {
        post("sapf~: Stack contents (top to bottom):");

        try {
            // Use Thread's built-in printStack method
            x->sapfThread->printStack();

        } catch (const std::exception& e) {
            error("sapf~: Error inspecting stack: %s", e.what());
        } catch (...) {
            error("sapf~: Unknown error inspecting stack");
        }

        if (stackDepth > 10) {
            post("sapf~: ⚠ Large stack depth - consider simplifying "
                 "expressions");
        }
    }

    post("sapf~: === END STACK ===");
}

// Input validation and code string construction
ValidationResult sapf_validateInput(t_sapf* x, t_symbol* s, long argc, t_atom* argv)
{
    ValidationResult result;
    result.success = false;

    // Enhanced validation and early error handling
    if (!x) {
        result.errorMessage = "Invalid object pointer (x is null)";
        return result;
    }

    if (!x->sapfThread) {
        result.errorMessage = "sapfThread is null";
        return result;
    }

    if (!x->audioThread) {
        result.errorMessage = "audioThread is null";
        return result;
    }

    // Input validation
    if (argc == 0) {
        result.errorMessage = "No code provided";
        return result;
    }

    // Report current error state if applicable
    if (x->compilationError) {
        post("sapf~: Warning - VM in error state: %s", x->errorMessage);
        post("sapf~: Attempting to compile new code...");
    }

    // Construct sapf code string from Max message atoms with buffer overflow protection
    char codeBuffer[CODE_BUFFER_SIZE];
    codeBuffer[0] = '\0';
    size_t bufferUsed = 0;

    for (long i = 0; i < argc; i++) {
        char atomStr[256];

        // Convert atom to string with type validation
        switch (argv[i].a_type) {
        case A_LONG:
            snprintf_zero(atomStr, sizeof(atomStr), "%ld", argv[i].a_w.w_long);
            break;
        case A_FLOAT:
            snprintf_zero(atomStr, sizeof(atomStr), "%g", argv[i].a_w.w_float);
            break;
        case A_SYM:
            if (argv[i].a_w.w_sym && argv[i].a_w.w_sym->s_name) {
                strncpy_zero(atomStr, argv[i].a_w.w_sym->s_name, sizeof(atomStr) - 1);
                atomStr[sizeof(atomStr) - 1] = '\0';
            } else {
                result.errorMessage = "Invalid symbol atom at position " + std::to_string(i);
                return result;
            }
            break;
        default:
            result.errorMessage = "Unsupported atom type " + std::to_string(argv[i].a_type) + " at position " + std::to_string(i);
            strncpy_zero(atomStr, "?", sizeof(atomStr) - 1);
            atomStr[sizeof(atomStr) - 1] = '\0';
            break;
        }

        // Calculate required space: current atom + space (if not first) + null terminator
        size_t atomLen = strlen(atomStr);
        size_t spaceNeeded = atomLen + (i > 0 ? 1 : 0) + 1; // +1 for space, +1 for null term

        // Check for buffer overflow
        if (bufferUsed + spaceNeeded > sizeof(codeBuffer)) {
            result.errorMessage = "Code string too long - truncated at " + std::to_string(bufferUsed) + " characters";
            return result;
        }

        // Add space if not first atom
        if (i > 0) {
            strncat_zero(codeBuffer, " ", sizeof(codeBuffer) - strlen(codeBuffer) - 1);
            bufferUsed++;
        }

        // Add the atom string
        strncat_zero(codeBuffer, atomStr, sizeof(codeBuffer) - strlen(codeBuffer) - 1);
        bufferUsed += atomLen;
    }

    // Final validation
    if (strlen(codeBuffer) == 0) {
        result.errorMessage = "No valid code generated from input";
        return result;
    }

    // Note: 'play' command is now handled by the sapf interpreter itself
    // via our custom playMax_ primitive that redirects to Max audio system
    if (strstr(codeBuffer, "play")) {
        post("sapf~: ✓ 'play' command detected - will be handled by Max-specific primitive");
    }

    result.success = true;
    result.codeBuffer = std::string(codeBuffer);
    return result;
}

// Compilation and caching logic
CompilationResult sapf_compileCode(t_sapf* x, const std::string& codeBuffer, bool needsRecompilation)
{
    CompilationResult result;
    result.success = false;

    if (!needsRecompilation) {
        post("sapf~: ⚡ Using cached compilation for: %s", codeBuffer.c_str());

        if (x->compilationError) {
            result.errorMessage = "Cached code is in error state: " + std::string(x->errorMessage);
        } else if (x->compiledFunction) {
            post("sapf~: ✓ Cached function ready for audio generation");
            result.success = true;
            result.compiledFunction = x->compiledFunction;
        } else {
            result.errorMessage = "Cached compilation exists but function is null";
        }
        return result;
    }

    post("sapf~: Compiling sapf code: %s", codeBuffer.c_str());

    try {
        // Clear previous compilation state
        x->compilationError = false;
        x->errorMessage[0] = '\0';

        // Clear audio state during compilation (atomic)
        x->hasValidAudio = 0;

        // Attempt compilation with comprehensive error capture
        P<Fun> newCompiledFunction;

        post("sapf~: DEBUG - Starting compilation of: %s", codeBuffer.c_str());
        post("sapf~: DEBUG - About to call x->sapfThread->compile()");
        post("sapf~: DEBUG - sapfThread pointer: %p", x->sapfThread);

        bool success = false;

        try {
            post("sapf~: DEBUG - Calling compile method...");

            // Additional safety check before compilation
            if (!x->sapfThread) {
                post("sapf~: DEBUG - sapfThread is null during compilation");
                success = false;
            } else {
                // Ensure the function pointer is properly initialized to null
                newCompiledFunction = P<Fun>(); // Reset to null

                success = x->sapfThread->compile(codeBuffer.c_str(), newCompiledFunction, true);
                post("sapf~: DEBUG - Compilation phase completed, success=%s",
                     success ? "true" : "false");

                // Additional validation of the compiled function
                if (success && !newCompiledFunction) {
                    post("sapf~: DEBUG - Compilation reported success but function is null");
                    success = false;
                }
            }
        } catch (const std::exception& e) {
            post("sapf~: DEBUG - Exception during compilation: %s", e.what());
            success = false;
            newCompiledFunction = P<Fun>(); // Ensure function is null on error
        } catch (...) {
            post("sapf~: DEBUG - Unknown exception during compilation");
            success = false;
            newCompiledFunction = P<Fun>(); // Ensure function is null on error
        }

        post("sapf~: DEBUG - Checking compilation results: success=%s, function=%s",
             success ? "true" : "false", newCompiledFunction ? "valid" : "null");

        if (success && newCompiledFunction) {
            post("sapf~: DEBUG - Compilation successful, updating state...");
            // Successful compilation - update all state
            x->compiledFunction = newCompiledFunction;
            post("sapf~: DEBUG - Function stored successfully");

            result.success = true;
            result.compiledFunction = newCompiledFunction;

            // Update cached code string with memory safety
            if (x->lastSapfCode) {
                free(x->lastSapfCode);
                x->lastSapfCode = nullptr;
            }

            x->lastSapfCode = strdup(codeBuffer.c_str());
            if (!x->lastSapfCode) {
                error("sapf~: Memory allocation failed for code cache");
                // Continue anyway - compilation succeeded
            }

            post("sapf~: ✓ Compilation complete");
        } else {
            // Compilation failed - set comprehensive error state
            x->compilationError = true;
            x->hasValidAudio = 0;

            // Try to get more detailed error information from sapf
            const char* errorDetail;
            if (!newCompiledFunction) {
                errorDetail = "Function creation failed after parsing";
            } else {
                errorDetail = "Parser returned false (syntax error)";
            }

            snprintf_zero(x->errorMessage, sizeof(x->errorMessage), "Compilation failed: %s", errorDetail);
            result.errorMessage = std::string(errorDetail);

            error("sapf~: ✗ Compilation failed for: \"%s\"", codeBuffer.c_str());
            post("sapf~: Error: %s", errorDetail);
            post("sapf~: Check sapf syntax - try simple expressions like '440 sinosc'");

            // Debug: Print some context about what was being parsed
            post("sapf~: Code length: %zu characters", codeBuffer.length());
            if (codeBuffer.length() > 0) {
                post("sapf~: First char: '%c' (0x%02x)", codeBuffer[0], (unsigned char)codeBuffer[0]);
            }
        }

    } catch (const std::exception& e) {
        // Exception during compilation - handle gracefully with enhanced error reporting
        x->compilationError = true;
        x->hasValidAudio = 0;

        snprintf_zero(x->errorMessage, sizeof(x->errorMessage), "Exception: %s", e.what());
        result.errorMessage = std::string(e.what());

        // Use enhanced error reporting for better user guidance
        reportSapfError(x, codeBuffer.c_str(), e);
    } catch (...) {
        // Catch any other exceptions
        x->compilationError = true;
        x->hasValidAudio = 0;

        strncpy_zero(x->errorMessage, "Unknown exception during compilation", sizeof(x->errorMessage) - 1);
        x->errorMessage[sizeof(x->errorMessage) - 1] = '\0';
        result.errorMessage = "Unknown exception during compilation";

        error("sapf~: ✗ Unknown error during compilation of: %s", codeBuffer.c_str());
        post("sapf~: This may indicate a serious VM issue - consider restarting Max");
    }

    return result;
}

// Function execution and stack management
ExecutionResult sapf_executeCode(t_sapf* x, P<Fun> compiledFunction)
{
    ExecutionResult result;
    result.success = false;
    result.stackDepth = 0;

    try {
        // Check stack depth before execution but don't clear it (preserve sapf stack-based model)
        size_t preStackDepth = x->sapfThread->stackDepth();
        if (preStackDepth > 0) {
            post("sapf~: Executing with %zu items already on stack", preStackDepth);
        }

        // Execute the compiled function with additional safety checks
        post("sapf~: DEBUG - Starting execution...");
        try {
            // Verify thread and function are still valid before execution
            if (!x->sapfThread) {
                throw std::runtime_error("sapfThread became null before execution");
            }
            if (!compiledFunction) {
                throw std::runtime_error("compiledFunction became null before execution");
            }

            // Additional safety: check if the function has valid internal state
            post("sapf~: DEBUG - Function and thread validated, calling apply...");
            compiledFunction->apply(*x->sapfThread);
            post("sapf~: DEBUG - Execution completed successfully");

            // Verify thread is still valid after execution
            if (!x->sapfThread) {
                throw std::runtime_error("sapfThread became null during execution");
            }

        } catch (const std::exception& e) {
            post("sapf~: DEBUG - Exception during execution: %s", e.what());
            throw; // Re-throw to be caught by outer handler
        } catch (...) {
            post("sapf~: DEBUG - Unknown exception during execution");
            throw; // Re-throw to be caught by outer handler
        }

        // Check execution results and manage stack state with safety checks
        if (!x->sapfThread) {
            throw std::runtime_error("sapfThread became null after execution");
        }

        size_t postStackDepth = x->sapfThread->stackDepth();
        post("sapf~: Execution completed, stack depth: %zu", postStackDepth);
        result.stackDepth = postStackDepth;

        // Debug: Print stack contents after execution
        if (postStackDepth > 0) {
            try {
                V topValue = x->sapfThread->top();
                post("sapf~: DEBUG - Top stack value type: %s",
                     topValue.isZIn()          ? "ZIn"
                         : topValue.isList()   ? "List"
                         : topValue.isReal()   ? "Real"
                         : topValue.isObject() ? "Object"
                                               : "Unknown");
            } catch (const std::exception& e) {
                post("sapf~: DEBUG - Error accessing top stack value: %s", e.what());
                // Continue with reduced functionality rather than crashing
            }
        }

        // Check for stack overflow conditions
        const size_t MAX_REASONABLE_STACK_DEPTH = 100;
        if (postStackDepth > MAX_REASONABLE_STACK_DEPTH) {
            post("sapf~: ⚠ WARNING: Very large stack depth (%zu items)", postStackDepth);
            post("sapf~: This may indicate runaway computation or inefficient code");
            post("sapf~: Consider sending 'clear' to reset stack");
        }

        // Simply report execution success - don't process stack for audio
        // Audio is now handled by the 'play' primitive directly
        if (postStackDepth > 0) {
            post("sapf~: ✓ Execution completed, %zu items on stack", postStackDepth);
            result.success = true;
        } else {
            post("sapf~: ✓ Execution completed, stack is empty");
            result.success = true;
        }

    } catch (const std::exception& e) {
        result.errorMessage = "Execution error: " + std::string(e.what());
        x->hasValidAudio = 0;

        // Provide execution-specific error guidance
        post("sapf~: Code compiled successfully but failed during execution");

        if (strstr(e.what(), "stack underflow") || strstr(e.what(), "Stack underflow")) {
            post("sapf~: Hint: Function called without enough arguments");
            post("sapf~: Example: 'sinosc' needs frequency and phase - try '440 0 sinosc'");
        } else if (strstr(e.what(), "type") || strstr(e.what(), "Type")) {
            post("sapf~: Hint: Wrong argument type during execution");
            post("sapf~: Check if numbers are used where audio is expected");
        } else {
            post("sapf~: Hint: Runtime error - try simpler expressions first");
            post("sapf~: Basic test: '440 0 sinosc 0.3 *'");
        }
    }

    return result;
}

// Audio result processing and type checking
AudioProcessingResult sapf_processAudioResult(t_sapf* x, const V& audioResult)
{
    AudioProcessingResult result;
    result.success = false;
    result.hasValidAudio = false;
    result.numChannels = 0;

    // Enhanced multi-channel audio result processing with safe type checking
    const char* resultType = "Unknown";
    try {
        if (audioResult.isReal()) {
            resultType = "Real";
        } else if (audioResult.isObject()) {
            Object* objPtr = audioResult.o();
            if (objPtr != nullptr) {
                // Safe type checking using TypeName instead of type methods
                const char* typeName = objPtr->TypeName();
                if (typeName != nullptr) {
                    resultType = typeName;
                } else {
                    resultType = "CorruptedObject";
                }
            } else {
                resultType = "NullObject";
            }
        } else {
            resultType = "Other";
        }
    } catch (...) {
        resultType = "ExceptionDuringTypeCheck";
    }

    post("sapf~: DEBUG - Processing audio result, type: %s", resultType);

    try {
        // Add defensive validation before calling isZIn() to prevent crashes
        bool isValidZIn = false;
        try {
            // First check if it's an object and the object pointer is valid
            if (audioResult.isObject()) {
                Object* objPtr = audioResult.o();
                if (objPtr != nullptr) {
                    // Perform a basic sanity check by trying to get the type name
                    // This will access the vtable, which crashes if the object is corrupted
                    const char* typeName = objPtr->TypeName();
                    if (typeName != nullptr) {
                        // Object seems valid, now safely call isZIn()
                        isValidZIn = audioResult.isZIn();
                        post("sapf~: DEBUG - Object type: %s, isZIn: %s",
                             typeName, isValidZIn ? "true" : "false");
                    } else {
                        post("sapf~: DEBUG - Object has null TypeName, treating as invalid");
                    }
                } else {
                    post("sapf~: DEBUG - Object pointer is null");
                }
            } else {
                // For non-objects (reals), isZIn() should return false safely
                isValidZIn = audioResult.isZIn();
                post("sapf~: DEBUG - Non-object value, isZIn: %s", isValidZIn ? "true" : "false");
            }
        } catch (const std::exception& e) {
            post("sapf~: DEBUG - Exception during isZIn() validation: %s", e.what());
            isValidZIn = false;
        } catch (...) {
            post("sapf~: DEBUG - Unknown exception during isZIn() validation");
            isValidZIn = false;
        }

        if (isValidZIn) {
            // Single channel audio result (ZList)
            // Lock-free atomic updates
            x->audioExtractor.set(audioResult);
            x->numAudioChannels = 1;

            ATOMIC_INCREMENT(&x->audioStateVersion);
            x->hasValidAudio = 1;

            result.success = true;
            result.hasValidAudio = true;
            result.numChannels = 1;

            post("sapf~: ✓ Single-channel audio result ready for playback");

        } else if (resultType != nullptr
                   && (strcmp(resultType, "VList") == 0 || strcmp(resultType, "ZList") == 0)) {

            // VList and ZList are single-channel audio results
            // Treat them like ZIn objects for single-channel audio
            x->audioExtractor.set(audioResult);
            x->numAudioChannels = 1;

            ATOMIC_INCREMENT(&x->audioStateVersion);
            x->hasValidAudio = 1;

            result.success = true;
            result.hasValidAudio = true;
            result.numChannels = 1;

            post("sapf~: ✓ Single-channel audio result (%s) ready for playback", resultType);

        } else if (audioResult.isList()) {
            // Check if this is a list containing multiple audio objects (multi-channel)
            try {
                P<List> resultList = (List*)audioResult.o();
                if (resultList && resultList->isFinite()) {
                    Array* channels = resultList->mArray.get();
                    if (channels != nullptr && channels->size() > 1) {
                        // This might be a multi-channel audio list - delegate to multi-channel handler
                        post("sapf~: DEBUG - Detected potential multi-channel list with %d elements", (int)channels->size());
                        return sapf_handleMultiChannelAudio(x, audioResult, "List");
                    } else {
                        // Single element list or null array - treat as single-channel
                        x->audioExtractor.set(audioResult);
                        x->numAudioChannels = 1;
                        x->hasValidAudio = 1;

                        result.success = true;
                        result.hasValidAudio = true;
                        result.numChannels = 1;

                        post("sapf~: ✓ Single-element list treated as single-channel audio");
                    }
                } else {
                    // Infinite list - treat as single channel
                    x->audioExtractor.set(audioResult);
                    x->numAudioChannels = 1;
                    x->hasValidAudio = 1;

                    result.success = true;
                    result.hasValidAudio = true;
                    result.numChannels = 1;

                    post("sapf~: ✓ Infinite list treated as single-channel audio");
                }
            } catch (const std::exception& e) {
                // Error processing list - treat as single channel fallback
                x->audioExtractor.set(audioResult);
                x->numAudioChannels = 1;
                x->hasValidAudio = 1;

                result.success = true;
                result.hasValidAudio = true;
                result.numChannels = 1;

                post("sapf~: ⚠ List processing error: %s - using single channel fallback", e.what());
            }

        } else {
            // Non-audio result
            x->hasValidAudio = 0;
            x->numAudioChannels = 0;

            result.errorMessage = "Code executed but result is not audio-compatible";
            post("sapf~: ⚠ Code executed but result is not audio-compatible");
            post("sapf~: Result type: %s", audioResult.isReal() ? "number" : "object");
        }

    } catch (const std::exception& e) {
        result.errorMessage = "Exception during audio result processing: " + std::string(e.what());
        x->hasValidAudio = 0;
        post("sapf~: DEBUG - Exception during audio result processing: %s", e.what());
    } catch (...) {
        result.errorMessage = "Unknown exception during audio result processing";
        x->hasValidAudio = 0;
        post("sapf~: DEBUG - Unknown exception during audio result processing");
    }

    return result;
}

// Multi-channel audio processing
AudioProcessingResult sapf_handleMultiChannelAudio(t_sapf* x, const V& audioResult, const char* resultType)
{
    AudioProcessingResult result;
    result.success = false;
    result.hasValidAudio = false;
    result.numChannels = 0;

    // Potential multi-channel audio result (List of ZLists)
    try {
        P<List> resultList = (List*)audioResult.o();
        if (resultList && resultList->isFinite()) {
            Array* channels = resultList->mArray.get();
            if (channels == nullptr) {
                // mArray returned null - fall back to single channel processing
                post("sapf~: DEBUG - mArray returned null, falling back to single channel");
                x->audioExtractor.set(audioResult);
                x->numAudioChannels = 1;
                x->hasValidAudio = 1;

                result.success = true;
                result.hasValidAudio = true;
                result.numChannels = 1;

                post("sapf~: ✓ Null array fallback to single-channel audio");
                return result;
            }

            int numChannels = std::min((int)channels->size(), 8); // Limit to 8 channels
            post("sapf~: DEBUG - numChannels calculated: %d", numChannels);

            if (numChannels > 0) {
                // Check if all list elements are audio-compatible
                bool allAudioCompatible = true;
                for (int i = 0; i < numChannels; i++) {
                    if (i >= (int)channels->size()) {
                        post("sapf~: ERROR - Channel index %d out of bounds (size: %d)", i, (int)channels->size());
                        allAudioCompatible = false;
                        break;
                    }
                    V channelData = channels->at(i);

                    // Safe type checking for channelData
                    bool isChannelAudio = false;
                    try {
                        if (channelData.isReal()) {
                            isChannelAudio = false; // Raw numbers aren't audio
                        } else if (channelData.isObject()) {
                            Object* channelObj = channelData.o();
                            if (channelObj != nullptr) {
                                const char* channelTypeName = channelObj->TypeName();
                                if (channelTypeName != nullptr
                                    && strcmp(channelTypeName, "ZList") == 0) {
                                    isChannelAudio = true;
                                }
                            }
                        }
                        post("sapf~: DEBUG - Processing channel %d, type check completed", i);
                    } catch (...) {
                        isChannelAudio = false;
                    }

                    if (isChannelAudio) {
                        x->audioExtractors[i].set(channelData);
                    } else {
                        allAudioCompatible = false;
                        break;
                    }
                }

                if (allAudioCompatible) {
                    x->numAudioChannels = numChannels;
                    x->hasValidAudio = 1;

                    result.success = true;
                    result.hasValidAudio = true;
                    result.numChannels = numChannels;

                    post("sapf~: ✓ %d-channel audio result ready for playbook", numChannels);
                } else {
                    x->hasValidAudio = 0;

                    post("sapf~: ⚠ List contains non-audio elements - using single channel fallback");
                    // Fall back to single channel processing
                    x->audioExtractor.set(audioResult);
                    x->numAudioChannels = 1;
                    x->hasValidAudio = 1;

                    result.success = true;
                    result.hasValidAudio = true;
                    result.numChannels = 1;
                }
            } else {
                x->hasValidAudio = 0;
                result.errorMessage = "Empty list - no audio channels";
                post("sapf~: ⚠ Empty list - no audio channels");
            }
        } else {
            // Infinite list - treat as single channel
            x->audioExtractor.set(audioResult);
            x->numAudioChannels = 1;
            x->hasValidAudio = 1;

            result.success = true;
            result.hasValidAudio = true;
            result.numChannels = 1;

            post("sapf~: ✓ Infinite list treated as single-channel audio");
        }
    } catch (const std::exception& e) {
        // Error processing list - fall back to single channel
        x->audioExtractor.set(audioResult);
        x->numAudioChannels = 1;
        x->hasValidAudio = 1;

        result.success = true;
        result.hasValidAudio = true;
        result.numChannels = 1;
        result.errorMessage = "List processing error: " + std::string(e.what()) + " - using single channel";

        post("sapf~: ⚠ List processing error: %s - using single channel", e.what());
    }

    return result;
}

// Final status reporting
void sapf_reportStatus(t_sapf* x, bool compilationError, bool hasValidAudio, P<Fun> compiledFunction)
{
    // Final status summary
    post("sapf~: Status - Error: %s, Audio: %s, Function: %s",
         compilationError ? "YES" : "NO",
         hasValidAudio ? "READY" : "PENDING",
         compiledFunction ? "LOADED" : "NULL");
}

void sapf_clear(t_sapf* x)
{
    if (!x) {
        error("sapf~: Invalid object pointer");
        return;
    }

    if (!x->sapfThread) {
        error("sapf~: VM thread not initialized");
        return;
    }

    size_t stackDepth = x->sapfThread->stackDepth();

    if (stackDepth == 0) {
        post("sapf~: Stack already empty");
    } else {
        try {
            // Clear the stack
            x->sapfThread->clearStack();

            post("sapf~: ✓ Cleared %zu items from stack", stackDepth);

            // Thread-safe clearing of audio state since stack was cleared
            // Lock-free atomic update
            x->hasValidAudio = 0;

            post("sapf~: Audio state cleared due to stack clear");

        } catch (const std::exception& e) {
            error("sapf~: Error clearing stack: %s", e.what());
        } catch (...) {
            error("sapf~: Unknown error clearing stack");
        }
    }
}
