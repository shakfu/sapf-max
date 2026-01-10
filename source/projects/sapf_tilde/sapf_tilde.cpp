// sapf~ - Max/MSP external using the SAPF language library
// Uses the sapf_engine library for all SAPF functionality

// Max SDK headers MUST be included first due to 'post' function declaration
#include "ext.h"
#include "ext_obex.h"
#include "z_dsp.h"

// SAPF library headers (after Max headers to use Max's post function)
#include "sapf/Engine.hpp"
#include "sapf/MaxAudioBackend.hpp"
#include "VM.hpp"

#include <CoreFoundation/CoreFoundation.h>
#include <algorithm>
#include <cstring>
#include <vector>

#define CODE_BUFFER_SIZE 4096

// enums for inlets / outlets
enum INLETS { I_INPUT, NUM_INLETS };
enum OUTLETS { O_OUTPUT, NUM_OUTLETS };

// struct to represent the object's state
typedef struct _sapf {
    t_pxobject ob;

    // Sapf execution context
    Thread* mainThread;

    // Audio output buffers (float* for MaxMSPProcessAudio)
    std::vector<float*> audioBuffers;
    long bufferSize;
    int numOutputChannels;

    // Sample rate tracking
    double currentSampleRate;

    // Error handling
    char errorMessage[256];

    // Non-audio outlet for text output
    void* text_outlet;
} t_sapf;

// method prototypes
void* sapf_new(t_symbol* s, long argc, t_atom* argv);
void sapf_free(t_sapf* x);
void sapf_assist(t_sapf* x, void* b, long m, long a, char* s);
void sapf_dsp64(t_sapf* x, t_object* dsp64, short* count, double samplerate, long maxvectorsize, long flags);
void sapf_perform64(t_sapf* x, t_object* dsp64, double** ins, long numins, double** outs, long numouts,
                    long sampleframes, long flags, void* userparam);
void sapf_anything(t_sapf* x, t_symbol* s, long argc, t_atom* argv);
void sapf_list(t_sapf* x, t_symbol* s, long argc, t_atom* argv);
void sapf_status(t_sapf* x);
void sapf_help(t_sapf* x);
void sapf_stack(t_sapf* x);
void sapf_clear(t_sapf* x);
void sapf_stop(t_sapf* x);

// Helper function to output stack contents to text outlet
void outputStackToTextOutlet(t_sapf* x);

// global class pointer variable
static t_class* sapf_class = NULL;

// Flag to track if sapf engine has been initialized globally
static bool gSapfEngineInitialized = false;

// Initialize the SAPF engine (called once globally)
void initSapfEngine()
{
    if (gSapfEngineInitialized)
        return;

    post("sapf~: Initializing SAPF engine...");

    try {
        SapfEngine& engine = GetSapfEngine();

        SapfEngineConfig config;
        config.sampleRate = sys_getsr();
        config.preludeFile = nullptr;  // Will be loaded per-instance if needed
        config.logFile = nullptr;
        config.enableManta = false;  // Disable Manta in Max context

        engine.configure(config);
        engine.initialize();

        // Install Max-specific audio backend
        InstallMaxMSPBackend();

        gSapfEngineInitialized = true;
        post("sapf~: SAPF engine initialized (version %s)", engine.versionString());

    } catch (const std::exception& e) {
        error("sapf~: Error initializing SAPF engine: %s", e.what());
    } catch (...) {
        error("sapf~: Unknown error initializing SAPF engine");
    }
}

//***********************************************************************************************

void ext_main(void* r)
{
    t_class* c = class_new("sapf~", (method)sapf_new, (method)sapf_free, (long)sizeof(t_sapf), 0L, A_GIMME, 0);

    class_addmethod(c, (method)sapf_dsp64, "dsp64", A_CANT, 0);
    class_addmethod(c, (method)sapf_assist, "assist", A_CANT, 0);
    class_addmethod(c, (method)sapf_anything, "anything", A_GIMME, 0);
    class_addmethod(c, (method)sapf_list, "list", A_GIMME, 0);
    class_addmethod(c, (method)sapf_status, "status", 0);
    class_addmethod(c, (method)sapf_help, "help", 0);
    class_addmethod(c, (method)sapf_stack, "stack", 0);
    class_addmethod(c, (method)sapf_clear, "clear", 0);
    class_addmethod(c, (method)sapf_stop, "stop", 0);

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

        // Parse arguments for number of output channels (default 2)
        x->numOutputChannels = 2;
        if (argc > 0 && atom_gettype(argv) == A_LONG) {
            x->numOutputChannels = std::max(1L, std::min(8L, atom_getlong(argv)));
        }

        // General (non-audio) outlet for text output
        x->text_outlet = outlet_new((t_object*)x, NULL);

        // Create audio outlets
        for (int i = 0; i < x->numOutputChannels; i++) {
            outlet_new(x, "signal");
        }

        try {
            // Initialize SAPF engine (only once globally)
            initSapfEngine();

            // Create main execution thread
            x->mainThread = new Thread();

            // Load prelude file
            const char* preludePath = "sapf-prelude.txt";
            try {
                loadFile(*x->mainThread, preludePath);
                post("sapf~: Prelude loaded from %s", preludePath);
            } catch (const std::exception& e) {
                post("sapf~: Warning - Could not load prelude: %s", e.what());
            } catch (...) {
                post("sapf~: Warning - Could not load prelude");
            }

            // Initialize audio buffers
            x->audioBuffers.clear();
            x->bufferSize = 0;

            // Initialize sample rate
            x->currentSampleRate = sys_getsr();

            // Initialize error state
            x->errorMessage[0] = '\0';

            post("sapf~: Initialized with %d output channels", x->numOutputChannels);

        } catch (const std::exception& e) {
            error("sapf~: Error during initialization: %s", e.what());
            if (x->mainThread) {
                delete x->mainThread;
                x->mainThread = nullptr;
            }
        }
    }
    return (x);
}

void sapf_free(t_sapf* x)
{
    if (!x)
        return;

    post("sapf~: Cleaning up...");

    // Clean up main thread
    if (x->mainThread) {
        delete x->mainThread;
        x->mainThread = nullptr;
    }

    // Clean up audio buffers
    for (float* buf : x->audioBuffers) {
        delete[] buf;
    }
    x->audioBuffers.clear();

    // Must call dsp_free
    dsp_free((t_pxobject*)x);

    post("sapf~: Cleanup complete");
}

void sapf_anything(t_sapf* x, t_symbol* s, long argc, t_atom* argv)
{
    if (!x || !x->mainThread) {
        error("sapf~: Object not properly initialized");
        return;
    }

    // Construct code string from selector symbol and Max message atoms
    char codeBuffer[CODE_BUFFER_SIZE];
    codeBuffer[0] = '\0';
    size_t bufferUsed = 0;

    // Start with the selector symbol (first word of the message)
    if (s && s->s_name) {
        size_t symLen = strlen(s->s_name);
        if (symLen >= sizeof(codeBuffer)) {
            error("sapf~: Code string too long");
            return;
        }
        strcpy(codeBuffer, s->s_name);
        bufferUsed = symLen;
    }

    // Append remaining atoms
    for (long i = 0; i < argc; i++) {
        char atomStr[256];

        switch (atom_gettype(&argv[i])) {
        case A_LONG:
            snprintf(atomStr, sizeof(atomStr), "%lld", (long long)atom_getlong(&argv[i]));
            break;
        case A_FLOAT:
            snprintf(atomStr, sizeof(atomStr), "%g", atom_getfloat(&argv[i]));
            break;
        case A_SYM:
            if (atom_getsym(&argv[i]) && atom_getsym(&argv[i])->s_name) {
                strncpy(atomStr, atom_getsym(&argv[i])->s_name, sizeof(atomStr) - 1);
                atomStr[sizeof(atomStr) - 1] = '\0';
            } else {
                error("sapf~: Invalid symbol at position %ld", i);
                return;
            }
            break;
        default:
            error("sapf~: Unsupported atom type at position %ld", i);
            return;
        }

        size_t atomLen = strlen(atomStr);
        size_t spaceNeeded = atomLen + 2;  // space + atom + null

        if (bufferUsed + spaceNeeded > sizeof(codeBuffer)) {
            error("sapf~: Code string too long");
            return;
        }

        strcat(codeBuffer, " ");
        bufferUsed++;
        strcat(codeBuffer, atomStr);
        bufferUsed += atomLen;
    }

    post("sapf~: Compiling: %s", codeBuffer);

    try {
        // Compile the code
        P<Fun> compiledFunction;
        bool success = x->mainThread->compile(codeBuffer, compiledFunction, true);

        if (success && compiledFunction) {
            // Execute the compiled function
            compiledFunction->apply(*x->mainThread);

            size_t stackDepth = x->mainThread->stackDepth();
            if (stackDepth > 0) {
                post("sapf~: Executed, %zu items on stack", stackDepth);
                outputStackToTextOutlet(x);
            } else {
                post("sapf~: Executed, stack empty");
            }
        } else {
            error("sapf~: Compilation failed for: %s", codeBuffer);
        }

    } catch (const std::exception& e) {
        error("sapf~: Error: %s", e.what());
        strncpy(x->errorMessage, e.what(), sizeof(x->errorMessage) - 1);
        x->errorMessage[sizeof(x->errorMessage) - 1] = '\0';
    } catch (...) {
        error("sapf~: Unknown error during execution");
    }
}

void sapf_list(t_sapf* x, t_symbol* s, long argc, t_atom* argv)
{
    // List messages start with a number, so there's no selector symbol
    // Pass nullptr as the symbol and forward all atoms to sapf_anything
    sapf_anything(x, nullptr, argc, argv);
}

void sapf_status(t_sapf* x)
{
    post("sapf~: === STATUS ===");

    if (x->mainThread) {
        post("sapf~: Thread: initialized");
        post("sapf~: Stack depth: %zu", x->mainThread->stackDepth());
    } else {
        post("sapf~: Thread: NOT initialized");
    }

    post("sapf~: Sample rate: %.1f Hz", x->currentSampleRate);
    post("sapf~: Output channels: %d", x->numOutputChannels);
    post("sapf~: Buffer size: %ld", x->bufferSize);

    if (HasAudioBackend()) {
        post("sapf~: Audio backend: MaxAudioBackend");
    } else {
        post("sapf~: Audio backend: NONE");
    }

    if (x->errorMessage[0] != '\0') {
        post("sapf~: Last error: %s", x->errorMessage);
    }

    post("sapf~: SAPF version: %s", SapfGetVersionString());
    post("sapf~: === END STATUS ===");
}

void sapf_assist(t_sapf* x, void* b, long io, long idx, char* s)
{
    if (io == ASSIST_INLET) {
        snprintf(s, ASSIST_MAX_STRING_LEN, "signal input, SAPF code messages");
    } else if (io == ASSIST_OUTLET) {
        if (idx < x->numOutputChannels) {
            snprintf(s, ASSIST_MAX_STRING_LEN, "signal output %ld", idx + 1);
        } else {
            snprintf(s, ASSIST_MAX_STRING_LEN, "text output");
        }
    }
}

void sapf_dsp64(t_sapf* x, t_object* dsp64, short* count, double samplerate, long maxvectorsize, long flags)
{
    if (!x) {
        error("sapf~: Invalid object in dsp64");
        return;
    }

    post("sapf~: DSP setup: sample rate %.1f Hz, vector size %ld", samplerate, maxvectorsize);

    // Update sample rate if changed
    if (x->currentSampleRate != samplerate) {
        x->currentSampleRate = samplerate;
        vm.setSampleRate(samplerate);
        post("sapf~: Sample rate updated to %.1f Hz", samplerate);
    }

    // Reallocate audio buffers if needed
    if (x->bufferSize != maxvectorsize) {
        // Free old buffers
        for (float* buf : x->audioBuffers) {
            delete[] buf;
        }
        x->audioBuffers.clear();

        // Allocate new buffers
        for (int i = 0; i < x->numOutputChannels; i++) {
            x->audioBuffers.push_back(new float[maxvectorsize]);
        }
        x->bufferSize = maxvectorsize;
        post("sapf~: Audio buffers allocated (%d channels x %ld samples)", x->numOutputChannels, maxvectorsize);
    }

    object_method(dsp64, gensym("dsp_add64"), x, sapf_perform64, 0, NULL);
}

void sapf_perform64(t_sapf* x, t_object* dsp64, double** ins, long numins, double** outs, long numouts,
                    long sampleframes, long flags, void* userparam)
{
    if (!x || x->audioBuffers.empty()) {
        // Output silence
        for (long chan = 0; chan < numouts; chan++) {
            memset(outs[chan], 0, sizeof(double) * sampleframes);
        }
        return;
    }

    // Get float* pointers for MaxMSPProcessAudio
    int numChannels = std::min((int)numouts, (int)x->audioBuffers.size());

    // Clear the float buffers first
    for (int ch = 0; ch < numChannels; ch++) {
        memset(x->audioBuffers[ch], 0, sizeof(float) * sampleframes);
    }

    // Call the library's audio processing function
    MaxMSPProcessAudio(x->audioBuffers.data(), numChannels, (int)sampleframes);

    // Convert float buffers to Max's double outputs
    for (int ch = 0; ch < numChannels; ch++) {
        float* src = x->audioBuffers[ch];
        double* dst = outs[ch];
        for (long i = 0; i < sampleframes; i++) {
            dst[i] = (double)src[i];
        }
    }

    // Zero any remaining output channels
    for (long ch = numChannels; ch < numouts; ch++) {
        memset(outs[ch], 0, sizeof(double) * sampleframes);
    }
}

void sapf_help(t_sapf* x)
{
    post("sapf~: === SAPF LANGUAGE HELP ===");
    post("");
    post("sapf~ is a Max external embedding the SAPF language interpreter");
    post("SAPF (Sound As Pure Form) is a functional, stack-based audio language");
    post("");
    post("Basic Usage:");
    post("  Send SAPF code directly as messages");
    post("  Example: [440 0 sinosc 0.3 * play]");
    post("");
    post("Commands:");
    post("  status  - Show VM status");
    post("  help    - Show this help");
    post("  stack   - Inspect stack contents");
    post("  clear   - Clear the stack");
    post("  stop    - Stop all audio playback");
    post("");
    post("Examples:");
    post("  440 0 sinosc play                 - 440Hz sine wave");
    post("  440 0 sinosc 0.3 * play           - Sine at 30% volume");
    post("  [440 550] 0 sinosc play           - Two-channel output");
    post("  220 330 + 0 sinosc play           - Math: (220+330)Hz");
    post("");
    post("SAPF version: %s", SapfGetVersionString());
}

void sapf_stack(t_sapf* x)
{
    if (!x || !x->mainThread) {
        error("sapf~: Object not initialized");
        return;
    }

    size_t depth = x->mainThread->stackDepth();
    post("sapf~: Stack depth: %zu", depth);

    if (depth == 0) {
        post("sapf~: Stack is empty");
    } else {
        x->mainThread->printStack();
    }
}

void sapf_clear(t_sapf* x)
{
    if (!x || !x->mainThread) {
        error("sapf~: Object not initialized");
        return;
    }

    size_t depth = x->mainThread->stackDepth();
    if (depth > 0) {
        x->mainThread->clearStack();
        post("sapf~: Cleared %zu items from stack", depth);
    } else {
        post("sapf~: Stack already empty");
    }
}

void sapf_stop(t_sapf* x)
{
    if (HasAudioBackend()) {
        try {
            GetAudioBackend().stopAll();
            post("sapf~: Audio stopped");
        } catch (const std::exception& e) {
            error("sapf~: Error stopping audio: %s", e.what());
        }
    }
}

void outputStackToTextOutlet(t_sapf* x)
{
    if (!x || !x->mainThread || !x->text_outlet) {
        return;
    }

    try {
        size_t stackDepth = x->mainThread->stackDepth();

        if (stackDepth == 0) {
            t_atom emptyAtom;
            atom_setsym(&emptyAtom, gensym("stack_empty"));
            outlet_anything(x->text_outlet, gensym("stack"), 1, &emptyAtom);
        } else {
            // Output each stack item
            for (size_t i = 0; i < stackDepth; i++) {
                V stackItem = x->mainThread->stack[x->mainThread->stackBase + i];

                if (stackItem.isReal()) {
                    t_atom valueAtom;
                    atom_setfloat(&valueAtom, (float)stackItem.asFloat());
                    outlet_anything(x->text_outlet, gensym("value"), 1, &valueAtom);
                } else if (stackItem.isList()) {
                    P<List> list = (List*)stackItem.o();
                    if (list && list->isFinite()) {
                        Array* arr = list->mArray.get();
                        if (arr && arr->size() <= 10) {
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
                            }
                        } else {
                            t_atom sym;
                            atom_setsym(&sym, gensym("[large_list]"));
                            outlet_anything(x->text_outlet, gensym("value"), 1, &sym);
                        }
                    } else {
                        t_atom sym;
                        atom_setsym(&sym, gensym("[infinite_list]"));
                        outlet_anything(x->text_outlet, gensym("value"), 1, &sym);
                    }
                } else if (stackItem.isObject()) {
                    Object* obj = stackItem.o();
                    if (obj) {
                        const char* typeName = obj->TypeName();
                        t_atom typeAtom;
                        atom_setsym(&typeAtom, gensym(typeName ? typeName : "[object]"));
                        outlet_anything(x->text_outlet, gensym("object"), 1, &typeAtom);
                    }
                }
            }
        }

    } catch (const std::exception& e) {
        post("sapf~: Error outputting stack: %s", e.what());
    }
}
