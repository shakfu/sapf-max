#include "VM.hpp"
#include <stdio.h>
#include <histedit.h>
#include <algorithm>
#include <sys/stat.h>
#include "primes.hpp"
#include <complex>
#include <dispatch/dispatch.h>
#include <CoreFoundation/CoreFoundation.h>
#include "Manta.h"

#include "ext.h" // standard Max include, always required (except in Jitter)
#include "ext_obex.h" // required for "new" style objects
#include "z_dsp.h" // required for MSP objects



// struct to represent the object's state
typedef struct _sapf
{
    t_pxobject ob; // the object itself (Max MSP object)
    
    // Legacy field (can be removed later)
    double offset; // the value of a property of our object
    
    // Sapf execution context
    Thread* sapfThread; // Main sapf execution thread
    
    // Compiled sapf code cache
    P<Fun> compiledFunction; // Currently compiled sapf function
    char* lastSapfCode; // Last compiled sapf code string for change detection
    
    // Audio extraction interface
    ZIn audioExtractor; // Interface for extracting audio from sapf results
    bool hasValidAudio; // Flag indicating if audioExtractor contains valid audio data
    
    // Error handling and status
    bool compilationError; // True if last compilation failed
    char errorMessage[256]; // Last error message for debugging
    
    // Sample rate synchronization
    double currentSampleRate; // Current sample rate from Max
    bool sampleRateChanged; // Flag to trigger VM reconfiguration
    
} t_sapf;

// method prototypes
void* sapf_new(t_symbol* s, long argc, t_atom* argv);
void sapf_free(t_sapf* x);
void sapf_assist(t_sapf* x, void* b, long m, long a, char* s);
void sapf_float(t_sapf* x, double f);
void sapf_dsp64(t_sapf* x, t_object* dsp64, short* count, double samplerate, long maxvectorsize, long flags);
void sapf_perform64(t_sapf* x, t_object* dsp64, double** ins, long numins, double** outs, long numouts, long sampleframes, long flags, void* userparam);
void sapf_code(t_sapf* x, t_symbol* s, long argc, t_atom* argv);

// global class pointer variable
static t_class* sapf_class = NULL;

//***********************************************************************************************

void ext_main(void* r)
{
    // object initialization, note the use of dsp_free for the freemethod, which is required
    // unless you need to free allocated memory, in which case you should call dsp_free from
    // your custom free function.

    t_class* c = class_new("sapf~", (method)sapf_new, (method)sapf_free, (long)sizeof(t_sapf), 0L, A_GIMME, 0);

    class_addmethod(c, (method)sapf_float, "float", A_FLOAT, 0);
    class_addmethod(c, (method)sapf_dsp64, "dsp64", A_CANT, 0);
    class_addmethod(c, (method)sapf_assist, "assist", A_CANT, 0);
    class_addmethod(c, (method)sapf_code, "code", A_GIMME, 0);

    class_dspinit(c);
    class_register(CLASS_BOX, c);
    sapf_class = c;
}

void* sapf_new(t_symbol* s, long argc, t_atom* argv)
{
    t_sapf* x = (t_sapf*)object_alloc(sapf_class);

    if (x) {
        dsp_setup((t_pxobject*)x, 1); // MSP inlets: arg is # of inlets and is REQUIRED!
        outlet_new(x, "signal"); // signal outlet (note "signal" rather than NULL)
        
        // Legacy field initialization
        x->offset = 0.0;
        
        // Initialize sapf VM components
        try {
            // Create sapf execution thread
            x->sapfThread = new Thread();
            
            // Initialize compiled function storage
            x->compiledFunction = P<Fun>(); // Initialize empty smart pointer
            x->lastSapfCode = nullptr; // No cached code yet
            
            // Initialize audio extraction interface
            x->audioExtractor = ZIn(); // Initialize empty ZIn
            x->hasValidAudio = false;
            
            // Initialize error handling
            x->compilationError = false;
            x->errorMessage[0] = '\0'; // Empty error message
            
            // Initialize sample rate tracking
            x->currentSampleRate = kDefaultSampleRate; // Default until sapf_dsp64 sets it
            x->sampleRateChanged = true; // Force initial configuration
            
            post("sapf~: Initialized with sapf language interpreter");
            
        } catch (const std::exception& e) {
            post("sapf~: Error initializing sapf VM: %s", e.what());
            
            // Cleanup on failure
            if (x->sapfThread) {
                delete x->sapfThread;
                x->sapfThread = nullptr;
            }
            if (x->lastSapfCode) {
                free(x->lastSapfCode);
                x->lastSapfCode = nullptr;
            }
            
            // Mark as failed
            x->compilationError = true;
            strncpy(x->errorMessage, "VM initialization failed", sizeof(x->errorMessage) - 1);
        }
    }
    return (x);
}

void sapf_free(t_sapf* x)
{
    if (!x) return; // Safety check
    
    post("sapf~: Cleaning up sapf VM resources");
    
    // Clean up sapf Thread (manually allocated)
    if (x->sapfThread) {
        try {
            delete x->sapfThread;
            x->sapfThread = nullptr;
        } catch (const std::exception& e) {
            post("sapf~: Error cleaning up Thread: %s", e.what());
        }
    }
    
    // Clean up cached sapf code string (manually allocated)
    if (x->lastSapfCode) {
        free(x->lastSapfCode);
        x->lastSapfCode = nullptr;
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
    if (!x || !x->sapfThread) {
        post("sapf~: Error - VM not initialized");
        return;
    }
    
    if (x->compilationError) {
        post("sapf~: Warning - VM in error state: %s", x->errorMessage);
    }
    
    // Construct sapf code string from Max message atoms
    char codeBuffer[4096]; // Buffer for sapf code
    codeBuffer[0] = '\0';
    
    for (long i = 0; i < argc; i++) {
        char atomStr[256];
        
        switch (argv[i].a_type) {
            case A_LONG:
                snprintf_zero(atomStr, sizeof(atomStr), "%ld", argv[i].a_w.w_long);
                break;
            case A_FLOAT:
                snprintf_zero(atomStr, sizeof(atomStr), "%g", argv[i].a_w.w_float);
                break;
            case A_SYM:
                strncpy_zero(atomStr, argv[i].a_w.w_sym->s_name, sizeof(atomStr) - 1);
                atomStr[sizeof(atomStr) - 1] = '\0';
                break;
            default:
                strncpy_zero(atomStr, "?", sizeof(atomStr) - 1);
                break;
        }
        
        // Add space if not first atom
        if (i > 0) {
            strncat(codeBuffer, " ", sizeof(codeBuffer) - strlen(codeBuffer) - 1);
        }
        strncat(codeBuffer, atomStr, sizeof(codeBuffer) - strlen(codeBuffer) - 1);
    }
    
    // Check if code has changed (for caching)
    bool needsRecompilation = true;
    if (x->lastSapfCode) {
        needsRecompilation = (strcmp(codeBuffer, x->lastSapfCode) != 0);
    }
    
    if (needsRecompilation) {
        post("sapf~: Compiling sapf code: %s", codeBuffer);
        
        try {
            // Compile sapf code
            P<Fun> newCompiledFunction;
            bool success = x->sapfThread->compile(codeBuffer, newCompiledFunction, true);
            
            if (success && newCompiledFunction) {
                // Update cached function and code
                x->compiledFunction = newCompiledFunction;
                
                // Update cached code string
                if (x->lastSapfCode) {
                    free(x->lastSapfCode);
                }
                x->lastSapfCode = strdup(codeBuffer);
                
                // Clear error state
                x->compilationError = false;
                x->errorMessage[0] = '\0';
                x->hasValidAudio = false; // Will be set when audio is generated
                
                post("sapf~: Compilation successful");
                
            } else {
                // Compilation failed
                x->compilationError = true;
                strncpy(x->errorMessage, "Compilation failed", sizeof(x->errorMessage) - 1);
                x->hasValidAudio = false;
                
                post("sapf~: Compilation failed for: %s", codeBuffer);
            }
            
        } catch (const std::exception& e) {
            // Exception during compilation
            x->compilationError = true;
            snprintf(x->errorMessage, sizeof(x->errorMessage), "Exception: %s", e.what());
            x->hasValidAudio = false;
            
            post("sapf~: Compilation error: %s", e.what());
        }
        
    } else {
        post("sapf~: Using cached compilation for: %s", codeBuffer);
    }
}

void sapf_assist(t_sapf* x, void* b, long m, long a, char* s)
{
    if (m == ASSIST_INLET) { // inlet
        sprintf(s, "I am inlet %ld", a);
    }
    else { // outlet
        sprintf(s, "I am outlet %ld", a);
    }
}

void sapf_float(t_sapf* x, double f)
{
    x->offset = f;
}

// registers a function for the signal chain in Max
void sapf_dsp64(t_sapf* x, t_object* dsp64, short* count, double samplerate, long maxvectorsize, long flags)
{
    post("my sample rate is: %f", samplerate);

    // instead of calling dsp_add(), we send the "dsp_add64" message to the object representing the dsp chain
    // the arguments passed are:
    // 1: the dsp64 object passed-in by the calling function
    // 2: the symbol of the "dsp_add64" message we are sending
    // 3: a pointer to your object
    // 4: a pointer to your 64-bit perform method
    // 5: flags to alter how the signal chain handles your object -- just pass 0
    // 6: a generic pointer that you can use to pass any additional data to your perform method

    object_method(dsp64, gensym("dsp_add64"), x, sapf_perform64, 0, NULL);
}

// this is the 64-bit perform method audio vectors
void sapf_perform64(t_sapf* x, t_object* dsp64, double** ins, long numins, double** outs, long numouts, long sampleframes, long flags, void* userparam)
{
    t_double* inL = ins[0]; // we get audio for each inlet of the object from the **ins argument
    t_double* outL = outs[0]; // we get audio for each outlet of the object from the **outs argument
    int n = sampleframes;

    // this perform method simply copies the input to the output, offsetting the value
    while (n--) {
        *outL++ = *inL++ + x->offset;
    }
}
