#include "VM.hpp"
#include <stdio.h>
#include <histedit.h>
#include <algorithm>
#include <sys/stat.h>
#include "primes.hpp"
#include <complex>
#include <vector>
#include <dispatch/dispatch.h>
#include <CoreFoundation/CoreFoundation.h>
#include <os/lock.h>
#include "Manta.h"
#include "ErrorCodes.hpp"

// Forward declarations for sapf builtin initialization functions
extern void AddCoreOps();
extern void AddMathOps();
extern void AddStreamOps();
extern void AddUGenOps();
extern void AddSetOps();
extern void AddRandomOps();
extern void AddMidiOps();

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
    
    // Thread synchronization for real-time audio safety
    os_unfair_lock audioStateLock; // Protects audio-related shared state
    
} t_sapf;

// method prototypes
void* sapf_new(t_symbol* s, long argc, t_atom* argv);
void sapf_free(t_sapf* x);
void sapf_assist(t_sapf* x, void* b, long m, long a, char* s);
void sapf_float(t_sapf* x, double f);
void sapf_dsp64(t_sapf* x, t_object* dsp64, short* count, double samplerate, long maxvectorsize, long flags);
void sapf_perform64(t_sapf* x, t_object* dsp64, double** ins, long numins, double** outs, long numouts, long sampleframes, long flags, void* userparam);
void sapf_code(t_sapf* x, t_symbol* s, long argc, t_atom* argv);
void sapf_status(t_sapf* x);
void sapf_help(t_sapf* x);

// global class pointer variable
static t_class* sapf_class = NULL;

// Flag to track if sapf builtins have been initialized globally
static bool gSapfBuiltinsInitialized = false;

// Initialize all sapf built-in functions
void initSapfBuiltins() {
    if (gSapfBuiltinsInitialized) return; // Already initialized
    
    post("sapf~: Initializing sapf built-in functions...");
    
    try {
        AddCoreOps();
        AddMathOps();
        AddStreamOps();
        AddRandomOps();
        AddUGenOps();  // This includes AddOscilUGenOps() which has sinosc
        AddMidiOps();
        AddSetOps();
        
        gSapfBuiltinsInitialized = true;
        post("sapf~: Built-in functions initialized successfully");
        
    } catch (const std::exception& e) {
        error("sapf~: Error initializing built-ins: %s", e.what());
    } catch (...) {
        error("sapf~: Unknown error initializing built-ins");
    }
}

// Enhanced error reporting with specific error type detection and user guidance
void reportSapfError(t_sapf* x, const char* codeBuffer, const std::exception& e) {
    const char* errorMsg = e.what();
    
    // Check for specific error types and provide contextual guidance
    if (strstr(errorMsg, "Undefined")) {
        error("sapf~: ✗ Undefined symbol in: \"%s\"", codeBuffer);
        post("sapf~: Error: %s", errorMsg);
        post("sapf~: Hint: Check function names - available: sinosc, play, +, -, *, /, etc.");
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
        post("sapf~: Hint: Wrong argument type - check number vs audio vs array");
        
    } else if (strstr(errorMsg, "range") || strstr(errorMsg, "Range")) {
        error("sapf~: ✗ Range error in: \"%s\"", codeBuffer);
        post("sapf~: Error: %s", errorMsg);
        post("sapf~: Hint: Value out of valid range - check array indices, frequencies");
        
    } else if (strstr(errorMsg, "memory") || strstr(errorMsg, "alloc")) {
        error("sapf~: ✗ Memory error in: \"%s\"", codeBuffer);
        post("sapf~: Error: %s", errorMsg);
        post("sapf~: Hint: Out of memory - try simpler code or restart Max");
        
    } else {
        // Generic error with basic guidance
        error("sapf~: ✗ Compilation error: %s", errorMsg);
        post("sapf~: Code: \"%s\"", codeBuffer);
        post("sapf~: Hint: Try simpler expressions like '440 sinosc' or send 'status'");
    }
    
    // Always provide general help
    post("sapf~: For help: send 'status' for VM info, or try basic examples:");
    post("sapf~: '440 0 sinosc' (sine wave) or '220 330 + 0 sinosc' (math)");
}

//***********************************************************************************************

void ext_main(void* r)
{
    // object initialization, note the use of dsp_free for the freemethod, which is required
    // unless you need to free allocated memory, in which case you should call dsp_free from
    // your custom free function.

    t_class* c = class_new("sapf~", (method)sapf_new, (method)sapf_free, (long)sizeof(t_sapf), 0L, A_GIMME, 0);

    class_addmethod(c, (method)sapf_float,  "float",    A_FLOAT, 0);
    class_addmethod(c, (method)sapf_dsp64,  "dsp64",    A_CANT,  0);
    class_addmethod(c, (method)sapf_assist, "assist",   A_CANT,  0);
    class_addmethod(c, (method)sapf_code,   "code",     A_GIMME, 0);
    class_addmethod(c, (method)sapf_status, "status",   0);
    class_addmethod(c, (method)sapf_help,   "help",     0);

    class_dspinit(c);
    class_register(CLASS_BOX, c);
    sapf_class = c;
}

void* sapf_new(t_symbol* s, long argc, t_atom* argv)
{
    t_sapf* x = (t_sapf*)object_alloc(sapf_class);

    post("hello");

    if (x) {
        dsp_setup((t_pxobject*)x, 1); // MSP inlets: arg is # of inlets and is REQUIRED!
        outlet_new(x, "signal"); // signal outlet (note "signal" rather than NULL)
        
        // Legacy field initialization
        x->offset = 0.0;
        
        // Initialize sapf VM components
        try {
            // Initialize sapf built-in functions (only once globally)
            initSapfBuiltins();
            
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
            
            // Initialize thread synchronization
            x->audioStateLock = OS_UNFAIR_LOCK_INIT;
            
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
            strncpy_zero(x->errorMessage, "VM initialization failed", sizeof(x->errorMessage) - 1);
            x->errorMessage[sizeof(x->errorMessage) - 1] = '\0';
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
    // Enhanced validation and early error handling
    if (!x) {
        error("sapf~: Invalid object pointer");
        return;
    }
    
    if (!x->sapfThread) {
        error("sapf~: VM not initialized - object creation may have failed");
        x->compilationError = true;
        strncpy_zero(x->errorMessage, "VM not initialized", sizeof(x->errorMessage) - 1);
        x->errorMessage[sizeof(x->errorMessage) - 1] = '\0';
        return;
    }
    
    // Input validation
    if (argc == 0) {
        post("sapf~: No code provided");
        return;
    }
    
    // Report current error state if applicable
    if (x->compilationError) {
        post("sapf~: Warning - VM in error state: %s", x->errorMessage);
        post("sapf~: Attempting to compile new code...");
    }
    
    // Construct sapf code string from Max message atoms with buffer overflow protection
    char codeBuffer[4096]; // Buffer for sapf code
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
                    error("sapf~: Invalid symbol atom at position %ld", i);
                    return;
                }
                break;
            default:
                error("sapf~: Unsupported atom type %d at position %ld", argv[i].a_type, i);
                strncpy_zero(atomStr, "?", sizeof(atomStr) - 1);
                atomStr[sizeof(atomStr) - 1] = '\0';
                break;
        }
        
        // Calculate required space: current atom + space (if not first) + null terminator
        size_t atomLen = strlen(atomStr);
        size_t spaceNeeded = atomLen + (i > 0 ? 1 : 0) + 1; // +1 for space, +1 for null term
        
        // Check for buffer overflow
        if (bufferUsed + spaceNeeded > sizeof(codeBuffer)) {
            error("sapf~: Code string too long - truncated at %zu characters", bufferUsed);
            break;
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
        post("sapf~: No valid code generated from input");
        return;
    }
    
    // Check if code has changed (for caching)
    bool needsRecompilation = true;
    if (x->lastSapfCode) {
        needsRecompilation = (strcmp(codeBuffer, x->lastSapfCode) != 0);
    }
    
    if (needsRecompilation) {
        post("sapf~: Compiling sapf code: %s", codeBuffer);
        
        try {
            // Clear previous compilation state
            x->compilationError = false;
            x->errorMessage[0] = '\0';
            
            // Thread-safe clearing of audio state during compilation
            os_unfair_lock_lock(&x->audioStateLock);
            x->hasValidAudio = false;
            os_unfair_lock_unlock(&x->audioStateLock);
            
            // Attempt compilation with comprehensive error capture
            P<Fun> newCompiledFunction;
            bool success = x->sapfThread->compile(codeBuffer, newCompiledFunction, true);

            if (success && newCompiledFunction) {
                // Successful compilation - update all state
                x->compiledFunction = newCompiledFunction;

                // Execute the compiled sapf code to generate audio result
                try {
                    // Execute the compiled function
                    newCompiledFunction->apply(*x->sapfThread);

                    // Check if execution produced audio results on the stack
                    if (x->sapfThread->stackDepth() > 0) {
                        // Get the top result from execution
                        V audioResult = x->sapfThread->pop();

                        // Check if result is audio-compatible (ZIn compatible)
                        if (audioResult.isZIn()) {
                            // Thread-safe update of audio state
                            os_unfair_lock_lock(&x->audioStateLock);
                            x->audioExtractor.set(audioResult);
                            x->hasValidAudio = true;
                            os_unfair_lock_unlock(&x->audioStateLock);

                            post("sapf~: ✓ Audio result ready for playback");
                        } else {
                            // Clear audio state if result is not audio-compatible
                            os_unfair_lock_lock(&x->audioStateLock);
                            x->hasValidAudio = false;
                            os_unfair_lock_unlock(&x->audioStateLock);
                            
                            post("sapf~: ⚠ Code executed but result is not audio-compatible");
                            post("sapf~: Result type: %s", audioResult.isReal() ? "number" : "object");
                        }
                    } else {
                        post("sapf~: ⚠ Code executed but produced no results");
                    }

                } catch (const std::exception& e) {
                    error("sapf~: ✗ Execution error: %s", e.what());
                    
                    // Thread-safe clearing of audio state on execution error
                    os_unfair_lock_lock(&x->audioStateLock);
                    x->hasValidAudio = false;
                    os_unfair_lock_unlock(&x->audioStateLock);
                    
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
                
                // Update cached code string with memory safety
                if (x->lastSapfCode) {
                    free(x->lastSapfCode);
                    x->lastSapfCode = nullptr;
                }
                
                x->lastSapfCode = strdup(codeBuffer);
                if (!x->lastSapfCode) {
                    error("sapf~: Memory allocation failed for code cache");
                    // Continue anyway - compilation succeeded
                }
                
                // Report success with details
                post("sapf~: ✓ Compilation and execution complete");
                
                // TODO: Could add function introspection here
                // e.g., post("sapf~: Function takes %d inputs, produces %d outputs", ...)
                
            } else {
                // Compilation failed - set comprehensive error state
                x->compilationError = true;
                
                // Thread-safe clearing of audio state on compilation failure
                os_unfair_lock_lock(&x->audioStateLock);
                x->hasValidAudio = false;
                os_unfair_lock_unlock(&x->audioStateLock);
                
                // Try to get more detailed error information from sapf
                // Check if it's a parsing issue or function creation issue
                const char* errorDetail;
                if (!newCompiledFunction) {
                    errorDetail = "Function creation failed after parsing";
                } else {
                    errorDetail = "Parser returned false (syntax error)";
                }
                
                snprintf_zero(x->errorMessage, sizeof(x->errorMessage), "Compilation failed: %s", errorDetail);
                
                error("sapf~: ✗ Compilation failed for: \"%s\"", codeBuffer);
                post("sapf~: Error: %s", errorDetail);
                post("sapf~: Check sapf syntax - try simple expressions like '440 sinosc'");
                
                // Debug: Print some context about what was being parsed
                post("sapf~: Code length: %zu characters", strlen(codeBuffer));
                if (strlen(codeBuffer) > 0) {
                    post("sapf~: First char: '%c' (0x%02x)", codeBuffer[0], (unsigned char)codeBuffer[0]);
                }
            }
            
        } catch (const std::exception& e) {
            // Exception during compilation - handle gracefully with enhanced error reporting
            x->compilationError = true;
            x->hasValidAudio = false;
            
            snprintf_zero(x->errorMessage, sizeof(x->errorMessage), "Exception: %s", e.what());
            
            // Use enhanced error reporting for better user guidance
            reportSapfError(x, codeBuffer, e);
        } catch (...) {
            // Catch any other exceptions
            x->compilationError = true;
            
            // Thread-safe clearing of audio state on unknown exception
            os_unfair_lock_lock(&x->audioStateLock);
            x->hasValidAudio = false;
            os_unfair_lock_unlock(&x->audioStateLock);
            
            strncpy_zero(x->errorMessage, "Unknown exception during compilation", sizeof(x->errorMessage) - 1);
            x->errorMessage[sizeof(x->errorMessage) - 1] = '\0';
            
            error("sapf~: ✗ Unknown error during compilation of: %s", codeBuffer);
            post("sapf~: This may indicate a serious VM issue - consider restarting Max");
        }
        
    } else {
        post("sapf~: ⚡ Using cached compilation for: %s", codeBuffer);
        
        // Even for cached code, report current status
        if (x->compilationError) {
            post("sapf~: ⚠ Cached code is in error state: %s", x->errorMessage);
        } else if (x->compiledFunction) {
            post("sapf~: ✓ Cached function ready for audio generation");
        } else {
            post("sapf~: ⚠ Cached compilation exists but function is null");
        }
    }
    
    // Final status summary
    post("sapf~: Status - Error: %s, Audio: %s, Function: %s", 
         x->compilationError ? "YES" : "NO",
         x->hasValidAudio ? "READY" : "PENDING",
         x->compiledFunction ? "LOADED" : "NULL");
}

void sapf_status(t_sapf* x)
{
    // if (!x) {
    //     error("sapf~: Invalid object pointer");
    //     return;
    // }
    
    post("sapf~: === STATUS REPORT ===");
    
    // VM Initialization Status
    if (x->sapfThread) {
        post("sapf~: VM: ✓ Initialized and ready");
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
    os_unfair_lock_lock(&x->audioStateLock);
    audioStatus = x->hasValidAudio;
    os_unfair_lock_unlock(&x->audioStateLock);
    
    if (audioStatus) {
        post("sapf~: Audio: ✓ Ready for audio generation (thread-safe)");
    } else {
        post("sapf~: Audio: ○ No audio data generated yet (thread-safe)");
    }
    
    // Sample Rate Status
    post("sapf~: Sample Rate: %.1f Hz %s", 
         x->currentSampleRate,
         x->sampleRateChanged ? "(changed, needs VM update)" : "(synchronized)");
    
    // Memory Status
    post("sapf~: Memory: Function=%s, CodeCache=%s", 
         x->compiledFunction ? "allocated" : "null",
         x->lastSapfCode ? "cached" : "empty");
    
    post("sapf~: === END STATUS ===");
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
    int n = (int)sampleframes;

    // Thread-safe check for valid sapf audio
    bool hasValidAudio = false;
    ZIn audioExtractor; // Local copy for thread safety
    
    if (x && x->sapfThread) {
        // Briefly lock to copy audio state
        os_unfair_lock_lock(&x->audioStateLock);
        hasValidAudio = x->hasValidAudio;
        if (hasValidAudio) {
            audioExtractor = x->audioExtractor; // Copy the ZIn
        }
        os_unfair_lock_unlock(&x->audioStateLock);
    }
    
    if (hasValidAudio) {
        try {
            // Create temporary float buffer for ZIn::fill (which expects float*)
            // We'll then convert to Max's double format
            std::vector<float> floatBuffer(sampleframes);
            
            // Use sapf's audio extraction to fill temporary buffer with local copy
            bool audioAvailable = audioExtractor.fill(*x->sapfThread, n, floatBuffer.data(), 1);
            
            if (audioAvailable) {
                // Convert from float to double for Max's buffer
                for (int i = 0; i < (int)sampleframes; i++) {
                    outL[i] = (double)floatBuffer[i];
                }
            } else {
                // Audio stream ended - fill with silence
                for (int i = 0; i < (int)sampleframes; i++) {
                    outL[i] = 0.0;
                }
                // Thread-safely mark audio as no longer valid (stream ended)
                os_unfair_lock_lock(&x->audioStateLock);
                x->hasValidAudio = false;
                os_unfair_lock_unlock(&x->audioStateLock);
                // Note: Avoid posting in audio thread
            }
            
        } catch (const std::exception& e) {
            // Exception in audio thread - fill with silence and mark as failed
            for (int i = 0; i < (int)sampleframes; i++) {
                outL[i] = 0.0;
            }
            // Thread-safely mark audio as failed
            os_unfair_lock_lock(&x->audioStateLock);
            x->hasValidAudio = false;
            os_unfair_lock_unlock(&x->audioStateLock);
            // Note: Avoid posting in audio thread - handle error state in non-audio thread
        }
        
    } else {
        // No valid sapf audio - fall back to simple pass-through with offset
        // (This maintains backward compatibility and provides some output for testing)
        while (n--) {
            *outL++ = *inL++ + x->offset;
        }
    }
}

void sapf_help(t_sapf* x)
{
    post("sapf~: === SAPF LANGUAGE HELP ===");
    post("");
    
    post("sapf~ is a Max external that embeds the sapf language interpreter");
    post("SAPF (Sound As Pure Form) is a functional, stack-based audio programming language");
    post("");
    
    post("Basic Usage:");
    post("  Send 'code <expression>' messages to compile and execute sapf code");
    post("  Example: [code 440 0 sinosc 0.3 *(");
    post("");
    
    post("Common Commands:");
    post("  status  - Show VM status and current state");  
    post("  help    - Show this help message");
    post("");
    
    post("Basic Examples:");
    post("  440 0 sinosc                - 440Hz sine wave");
    post("  440 0 sinosc 0.3 *          - Sine wave at 30% volume");  
    post("  220 330 + 0 sinosc          - Math: (220+330)Hz sine wave");
    post("  440 0 sawtooth               - 440Hz sawtooth wave");
    post("  100 300 linterp 0 sinosc     - Linear interpolation between 100-300Hz");
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
    post("  • Stack underflow: Not enough arguments (try '440 0 sinosc' not 'sinosc')");
    post("  • Undefined symbol: Function name not found (check spelling)"); 
    post("  • Type error: Wrong argument type (number vs audio signal)");
    post("");
    
    post("For more info: Send 'status' to check VM state");
    post("sapf~ version with full sapf language interpreter embedded");
}
