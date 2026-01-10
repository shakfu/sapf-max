set(ENGINE_SOURCES
	MidiBackend.cpp
	MidiRouter.cpp
	AudioBackend.cpp
	CoreOps.cpp
	DelayUGens.cpp
	dsp.cpp
	elapsedTime.cpp
	ErrorCodes.cpp
	FilterUGens.cpp
	MathFuns.cpp
	MathOps.cpp
	Midi.cpp
	backends/MaxAudioBackend.cpp
	backends/AlsaAudioBackend.cpp
	backends/CoreMidiBackend.cpp
	backends/NullMidiBackend.cpp
	MultichannelExpansion.cpp
	Object.cpp
	Opcode.cpp
	OscilUGens.cpp
	Parser.cpp
	backends/CoreAudioBackend.cpp
	backends/NullAudioBackend.cpp
	primes.cpp
	RandomOps.cpp
	RCObj.cpp
	SetOps.cpp
	SoundFiles.cpp
	Spectrogram.cpp
	SapfEngine.cpp
	StreamOps.cpp
	symbol.cpp
	Types.cpp
	UGen.cpp
	VM.cpp
)

# Platform-specific sources
if(APPLE)
	list(APPEND ENGINE_SOURCES makeImage.mm)
else()
	list(APPEND ENGINE_SOURCES makeImageStub.cpp)
endif()

add_library(
	sapf_engine
	STATIC
	${ENGINE_SOURCES}
)

target_include_directories(
	sapf_engine
	PUBLIC
	${SAPF_ROOT}/include
)

# Compiler-specific warning flags
if(MSVC)
	target_compile_options(
		sapf_engine
		PUBLIC
		/W3
	)
else()
	target_compile_options(
		sapf_engine
		PUBLIC
		-Wmissing-field-initializers
		-Wreturn-type
		-Wparentheses
		-Wshadow
		-Wsign-compare
		-Wuninitialized
		-Wno-unused-function
		-Wunused-label
		-Wunused-value
		-Wunused-variable
	)
endif()

# Platform-specific libraries
if(APPLE)
	target_link_libraries(
		sapf_engine
		PUBLIC
		"-framework Accelerate"
		"-framework AudioToolbox"
		"-framework AudioUnit"
		"-framework Carbon"
		"-framework Cocoa"
		"-framework CoreFoundation"
		"-framework CoreMidi"
		"-framework CoreServices"
		-ledit
	)
endif()

set_target_properties(
	sapf_engine
	PROPERTIES
	OUTPUT_NAME "sapf"
	ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
	LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)
