set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # Optional: Disables compiler extensions beyond the standard

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

add_subdirectory(libmanta)

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(CPP_SOURCES
	${SOURCE_DIR}/CoreOps.cpp
	${SOURCE_DIR}/DelayUGens.cpp
	${SOURCE_DIR}/dsp.cpp
	${SOURCE_DIR}/elapsedTime.cpp
	${SOURCE_DIR}/ErrorCodes.cpp
	${SOURCE_DIR}/FilterUGens.cpp
	# ${SOURCE_DIR}/main.cpp
	${SOURCE_DIR}/MathFuns.cpp
	${SOURCE_DIR}/MathOps.cpp
	${SOURCE_DIR}/Midi.cpp
	${SOURCE_DIR}/MultichannelExpansion.cpp
	${SOURCE_DIR}/Object.cpp
	${SOURCE_DIR}/Opcode.cpp
	${SOURCE_DIR}/OscilUGens.cpp
	${SOURCE_DIR}/Parser.cpp
	${SOURCE_DIR}/Play.cpp
	${SOURCE_DIR}/primes.cpp
	${SOURCE_DIR}/RandomOps.cpp
	${SOURCE_DIR}/RCObj.cpp
	${SOURCE_DIR}/SetOps.cpp
	${SOURCE_DIR}/SoundFiles.cpp
	${SOURCE_DIR}/Spectrogram.cpp
	${SOURCE_DIR}/StreamOps.cpp
	${SOURCE_DIR}/symbol.cpp
	${SOURCE_DIR}/Types.cpp
	${SOURCE_DIR}/UGen.cpp
	${SOURCE_DIR}/VM.cpp
)

set(OBJC_SOURCES
	${SOURCE_DIR}/makeImage.mm
)

add_library(
	sapf
	STATIC
	${CPP_SOURCES}
	${OBJC_SOURCES}
)

target_compile_options(
	sapf
	PUBLIC
	-Wmissing-field-initializers
	-Wmissing-prototypes
	-Wreturn-type
	-Wparentheses
	-Wpedantic
	-Wno-pedantic
	-Wshadow
	-Wsign-compare
	-Wuninitialized
	-Wno-unused-function
	-Wunused-label
	-Wunused-value
	-Wunused-variable
)

target_link_libraries(
	sapf
	PUBLIC
	manta
	-ledit
	"-framework Accelerate"
	"-framework AudioToolbox"
	"-framework AudioUnit"
	"-framework Carbon"
	"-framework Cocoa"
	"-framework CoreFoundation"
	"-framework CoreMidi"
	"-framework CoreServices"
)

target_include_directories(
	sapf
	PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
)
