include(${CMAKE_CURRENT_SOURCE_DIR}/../../max-sdk-base/script/max-pretarget.cmake)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(
	"${MAX_SDK_INCLUDES}"
	"${MAX_SDK_MSP_INCLUDES}"
	"${MAX_SDK_JIT_INCLUDES}"
)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Build the local SAPF library
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../sapf ${CMAKE_CURRENT_BINARY_DIR}/sapf)

# Create the Max external
add_library(
	${PROJECT_NAME}
	MODULE
	sapf_tilde.cpp
)

target_compile_options(
	${PROJECT_NAME}
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

# Include directories from the SAPF library
target_include_directories(
	${PROJECT_NAME}
	PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/../sapf/include
)

target_compile_definitions(
	${PROJECT_NAME}
	PUBLIC
	SAPF_TILDE
)

# Link against the SAPF engine library and system frameworks
target_link_libraries(
	${PROJECT_NAME}
	PUBLIC
	sapf_engine
	-ledit
	"-framework Accelerate"
	"-framework AudioToolbox"
	"-framework AudioUnit"
	"-framework Carbon"
	"-framework Cocoa"
	"-framework CoreFoundation"
	"-framework CoreMidi"
	"-framework CoreServices"
	"-framework CoreAudio"
)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../max-sdk-base/script/max-posttarget.cmake)
