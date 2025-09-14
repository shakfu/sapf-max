

set(MANTA_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(MANTA_SOURCES
	${MANTA_SOURCE_DIR}/Manta.cpp
	${MANTA_SOURCE_DIR}/MantaMulti.cpp
	${MANTA_SOURCE_DIR}/MantaUSB.cpp
	${MANTA_SOURCE_DIR}/extern/hidapi/mac/hid.c
)

add_library(
	manta
	STATIC
	${MANTA_SOURCES}
)

target_include_directories(
	manta
	PUBLIC
	${CMAKE_SOURCE_DIR}/source/projects/sapf_tilde/include
	${CMAKE_CURRENT_SOURCE_DIR}
	${MANTA_SOURCE_DIR}/extern/hidapi/hidapi
)

target_link_libraries(
	manta
	PUBLIC
	"-framework IOKit"
	"-framework CoreFoundation"
)