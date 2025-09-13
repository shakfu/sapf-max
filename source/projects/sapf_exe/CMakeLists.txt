
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # Optional: Disables compiler extensions beyond the standard

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

add_executable(sapf_exe
	${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

target_include_directories(
	sapf_exe
	PUBLIC
	${CMAKE_SOURCE_DIR}/source/projects/sapf_lib/include
)

target_link_libraries(
	sapf_exe
	PUBLIC
	sapf
)

set_target_properties(sapf_exe PROPERTIES OUTPUT_NAME "sapf")
