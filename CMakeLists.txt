cmake_minimum_required(VERSION 3.15)

# Tell CMake to use our AVR toolchain
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/avr-gcc-toolchain.cmake")

project(AVRProject C CXX)

# Specify MCU config
set(MCU atmega2560)
set(F_CPU 16000000UL)

# Compiler flags
add_compile_options(
    -mmcu=${MCU}
    -DF_CPU=${F_CPU}
    -Os
)

# Linker flags
add_link_options(
    -mmcu=${MCU}
)

# Source files
file(GLOB SRC
    "${CMAKE_SOURCE_DIR}/src/*.c"
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/include/*.c"
    "${CMAKE_SOURCE_DIR}/include/*.cpp"
    "${CMAKE_SOURCE_DIR}/fatFs/*.c"
    "${CMAKE_SOURCE_DIR}/fatFs/*.cpp"
)

include_directories(include)
include_directories(fatFs)

list(REMOVE_ITEM SRC "${CMAKE_SOURCE_DIR}/include/st7920_graphics.cpp")

add_executable(${PROJECT_NAME}.elf ${SRC})

# Create HEX
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf
)
