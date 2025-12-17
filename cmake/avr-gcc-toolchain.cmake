# AVR GCC Toolchain File

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR avr)

# Path to your AVR toolchain
set(AVR_GCC_PATH "C:/AVR/avr8-gnu-toolchain/bin")

set(CMAKE_C_COMPILER "${AVR_GCC_PATH}/avr-gcc.exe")
set(CMAKE_CXX_COMPILER "${AVR_GCC_PATH}/avr-g++.exe")
set(CMAKE_AR "${AVR_GCC_PATH}/avr-ar.exe")
set(CMAKE_OBJCOPY "${AVR_GCC_PATH}/avr-objcopy.exe")
set(CMAKE_OBJDUMP "${AVR_GCC_PATH}/avr-objdump.exe")
set(CMAKE_SIZE "${AVR_GCC_PATH}/avr-size.exe")

# Avoid trying to run executables
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
