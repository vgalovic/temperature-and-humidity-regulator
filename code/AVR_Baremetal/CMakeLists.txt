cmake_minimum_required(VERSION 3.10)

# Set the project name
project(AVR_Baremetal)

# Enable export of compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# AVR include paths for clangd
set(AVR_INCLUDE_DIRS
    "/usr/lib/avr/include"
    "/usr/lib/gcc/avr/7.3.0/include"
)

# Add them as system includes (clangd-friendly)
foreach(dir IN LISTS AVR_INCLUDE_DIRS)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -isystem ${dir}")
endforeach()

# Set targeted microcontroller
set(AVR_MCU atmega328p)
set(AVRDUD_MCU ATMEGA328P)

# Set the flash port and baud rate
set(PORT "/dev/ttyACM0")
set(DF_CPU "16000000UL")

# Set bit rate
set(BPS 115200)

# Set the compiler and related tools
set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_OBJCOPY avr-objcopy)
set(CMAKE_SIZE avr-size)
set(CMAKE_C_FLAGS "-mmcu=${AVR_MCU} -Os -DF_CPU=${DF_CPU} -Wall")

# Automatically gather all .c files in the src directory
file(GLOB_RECURSE SOURCES "src/*.c")

# Specify include directories
include_directories(include)

# Add executable (this creates the .elf file)
add_executable(${PROJECT_NAME}.elf ${SOURCES})

# Set linker flags
set_target_properties(${PROJECT_NAME}.elf PROPERTIES
    LINK_FLAGS "-mmcu=${AVR_MCU}"
)

# Convert the ELF file to HEX after build
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMENT "Generating HEX file"
)

# Copy compile_commands.json to the project root
add_custom_command(
    TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_BINARY_DIR}/compile_commands.json
            ${CMAKE_SOURCE_DIR}/compile_commands.json
    COMMENT "Copying compile_commands.json to project root"
)

# Flashing the microcontroller after build
add_custom_target(program
    COMMAND avrdude -c arduino -p ${AVRDUD_MCU} -P ${PORT} -b ${BPS} -U flash:w:${PROJECT_NAME}.hex:i
    DEPENDS ${PROJECT_NAME}.hex
    COMMENT "Programming AVR microcontroller"
)

# Post-build: show memory usage using avr-size
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND avr-size -C --mcu=${AVR_MCU} ${PROJECT_NAME}.elf
    COMMENT "Printing memory usage"
)

add_custom_target(clean-all
    COMMAND ${CMAKE_COMMAND} -E rm -f
        ${PROJECT_NAME}.elf
        ${PROJECT_NAME}.hex
        ${PROJECT_NAME}.map
    COMMENT "Cleaning build artifacts"
)
