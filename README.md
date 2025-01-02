

# Smart Washing Machine Controller

A DIY IoT project that transforms a standard washing machine into a smart, Alexa-controlled appliance with real-time Telegram feedback.

## Features
- **Alexa Integration**: Voice commands for hands-free control.
- **Remote Monitoring**: Updates and alerts via Telegram.
- **Motor Control**: Inverter drive for precise speed and direction control.
- **User Interface**: Simple pushbuttons, LEDs, and a 16x2 display.

## Highlights
- ESP32-based modular PCB for all control operations.
- Programmed wash cycles for automation.
- Easily customizable for different machine models.

## How It Works
The ESP32 handles user inputs, motor operations, and wireless communication. Additional components like relays, buck converters, and an inverter drive ensure smooth functionality.

For detailed assembly, schematics, and programming instructions, visit the project page: [Alexa Washing Machine](https://atharvap8.github.io/posts/2024/06/alexa-washing-machine/).


## Folder contents

The project contains one source file in C++ language [main.cpp](main/main.cpp). The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   └── main.cpp
└── README.md                  This is the file you are currently reading
```
Additionally, the project contains Makefile and component.mk files, used for the legacy Make based build system. 
They are not used or needed when building with CMake and idf.py.
