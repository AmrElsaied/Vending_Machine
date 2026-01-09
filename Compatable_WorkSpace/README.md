# Vending Machine Controller (VMC) Project

## Overview
This project implements a Vending Machine Controller (VMC) system using STM32F103C8T6 microcontroller with FreeRTOS. The system handles MDB (Multi-Drop Bus) protocol communication for managing cashless payment devices and peripheral control.

## Table of Contents
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Architecture](#software-architecture)
- [Getting Started](#getting-started)
- [Documentation](#documentation)
- [Contributing](#contributing)
- [License](#license)

## Features
- **MDB Protocol Support**: Full implementation of Multi-Drop Bus protocol for vending machine peripherals
- **Cashless Payment Integration**: Support for card readers and cashless payment systems
- **Real-time Processing**: FreeRTOS-based task management for real-time operations
- **State Machine Management**: Robust state handling for different operational modes
- **USB Communication**: USB CDC interface for monitoring
- **Modular Design**: Clean, maintainable code structure with proper separation of concerns

## Hardware Requirements
- **Microcontroller**: STM32F103C8T6 (Blue Pill board compatible)
- **Communication**: 
  - UART1 for MDB communication
  - USB for configuration interface
- **Power Supply**: 5V/12V for MDB bus compatibility

## Software Architecture

### Core Modules
1. **MDB_Handler**: Manages MDB protocol communication
2. **System_Tasks**: FreeRTOS task management and scheduling
3. **VMC_Config**: Configuration and command definitions
4. **USB_Device**: USB CDC communication interface


## Getting Started

### Prerequisites
- STM32CubeIDE or compatible development environment
- STM32CubeMX for configuration
- ST-Link programmer/debugger

### Building the Project
1. Clone the repository
2. Open `VMC_Simulator.ioc` in STM32CubeMX
3. Generate code if needed
4. Build using STM32CubeIDE
5. Flash to target hardware

### Quick Start
```c
// Initialize MDB bus
MDB_BusInit();

// Create system tasks
System_TaskCreate();

// Start FreeRTOS scheduler
vTaskStartScheduler();
```

## Documentation
Detailed documentation is available in the `/docs` directory:

- [Architecture Overview](docs/architecture.md)
- [API Reference](docs/api-reference.md)
- [Configuration Guide](docs/configuration.md)
- [State Machine Documentation](docs/state-machines.md)
- [Troubleshooting Guide](docs/troubleshooting.md)
- [Coding Standards](docs/coding-standards.md)
- [Testing Guide](docs/testing-guide.md)

## Project Structure
```
Compatable_WorkSpace/
├── Core/
│   ├── Inc/          # Header files
│   ├── Src/          # Source files
│   └── Startup/      # Startup files
├── Drivers/          # HAL drivers
├── Middlewares/      # Third-party middleware
├── USB_DEVICE/       # USB device implementation
├── Debug/           # Debug build files
└── docs/            # Project documentation
```

## 👥 Authors

- **Amr Elsaied** - *Initial development and architecture* - [AmrElsaied](https://github.com/AmrElsaied)


## 📈 Version History

- **v1.2.0** (2025-09-02): Comprehensive documentation, testing framework, and contribution guidelines
- **v1.1.0** (2025-08-15): Enhanced state management and cashless integration
- **v1.0.0** (2025-08-08): Initial release with basic MDB functionality

---