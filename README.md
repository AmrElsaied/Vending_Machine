# Vending Machine Controller (VMC)

## Objective

Enable seamless communication with the Vending Machine Controller
regardless of the payment device type.

Supports multiple peripherals including:
- Bill validators
- Cashless payment devices

## Overview

This project implements a comprehensive embedded systems solution for interfacing with Vending Machine Controllers (VMC) across multiple payment device types, including bill validators and cashless payment systems. Built on FreeRTOS with support for STM32F1 and STM32F4 microcontrollers, the VMC provides a unified, device-agnostic communication framework using the MDB protocol standard. MQTT connectivity enables IoT-based remote monitoring and control of vending machines.

## Key Features

- **Device-Agnostic Communication**: 
  Unified MDB protocol implementation supporting multiple device types

- **Cashless Payment Integration**: 
  Full compatibility with standard cashless devices

- **Bill Validator Support**: 
  Full compatibility with standard bill acceptance devices

- **Real-Time Operating System**: 
  FreeRTOS-based task management for concurrent operations

- **Dual Microcontroller Support**: 
  STM32F1 (STM32F103C8T6) with core functionality and STM32F4 (STM32F401RCT6) with full feature support due to enhanced RAM and processing capabilities

- **State Machine Architecture**: 
  Robust handling of device states and protocol sequences

- **Modular Design**: 
  Clean separation of concerns with dedicated handler modules

## Repository Structure

- **`Compatable_WorkSpace/`** 
STM32F1 project workspace with core MDB protocol implementation (limited by RAM constraints; see STM32F4_Workspace for full-featured variant)

- **`STM32F4_Workspace/`** 
  STM32F4-based variant with enhanced features

- **`Essentials_Scripts/`** 
  Python utilities for serial monitoring and firmware analysis

- **`V1/`** 
  The code that was the start point od development

## Quick Start
## Quick Start

### STM32F1 (Compact Version)
See [Compatable_WorkSpace/README.md](Compatable_WorkSpace/README.md) for detailed setup, build, and deployment instructions.

### STM32F4 (Full-Featured Version)
See [STM32F4_Workspace/README.md](STM32F4_Workspace/README.md) for detailed setup, build, and deployment instructions.

## Documentation

- [Architecture Overview](STM32F4_Workspace/docs/architecture.md)
- [MDB Protocol Reference](STM32F4_Workspace/docs/mdb-protocol.md)
- [Configuration Guide](STM32F4_Workspace/docs/configuration.md)
- [API Reference](STM32F4_Workspace/docs/api-reference.md)
- [Coding Standards](STM32F4_Workspace/docs/coding-standards.md)

> **Note**: Complete documentation is available in the STM32F4 version. The Compatable_WorkSpace variant has limited documentation due to resource constraints.

## Hardware Requirements

- **Microcontroller (Primary)**: 
    STM32F4 (STM32F401RCT6) with extended capabilities

- **Microcontroller (Alternative)**: 
    STM32F1 (STM32F103C8T6)

## Interfaces

- **Communication**: 
  UART for MDB
  uart for debugging
