# Vending Machine Project TODO List

**Last Updated:** September 21, 2025  
**Project:** Vending Machine Controller  
**Branch:** Casless_integration

## Overview

This document tracks pending tasks, missing implementations, and improvements needed across all modules in the vending machine project. Tasks are organized by module and priority level.

### Priority Levels
- 🔴 **Critical**: Blocking functionality, must be completed immediately
- 🟡 **High**: Important features, should be completed soon
- 🟢 **Medium**: Nice-to-have features, can be scheduled later
- 🔵 **Low**: Optimization or enhancement tasks

---

## 📡 ESP8266 WiFi Module

### Core Implementation
- [ ] 🔴 **Modify ESP8266 APIs implementation**
  - [ ] **Subscribe** API to be able to pass the topic as an argument
  - [ ] **publish** API to be able to pass the topic  and the value as arguments.

- [ ] 🟡 **Looger for the ESP module**
  - [ ] implement a main logger for the Whole system.

- [ ] 🟡 **Integration the ESP module**
  - [ ] integrate the **publish** process.
  - [ ] integrate the **subscribe** process.


### Testing & Validation
- [ ] 🔴 **Test the previous behavior of the ESP module after the refactor**

---

## 🔌 MDB (Multi-Drop Bus) Protocol Module

### Core Protocol Implementation
- [ ] 🔴 **Complete MDB Funcitonallity for Cashless device**
  - [ ] The subtraction after vending the drink
  - [ ] The fetching of the price info from the vending request
  

- [ ] 🟡 **Looger for the MDB module**
  - [ ] implement a main logger for the Whole system.

- [ ] 🟡 **Error handler for the MDB module**
  - [ ] implement an Error handler to react one there is a violation.

### Testing & Validation
- [ ] 🔴 **Test the system with the real HW**

---

## ⚙️ System Configuration & Management

### Configuration System
- [ ] 🟡 **Implement configuration persistence**
  - [ ] Flash/EEPROM storage
  - [ ] Configuration backup/restore
  - [ ] Factory reset functionality
  - [ ] Configuration versioning

### Testing & Validation
- [ ] 🟡 **MDB protocol compliance testing**
- [ ] 🟡 **Hardware-in-loop testing**
- [ ] 🟢 **Stress testing with multiple devices**
- [ ] 🟢 **EMI/EMC compliance verification**
---

## 🔄 FreeRTOS Integration & Task Management

### Task Implementation
- [ ] 🔴 **Define all required tasks**
  - [ ] MDB communication task
  - [ ] ESP8266 WiFi task
  - [ ] User interface task
  - [ ] System monitoring task

### Testing & Validation
- [ ] 🟡 **MDB protocol compliance testing**
- [ ] 🟡 **Hardware-in-loop testing**
- [ ] 🟢 **Stress testing with multiple devices**
- [ ] 🟢 **EMI/EMC compliance verification**
---
---

## 🖥️ User Interface & Display

### Core UI Implementation
- [ ] 🔴 **Basic display functionality**
  - [ ] LCD/OLED driver implementation
  - [ ] Menu system structure
  - [ ] User input handling
  - [ ] Status indicators

---

## 🔧 Hardware Abstraction & Drivers

### Hardware Drivers
- [ ] 🔴 **Power consumption and distribution**
  - [ ] Discuss the power of
    - [ ] Contoller
    - [ ] ESP8266
    - [ ] MIFI device


---

## 📚 Documentation & Training

### Technical Documentation
- [ ] 🟡 **API documentation completion**
  - [ ] Function documentation
  - [ ] Parameter descriptions
  - [ ] Return value documentation
  - [ ] Usage examples

- [ ] 🟡 **System architecture documentation**
  - [ ] Module interaction diagrams
  - [ ] Data flow documentation
  - [ ] State machine diagrams
  - [ ] Timing requirements

### User Documentation
- [ ] 🟢 **Operation manual**
  - [ ] Installation procedures
  - [ ] Configuration guide
  - [ ] Troubleshooting guide
  - [ ] Maintenance procedures

- [ ] 🟢 **Training materials**
  - [ ] Video tutorials
  - [ ] Quick start guides
  - [ ] Best practices documentation
  - [ ] FAQ compilation

---

## 📝 Notes

### Current Status Summary
- **ESP8266 Module**: Configuration system implemented, missing core AT command functionality
- **MDB Module**: Basic structure in place, missing complete protocol implementation
- **System Configuration**: PBCFG pattern implemented, missing persistence layer
- **FreeRTOS Integration**: Basic task structure defined, missing inter-task communication
- **Documentation**: Examples converted to markdown, API documentation incomplete
