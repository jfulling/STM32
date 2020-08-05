# STM32 SmartWatch #
## Summary ##

**Goal:** Create a wearable smartwatch with wireless analysis tools.

**Context:** This project is a way for me to improve my C programming, understanding of how operating systems work, and dive into electrical engineering and microcontrollers.

## Milestone Checklist ##

Milestone | Status | Extra
----------|--------|------
Learn SPI integration using NRF24L01 | Complete |
Implement [uC_mousejack](https://github.com/insecurityofthings/uC_mousejack) and modify the NRF24 library | Complete |
Utilize FreeRTOS hooks to register uC_mousejack library into a parallel task | Complete | 
Bluetooth Integration for custom control | Not Started | Pending GUI completion, BLE module acquisition, and basic android app for control
Portable In-System Programmer | Not Started |
Wireless spectrum analyzer (sub-1GHz) | Not Started|
Low-Power AM Transmitter| Not Started
Low-Power FM Transmitter| Not Started

## Hardware ##
* STM32L4R9 DISCO board (STM32L4R9AI)
* NRF24L01

## References ##
* Credit to [MYaqoobEmbedded](https://github.com/MYaqoobEmbedded/STM32-Tutorials/tree/master/Tutorial%2024%20-%20NRF24L01%20Radio%20Transceiver) for base NRF24 Library

* Credit to insecurityofthings (https://github.com/insecurityofthings/uC_mousejack) for the uC mousejack framework.

This repo will be replaced with an updated repo once GUI draft is completed and FreeRTOS hooks are implemented.
