# The Integrated Value Communication Protocol (IVCP)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)  
A MicroPython library implementing IVCP’s unidirectional, numeric messaging over RP2040 PIO.  
For full design rationale, protocol details, timing diagrams and integration guidance, see the **Technical Specification Exposition** in `IVCP-TSE-0725A.pdf`.
[![Docs: PDF](https://img.shields.io/badge/Docs–PDF-lightgrey.svg)](IVCP-TSE-0725A.pdf)

---

## Features

- **Stateless & Self-Syncing**  
  The receiver timing resets after every message, eliminating clock drift and enabling self-syncing behaviour.
- **Unidirectional Numeric Messaging**  
  Transmit floating-point values with up to 15 individual tagged identifiers at up to 100 Hz.
- **CRC-8 (0x07) Error Detection**  
  Every message is CRC checked, enabling receiver-side validation and peace of mind in motion control systems.
- **Return-to-Zero (RZ) Framing**  
  Inspired by my aviation background, a return-to-zero rule is implemented to detect issues at the electric level.
- **Configurable Timing**  
  T-period (in μs) is user configurable to meet a desired message rate. The T values on the Tx and Rx must match.
- **Minimal Dependencies**  
  Pure Python implementation—no external modules beyond MicroPython’s standard library, ready to be used through importing Tx or Rx from ivcp.py
- **Tiny Code Footprint**  
  Optimized for microcontrollers; fits easily in RP2040 PIO and similar MCUs.

---

## Installation

Simply save ivcp.py in your Raspberry Pi Pico and follow the usage instructions outlined in `IVCP-TSE-0725A.pdf`.
