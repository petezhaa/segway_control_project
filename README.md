# 🚀 Segway Balance Control System (FPGA | Verilog)

Real-time self-balancing Segway-style control system implemented in **SystemVerilog** and deployed on an **FPGA**.  
This project integrates **PID control**, **SPI & UART interfaces**, **inertial sensing**, and **motor PWM drive** into a unified embedded control pipeline.

---

## ⭐ Features

### 🔧 Real-Time Balance Controller
- Discrete **PID controller** for pitch stabilization  
- Soft-start logic, integrator reset, saturation protection  
- Independent left/right motor control based on pitch & pitch-rate

### 🏎 Inertial Sensor Interface (SPI)
- Custom **SPI master** interface to IMU  
- Captures pitch, pitch rate, and data-ready signal  
- Hardware pipeline for deterministic timing

### ⚡ A2D Current Sensing
- SPI-based ADC interface for real-time motor current monitoring  
- Overcurrent detection + safety shutoff logic

### 💬 UART Module
- UART RX for Bluetooth / serial debugging  
- Allows real-time tuning of control parameters

### 🔋 Motor PWM Driver
- Dual-channel PWM generation  
- Supports dead-time, saturation, and left/right differential control



