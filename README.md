# 🛠️ RASPI5_BOOT: Researching the BCM2712 Booting Sequence

## 📖 Project Overview
This research project focuses on the **low-level initialization process of the Raspberry Pi 5 (BCM2712 SoC)**.  
The goal is to document and experiment with the transition from **hardware reset to operating system execution**, including the interaction between:

- SPI EEPROM bootloader
- Raspberry Pi firmware
- Secondary bootloaders such as **U-Boot**

This project is intended for **embedded systems and bootloader research**, helping developers understand the internal boot mechanism of the Raspberry Pi 5 platform.

---

## 🔍 Booting Mechanism Stages

The **Raspberry Pi 5** introduces a more standard **ARM-style boot flow** compared to previous generations.

| Stage | Component | Responsibility |
|------|-----------|---------------|
| **Stage 1** | SoC ROM | Executes immediately after power-on. Fetches the bootloader from the **on-board SPI EEPROM**. |
| **Stage 2** | EEPROM Bootloader | Initializes **LPDDR4X RAM** and scans boot media such as **SD, NVMe, or USB** for the firmware partition. |
| **Stage 3** | VideoCore Firmware | Executes `bootcode.bin`, loads configuration from `config.txt`, and prepares the **Device Tree**. |
| **Stage 4** | OS Loader | Transfers control to the **Linux Kernel (`kernel_2712.img`)** or a secondary bootloader such as **U-Boot**. |

---

## 🎯 Project Goals
- Understand the BCM2712 boot architecture
- Reverse engineer parts of the boot process
- Experiment with custom bootloaders
- Document the interaction between firmware and hardware initialization

## 🧰 Tools & Resources
- U-Boot
- Raspberry Pi Firmware
- UART debugging
- Linux kernel source
