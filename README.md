# 📘 LPC17xx CMSISv2p00 Modernized Driver Library

A modern, well-documented, and thoroughly tested enhancement of the original `CMSISv2p00_LPC17xx` driver library for
LPC176x/5x microcontrollers.

---

## 🎯 Project Overview

This project aims to bring new life to the abandoned `CMSISv2p00_LPC17xx` driver library by:

- ✍️ Providing professional-grade documentation using **Doxygen**
- 🔧 Refactoring and extending existing driver **implementations**
- 📐 Standardizing **naming conventions** for consistency across modules
- 🧪 Introducing a **unit testing framework** for all functions (legacy, new, and modified)
- 🔄 Maintaining a **modern GitHub workflow** with issues, branches, and pull requests

Initially built for educational purposes in the course _Digital Electronics 3_ at FCEFyN, this project also serves as a
robust, production-ready foundation for embedded development on the LPC17xx platform.

---

## 🚀 Features

- **Modular and Consistent Drivers:** Improved APIs with reliable, easy-to-read code and consistent behavior across all
  modules.
- **Complete Test Coverage:** Each function is rigorously tested, including legacy code, with a minimal, self-contained
  test framework designed for on-board execution.
- **Doxygen Integration:** All modules are documented to support both learning and maintainability.
- **Developer-Friendly:** Naming schemes, macros, and defines follow strict conventions for clarity and scalability.
- **Actively Maintained:** Development is tracked through GitHub using collaborative tools such as issues, discussions,
  and pull requests.

---

## 📦 Module Status

| Module    | Refactored  | Documented  |   Tested    |
|-----------|:-----------:|:-----------:|:-----------:|
| `PINSEL`  | ✅ Completed | ✅ Completed | ✅ Completed |
| `GPIO`    | ✅ Completed | ✅ Completed | ✅ Completed |
| `EINT`    | ✅ Completed | ✅ Completed | ✅ Completed |
| `SysTick` | ✅ Completed | ✅ Completed | ✅ Completed |
| `Timer`   | ✅ Completed | ✅ Completed | ✅ Completed |
| `PWM`     | ✅ Completed | ✅ Completed | ✅ Completed |
| `ADC`     | ✅ Completed | ✅ Completed | ✅ Completed |
| `DAC`     | ✅ Completed | ✅ Completed | ✅ Completed |
| `GPDMA`   | ✅ Completed | ✅ Completed | ✅ Completed |
| `UART`    | ✅ Completed | ✅ Completed | ✅ Completed |

_This table is continuously updated as development progresses._

---

## 🧪 Testing Framework

The project includes a lightweight, modular test framework:

- Designed to run directly on the target hardware
- Each function is tested individually in isolation
- Encourages test-driven development and safe contributions

---

## 🎓 Educational Context

In _Digital Electronics 3_, students:

- Start with low-level register programming on the LPC1769 (e.g., GPIO, PINSEL)
- Later transition to abstraction through this driver library
- Contribute to the library itself, learning collaborative software development

This project ensures the library is not only a usable HAL but also a **maintainable and extendable codebase** for
students and professionals alike.

---

## 🔗 Companion Repository

A [companion repository](https://github.com/David-A-T-M/DigitalElectronics3_2025) is being developed alongside this one,
providing:

- 🔲 Example projects at **register-level** and using the **modern HAL**
- 📚 Guides and working examples for the **LPC1769** and **STM32 Blue Pill**
- 🔁 Migration paths between LPC and STM32, encouraging platform independence

The goal is to give students a clear, practical guide for real-world development across architectures.

---

## 📝 License

The original library is © NXP and licensed under BSD-3-Clause.  
All modifications, enhancements, and new modules in this repository are also released under the **BSD-3-Clause License
**.

---

## 👤 Author

**[David Trujillo Medina](https://github.com/David-A-T-M)**

Embedded systems educator and developer.  
This project is part of my long-term goal to modernize educational tools while building production-level embedded
software infrastructure.

---

## 🤝 Contributions

Contributions are welcome!

If you wish to propose a change:

1. Fork the repository
2. Create a branch with your feature or fix
3. Submit a pull request with:
    - Code aligned with the style and conventions
    - Doxygen documentation (if applicable)
    - Corresponding unit tests

Open issues or discussions for suggestions and ideas — let’s make embedded development better together.

---
