# 📘 LPC17xx CMSISv2p00 Educational Refactor

Improved and documented version of the legacy `CMSISv2p00_LPC17xx` driver library  
**Used in the course _Electrónica Digital 3_ as a mid-semester support package**

---

## 🎯 Purpose

This repository is part of an educational effort in the course **Electrónica Digital 3**, where students begin the semester writing firmware directly with **register-level programming**, and midway are introduced to a **driver-based abstraction**.

This project is a **light refactor and documentation initiative** on top of the original `CMSISv2p00_LPC17xx` package, aiming to:

- 🧹 Improve code readability and structure
- 📚 Add comments and usage documentation
- 🛠️ Fix minor issues and inconsistencies
- ➕ Add missing but useful helper functions
- 🧑‍🏫 Provide a clean base for students to build on

---

## 🚩 Project Scope

This is **not** a complete rewrite. Changes are kept minimal and targeted:

| Area                | Change type                    | Status         |
|---------------------|---------------------------------|----------------|
| `lpc17xx_pinsel.c`  |   | 🔲 Planned      |
| `lpc17xx_gpio.c`    |        | 🔲 Planned  |
| `lpc17xx_timer.c`   |  | 🔲 Planned  |
| `lpc17xx_uart.c`    |       | 🔲 Planned      |
| `lpc17xx_systick.c` |     | 🔲 Planned  |
| `lpc17xx_nvic.c`    |     | 🔲 Planned  |
| `lpc17xx_adc.c`     |     | 🔲 Planned  |

Pull requests and contributions are welcome if consistent with the educational goal and simplicity of the project.

---

## 🧪 Example usage

```c
```
## 🕘 Versioning and History
- Initial commit: exact copy of the legacy CMSISv2p00_LPC17xx codebase (preserved for diffing)

- Subsequent commits: light refactors, added documentation, helper functions

## 🧠 For Students
When using this library, you no longer need to write directly to registers like GPIOx->FIOSET, FIOCLR, or set PINSEL bits manually. Instead, use the provided functions such as:

- GPIO_SetDir()

- GPIO_SetValue()

- GPIO_Toggle()

- UART_Init()

- TIMER_Start()

These simplify your code, reduce errors, and help focus on application logic.
## 📜 License
The base code is from NXP's CMSISv2p00_LPC17xx, under BSD-3-Clause license.
All additions and changes in this repository also follow the BSD-3-Clause license.
## 👨‍💻 Developed By
[David Trujillo Medina](https://github.com/David-A-T-M)  
Feel free to use this as a learning resource or base for embedded firmware development on LPC17xx.
