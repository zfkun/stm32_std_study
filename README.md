# STM32 测试学习工程

测试及学习目的, 记录为主, 方便其他项目快速移植和代码复用

> STM32 的学习测试, 相比没人会不接触神一样存在的 江协科技 把, 项目代码基本是基于江协的代码基础之上借鉴需改优化而来

> 工程配置默认按 `C6T6` 配置的, 其他芯片适当修改编译配置即可

> 比如, 用常见的 `C8T6` 的话, 重新eide导入下, 对应修改配置即可

## 目录结构

- core
  - bsp         板级代码
  - hardward    硬件代码
  - user        用户代码
- doc           文档
- driver        驱动库
  - CMSIS                       ARM Cortex-M3/M4标准库
  - STM32F10x_StdPeriph_Driver  STM32F10x标准外设库
- project       IDE工程
  - MDK(V5)     Keil工程

## IDE

- [Keil](https://www.keil.com/)

> 上古神器, 偶尔用还是可以的

- [CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)

> 官方IDE, 也是可以用用的

- [Clion](https://www.jetbrains.com/clion/)

> Jetbrains 出品无需多说 (个人用户早已免费)

- [VScode] + [Cmake] + [GCC] + [Ninja]

> 轻量高效, 跨平台支持也很不错

- [VSCode](https://code.visualstudio.com/) + [EIDE](https://em-ide.com/docs/intro/)

> 轻量高效, 也挺省事
> 仿真调试器用 ST-Link (或 DAP-Link, WCH-Link, 自制DIY) 都行

- 等等 (多年来, 开发方式演变了多种多样, 晚些时候单独发篇文章介绍吧)
