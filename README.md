---
layout: doc
---

# 环境搭建

南京沁恒的芯片大家应该都比较熟悉，特别是USB转串口，国内、国外市场占有率很大。 沁恒在国内USB接口领域做到了邻头羊的地位，在RISC- V也有较早的布局。很早就想玩玩他家的板子，手上也有一块他家的开发版，但因其提供的工具链主要是windows下，官方提供的开发IDE 为MounRiver Studio 目前仅支持windows和LINUX。他们似乎忘记了，还有一些选手工作在MAC上所以一直放下了。但后来在 youtube 看到歪果仁基于CH32X035做的两个项目感觉挺有趣，随也有了自己做一个板子的想法。本文主要讲解在 Mac下 CH32 Risc-v开发环境的搭建。

## Youtube Video
>  
  - https://youtu.be/wqLiRnbcISo?si=fp_JowhAQWmYBOSz
  - https://youtu.be/UPd5qPuhOCs?si=8_eOXN0sKj8t5daM
  - https://www.youtube.com/watch?v=wqLiRnbcISo&pp=ygUIQ0gzMlgwMzU%3D



## CH32X035介绍

性价比很高的CH32X035，
![CH32X035](https://img.wch.cn/20240312/e7da1f2a-ed64-43a3-a9dc-8c72fe8b519f.jpg)

![PCB](/doc/x035_3d.png)
![PCB](/doc/x035_real.png)

  

## 如何编译

## MacOS 工具链下载

[Toolchain]http://file-oss.mounriver.com/tools/MRS_Toolchain_MAC_V191.zip

解压后有两部分
- gnu toolchain
- openocd （wch魔改后的）

## 系统环境变量
> 将工具链的执行目录添加到系统环境变量中
```shell
# 以zsh shell为例:添加 ～/.zshrc 

export PATH=/Users/bright/.toolchain/xpack-riscv-none-embed-gcc-8.2.0/bin:$PATH
export PATH=/opt/openocd/bin:$PATH
```

## 根目录下 CMakeLists.txt 设置编译器参数

```shell
SET(TOOLCHAIN_DIR /Users/bright/.toolchain/xpack-riscv-none-embed-gcc-8.2.0)
SET(TARGET_TRIPLET "riscv-none-embed")

```

### - clone 原代码
  ```shell
  git clone https://github.com/ppvision/CH32X035.git
  cd CH32X035
  
  ```

###  - 编译
```shell
  git submodule update --init
  mkdir build
  cd build
  cmake ..
  make gpio
```

### - 命令行烧录
```shell
  ../flash.sh Apps/gpio/gpio.bin

```

> flash.sh
```shell
#!/bin/bash

# my dev : 
usb_pdev=0x01120000
firmware=""

if [ $# -lt 1 ]; then
    echo "需要至少1个参数"
    echo "用法: firmware"
    exit 1
fi

if [ $# -eq 2 ]; then
    usb_pdev=$1
    firmware=$2
fi

if [ $# -eq 1 ]; then
    firmware=$1
fi

script_path=$(dirname "$0")


WCHISPTool_CMD -p $usb_pdev -c $script_path/CH32V035.INIT -o program -v boot -f $firmware 

```

> usb_pdev的取值可以在系统信息里查看：
![USB dev](/doc/mac_usb_dev.png)


### 调试

- 打开工程

> 项目是CMake组织的，用CLion打开项目根目录即可
- CLion


> 创建 Clion的调试配置项
![CLion Debugger](/doc/clion_debug.png)


> 调试 
![CLion Debugger1](/doc/clion_debug01.png)






