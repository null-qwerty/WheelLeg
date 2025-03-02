# 轮腿步兵机器人电控代码仓库

这是一份轮腿机器人的电控代码，对应机器人相关硬件如下：

- 底盘控制芯片: 喵板(dm-mc02, stm32h723vgt6)
- 云台控制芯片: C 板(stm32f407igt6)
- 轮毂电机: RM3508
- 关节电机: Unitree A1
- 云台 yaw 轴电机: DM4310
- ...

代码使用 CubeMX + Cmake + VSCode + openocd 进行开发，使用 FreeRTOS 和以 C++ 实现的接口库 [phoenix-base-control](https://github.com/null-qwerty/phoenix-base-control) 作为中间件。

开发环境： Ububtu 24.04 LTS, gcc-arm-none-eabi, C++11
> 开发环境搭建可以参考[这篇博客](https://blog.null-qwerty.work/2024/10/03/ubuntu-%E4%B8%8B-CubeMX-cmake-gcc-arm-none-eabi-ozone-%E5%BC%80%E5%8F%91%E7%8E%AF%E5%A2%83%E6%90%AD%E5%BB%BA/)，感谢湖南大学跃鹿战队开源的环境搭建方案。

# 项目框架图

**不一定完全准确，但大概是这个意思**

箭头表示数据交换；上层应用以 FreeRTOS 的 Task 形式实现。

![](doc/layer.svg)