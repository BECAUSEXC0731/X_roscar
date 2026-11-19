# 重建 micro-ROS 库（改 MTU）踩坑记录

`custom.meta` 把自定义传输 MTU 提到 2048（为了发布 360 点 LaserScan）。
**改完必须重建库**：构建脚本发现 `libmicroros/` 已存在会直接跳过，不会重新生成。

```bash
rm -rf .pio/libdeps/xc/micro_ros_platformio/libmicroros
~/.platformio/penv/bin/platformio run          # 会用 custom.meta 重新构建

# 验证是否生效（应为 2048）
grep UXR_CONFIG_CUSTOM_TRANSPORT_MTU \
  .pio/libdeps/xc/micro_ros_platformio/libmicroros/include/uxr/client/config.h
```

重建过程中遇到并修掉的 3 个环境问题（补丁都在 `.pio/libdeps/xc/micro_ros_platformio/` 里，
**重装/更新 micro_ros_platformio 后会丢失，需要按下面重新打**）：

| # | 问题 | 现象 | 修法 |
|---|------|------|------|
| 1 | 下载源挂了 | `github.fishros.org` 返回 **403**，`ament_cmake clone failed` | `microros_utils/repositories.py`：把 `http://github.fishros.org/https://github.com/` 全部替换成 `https://github.com/`；并把 `clone()` 的 `git clone` 加 `--depth 1` 提速 |
| 2 | CMake 挑错 python | `ModuleNotFoundError: No module named 'em'`（rcutils 生成头文件时）；dev/mcu 的 site-packages 出现 `python3.10` 与 `python3.14` 混装 | `microros_utils/library_builder.py`：`build_dev_environment()` 和 `build_mcu_environment()` 的 colcon 命令都加 `-DPYTHON_EXECUTABLE=<platformio 的 python> -DPython3_EXECUTABLE=<同一个>` |
| 3 | **中文路径** | `rosidl_adapter` 正常生成了 `.idl`，但 `rosidl_generate_interfaces.cmake:240` 报 `list index: 1 out of range (-1, 0)` | CMake 3.22 的 `file(STRINGS)` 遇到非 ASCII 字节（`桌面`）会把一行截断，idl tuple 变成没有冒号的单元素。`library_builder.py` 里加 `patch_rosidl_adapter_for_nonascii_path()`，把 `rosidl_adapter/cmake/rosidl_adapt_interfaces.cmake` 里的 `file(STRINGS "${idl_output}" idl_tuples)` 换成 `file(READ)` + `string(REPLACE "\n" ";")` + `list(REMOVE_ITEM idl_tuples "")` |

> 想彻底躲开第 3 条：把工程放在**纯英文路径**下（例如 `~/X_roscar`），
> 中文路径对 ROS/CMake 工具链是一类常见坑。

## ⚠️ Agent 侧还有第二道限制（比客户端更硬）

把客户端 MTU 提到 2048 之后，**360 点（约 1500 B）的 /scan 依然传不过去**，
agent 报：

```
error | Processor.cpp | process_write_data_submessage |
  deserialization error processing WRITE_DATA submessage
```

原因：agent（Micro-XRCE-DDS-Agent）自己也有一个 MTU —— `UXR_CONFIG_UDP_TRANSPORT_MTU`，
**编译期写死**（agent 的 InternalClient 也用它），实测效果就是**客户端消息超过约 1024 B
就被拒**（200 点/860 B 能过，360 点/1500 B 不行）。而老版 agent（如 fishros 镜像
`micro-ros-agent:humble`）的命令行**没有 `--mtu` 选项**，改不了。

所以点数的实际上限由 agent 决定：

| /scan 点数 | 消息大小 | 结果 |
|---:|:---:|:---|
| 200 | ~860 B | ✅ 实测可用（当前配置） |
| 360 | ~1500 B | ❌ agent 反序列化失败 |

想突破只有一条路：**自己编译一个 MTU 改大的 agent**（改 `UXR_CONFIG_UDP_TRANSPORT_MTU`
后 `colcon build` micro-ROS-Agent，superbuild 会拉 Fast-DDS/Fast-CDR，网络耗时较长）。

## 回滚

`.pio/libmicroros_mtu1024.bak` 是旧的 MTU=1024 预编译库，需要时可 `mv` 回去
（放回去就等于回到 200 点，不用重新编译）。确认板子跑得稳之后可以删掉。
