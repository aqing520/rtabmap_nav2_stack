# RTAB-Map Nav2 Stack

## 0. 编译说明

### 0.1 编译方法

所有命令均在项目根目录执行。开始前确认没有其他编译进程：

```bash
pgrep -af 'colcon build|cmake --build'
```

首次编译或彻底重建：

```bash
source /opt/ros/humble/setup.bash

CLEAN_BUILD=1 \
JOBS=8 \
WORKERS=4 \
HEAVY_JOBS=1 \
bash scripts/do_build_all.sh
```

日常增量编译只需将 `CLEAN_BUILD` 改为 `0`：

```bash
CLEAN_BUILD=0 bash scripts/do_build_all.sh
```

编译完成后加载环境：

```bash
source install/setup.bash
```

内存不足时可降低并行度：

```bash
JOBS=4 WORKERS=2 HEAVY_JOBS=1 bash scripts/do_build_all.sh
```

### 0.2 本次编译问题

`rtabmap_util` 链接阶段出现以下错误，导致依赖它的包被跳过：

```text
librtabmap_util_plugins.so: undefined reference to
rtabmap_util::MapsManager::MapsManager()
```

检查确认：

- `MapsManager.cpp` 和 CMake 配置正常。
- 源码与旧工作空间中的可编译版本一致。
- 不需要修改源码或复制旧二进制库。
- 根据日志判断，很可能是重复或并发构建时产生了共享库链接竞争。

确认没有其他编译进程后，串行重编该包：

```bash
source /opt/ros/humble/setup.bash
source scripts/use_rtabmap_0234_env.sh

colcon build \
  --executor sequential \
  --parallel-workers 1 \
  --symlink-install \
  --allow-overriding rtabmap_util \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DOpenCV_DIR=/usr/lib/cmake/opencv4 \
  --packages-select rtabmap_util
```

本次串行重编成功，且没有修改 `rtabmap_util` 源码。随后执行增量编译即可继续构建剩余包：

```bash
CLEAN_BUILD=0 bash scripts/do_build_all.sh
```

### 0.3 失败时先检查

1. 不要同时运行多个 `colcon build`。
2. 查看 `log/latest_build/<包名>/stderr.log`。
3. 先单独重编失败包，再继续增量编译。
4. 不要复制其他工作空间的 `build/`、`install/` 或动态库。
