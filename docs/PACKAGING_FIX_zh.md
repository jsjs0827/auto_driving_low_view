> **历史材料（v1/v2）**：本文保留用于理解原算法与历史验证。v3 的配置、路径、构建及运行入口已经改为纯 C++；本文旧命令和旧 DAG 配置不能直接用于本版。请以 [09_纯CPP输入与Graph迁移](09_纯CPP输入与Graph迁移.md) 为准。

# v7p1 打包修复

用户的 ARM 构建日志已经成功链接并安装 demo/raw_camera。随后在运行包全量 UTF-8 检查中，安装目录的 stb_image.h 以 `87 7d 1c 94 ...` 开头，触发失败。原始源码 SHA256/UTF-8 校验已经通过。

v7 的打包脚本会从已验证源码恢复安装后的头文件、示例 cpp 和 package.xml，但没有覆盖新安装的 vendor/stb 和 config。v7p1 将恢复映射与 CMake 的全部源码文本安装项对应，加入源文件必须存在于校验清单的要求，只修改临时 staging 副本，保留原 install 和可执行文件。仍对其他安装文本严格校验；不跳过头文件，不改编码判定，不修改编译器或链接参数。

这次日志没有说明安装副本在哪一步变坏。CMake strip 仅针对可执行文件，不能据此归因于 strip。需要进一步调查环境时，在容器原工作目录执行：

```bash
sha256sum src/perception_ros_so_demo/vendor/stb/stb_image.h \
  install/perception_ros_so_demo/share/perception_ros_so_demo/vendor/stb/stb_image.h
od -An -tx1 -N16 install/perception_ros_so_demo/share/perception_ros_so_demo/vendor/stb/stb_image.h
```

正确源文件 SHA256 是 `594c2fe35d49488b4382dbfaec8f98366defca819d916ac95becf3e75f4200b3`，开头为可读注释 `/* stb_image - v`。

补丁包的 apply.py 先验证原源码清单和三个被替换文件的原始版本，再替换打包脚本、宿主机构建入口及说明文档并更新对应 SHA256 项。支持重复应用；拒绝未知版本或已经自行修改的受影响文件。不会重新编译或改写 install。原 v7 源码压缩包保留。

回归测试使用真实交叉编译器生成的最小 AArch64 可执行程序作为打包 fixture，验证损坏安装文本恢复、源清单约束、未知异常文本拒绝、原安装目录与 ELF 字节保持、归档校验。它验证的是打包逻辑，不是完整 ROS 节点在 X5 的编译或运行。相关结果位于 validation/packaging_fix_logs。v7 原有 WSL 三输入测试证据保留，C++ 代码未变。
