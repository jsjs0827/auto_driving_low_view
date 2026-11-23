# librecharge_mark.so 版本说明

升库是否成功：**对比本文件 / `VERSION` 里的 `so_md5` 与实际 so 的 md5**。一致即本版本；不一致说明 submodule 未更新，或 so 被换过但版本说明没跟上。

| 项 | 值 |
|---|---|
| so md5 | `d25ad8e5a72bfe90035787e494e9753b` |
| so 大小 | 8616944 字节 |
| 编译时间（so mtime） | 2026-10-30T15:28:42+08:00 |
| 写入 main 时间 | 2026-10-30T15:28:43+08:00 |
| 源码分支 | `apriltaglib` |
| 源码 commit | `a80cbbe683daa00d0a187ec127ddd18dde27e385` |
| 源码工作区 dirty | 0（1=未提交改动也编进了 so） |
| 头文件 md5 | `ee48e27594d7646193827e88a653e3ff` |

## 核对命令

在 submodule 或 main 工作区执行：

```bash
md5sum lib/aarch64/librecharge_mark.so
grep '^so_md5=' VERSION
# 两行 hash 相同 → 当前库与版本说明一致
```

感知仓对照：

```bash
md5sum recharge_mark/lib/aarch64/librecharge_mark.so
grep '^so_md5=' recharge_mark/VERSION
```
