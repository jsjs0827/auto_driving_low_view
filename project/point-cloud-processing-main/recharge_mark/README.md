# recharge_mark SDK (`main`)

本分支**只含**交叉编译产物，给感知仓 submodule 使用。

当前库版本见 **[VERSION.md](VERSION.md)**（so md5 + 编译时间）。机器可读副本：`VERSION`。

- `include/percep_demo/api_recharge_mark.h`（单头：API + 宿主）
- `lib/aarch64/librecharge_mark.so`
- `config/recharge_mark.json`（回充参数；板端装到 config/ai_node/）

源码在 `apriltaglib`。不要把 cpp / CMakeLists / package.xml 合进本分支。

核对：

```bash
md5sum lib/aarch64/librecharge_mark.so
grep '^so_md5=' VERSION
```
