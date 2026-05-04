# AGENT 本地上下文

本文件是当前工作树的本地 AGENT 指南。全局 memory 只能作为线索，遇到差异时以当前代码、测试和新鲜构建结果为准。

## 当前事实

- 当前流水线是 `OBJ -> 多边形集合 -> BoolProblem -> SubdivisionSolver -> resultFragments -> OBJ n 边面`。
- `BoolProblem` 是公开门面，不再暴露递归子节点；叶子诊断使用 `leafSummaries()`。
- `SubdivisionSolver` 独占递归节点、AABB、局部参考点、叶片片段、分类片段和结果汇总。
- `path_candidates.h` 保留公开候选类型和模板枚举入口；内部候选构造在 `path_candidate_details.h`。
- CMake 使用显式源文件列表，新增源码需要同步加入 `CMakeLists.txt`。

## 工作规则

- 所有构建、测试和 smoke 产物都放在 `build/`。
- 仓库自有源码的文件头和解释性注释写中文；公开接口使用 Doxygen 结构。
- 不参考旧 README 作为架构事实；如果全局 memory 仍提到 `BoolProblem::solveRecursive()` 调用链，应视为过期。
- 默认保持 OBJ 输出为 n 边面多边形集合，不主动三角化输出。
- 默认不编辑 `include/slimcpplib`、`assets`、`reference`、`Doxyfile` 或构建产物。

## 验证命令

```powershell
cmake -S . -B build
cmake --build build --config Debug --target re-EMBER_tests
ctest --test-dir build -C Debug --output-on-failure --timeout 60
cmake --build build --config Debug --target re-EMBER
build\Debug\re-EMBER.exe --lhs assets\models\workpiece_block.obj --rhs assets\models\tool_box.obj --op difference --out build\codex_boolean_smoke.obj --leaf-threshold 25
```
