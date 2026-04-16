# kEmber 项目架构

## 📁 目录结构

```
kEmber/
├── src/
│   ├── math/              # 数学基础层
│   │   ├── math256.h                 # 256位整数向量、行列式
│   │   └── plane_geometry256.h       # 平面、齐次坐标
│   │
│   ├── geometry/          # 几何图元层
│   │   ├── geometry256.h/cpp         # Line256, Segment256, Polygon256
│   │
│   ├── algorithm/         # 核心算法层
│   │   ├── clipping.h/cpp            # 多边形裁剪
│   │   ├── bsp.h/cpp                 # BSP树
│   │   ├── winding_number.*          # (待实现) 环绕数系统
│   │   └── subdivision.*             # (待实现) 空间细分
│   │
│   ├── core/              # 顶层接口
│   │   └── bool_problem.h/cpp        # 布尔运算管理器
│   │
│   ├── tests/             # 测试代码
│   │   └── math256_tests.h/cpp
│   │
│   └── main.cpp           # 程序入口
│
├── include/               # 第三方库
│   └── slimcpplib/                   # 256位整数库
│
├── reference/             # 论文文档
└── docs/                  # 项目文档
```

## 🔗 依赖关系

```
main.cpp
  └─> core/
       └─> algorithm/
            └─> geometry/
                 └─> math/
                      └─> slimcpplib/
```

**设计原则**：低层不依赖高层，保持单向依赖。
