/**
 * @file output_modes.h
 * @brief 声明应用层导出拓扑模式。
 */
#pragma once

#include "io/io.h"

#include <string>

namespace ember::app
{
/**
 * @brief 应用层可见的布尔结果导出拓扑模式。
 */
enum class AppOutputTopologyMode
{
    Raw,
    Conforming,
    ConformingMergeConvex,
    Nef
};

/**
 * @brief 返回应用层导出拓扑模式的 CLI/日志名称。
 */
inline const char *toString(AppOutputTopologyMode mode) noexcept
{
    switch (mode)
    {
    case AppOutputTopologyMode::Raw:
        return "raw";
    case AppOutputTopologyMode::Conforming:
        return "conforming";
    case AppOutputTopologyMode::ConformingMergeConvex:
        return "conforming-merge-convex";
    case AppOutputTopologyMode::Nef:
        return "nef";
    }

    return "raw";
}

/**
 * @brief 从 CLI 字符串解析应用层导出拓扑模式。
 */
inline bool parseAppOutputTopologyMode(
    const std::string &token,
    AppOutputTopologyMode &outMode) noexcept
{
    if (token == "raw")
    {
        outMode = AppOutputTopologyMode::Raw;
        return true;
    }
    if (token == "conforming")
    {
        outMode = AppOutputTopologyMode::Conforming;
        return true;
    }
    if (token == "conforming-merge-convex")
    {
        outMode = AppOutputTopologyMode::ConformingMergeConvex;
        return true;
    }
    if (token == "nef")
    {
        outMode = AppOutputTopologyMode::Nef;
        return true;
    }

    return false;
}

/**
 * @brief 判断应用层导出模式是否需要 CGAL Nef 正则化。
 */
inline bool isNefOutputTopologyMode(AppOutputTopologyMode mode) noexcept
{
    return mode == AppOutputTopologyMode::Nef;
}

/**
 * @brief 映射到 I/O 层轻量拓扑恢复策略。
 *
 * `Nef` 模式内部固定先补齐 T-junction，再交给 CGAL Nef 正则化；不会叠加凸共面合并。
 */
inline PolygonSoupTopologyMode toPolygonSoupTopologyMode(AppOutputTopologyMode mode) noexcept
{
    switch (mode)
    {
    case AppOutputTopologyMode::Raw:
        return PolygonSoupTopologyMode::Raw;
    case AppOutputTopologyMode::Conforming:
    case AppOutputTopologyMode::Nef:
        return PolygonSoupTopologyMode::Conforming;
    case AppOutputTopologyMode::ConformingMergeConvex:
        return PolygonSoupTopologyMode::ConformingMergeConvex;
    }

    return PolygonSoupTopologyMode::Raw;
}
}
