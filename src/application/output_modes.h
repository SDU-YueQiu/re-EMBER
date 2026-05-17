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
    Conforming
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
    return false;
}

/**
 * @brief 判断应用层导出模式是否需要 CGAL Nef 正则化。
 */
inline bool isNefOutputTopologyMode(AppOutputTopologyMode) noexcept
{
    return false;
}

/**
 * @brief 映射到 I/O 层轻量拓扑恢复策略。
 */
inline PolygonSoupTopologyMode toPolygonSoupTopologyMode(AppOutputTopologyMode mode) noexcept
{
    switch (mode)
    {
    case AppOutputTopologyMode::Raw:
        return PolygonSoupTopologyMode::Raw;
    case AppOutputTopologyMode::Conforming:
        return PolygonSoupTopologyMode::Conforming;
    }

    return PolygonSoupTopologyMode::Raw;
}
}
