/**
 * @file output_modes.h
 * @brief 声明命令行导出后处理模式。
 */
#pragma once

#include <string>

namespace ember::app
{
/**
 * @brief 布尔结果导出前的重型后处理模式。
 */
enum class OutputPostprocessMode
{
    None,
    Nef
};

/**
 * @brief 返回导出后处理模式的 CLI/日志名称。
 */
inline const char *toString(OutputPostprocessMode mode) noexcept
{
    switch (mode)
    {
    case OutputPostprocessMode::None:
        return "none";
    case OutputPostprocessMode::Nef:
        return "nef";
    }

    return "none";
}

/**
 * @brief 从 CLI 字符串解析导出后处理模式。
 */
inline bool parseOutputPostprocessMode(
    const std::string &token,
    OutputPostprocessMode &outMode) noexcept
{
    if (token == "none")
    {
        outMode = OutputPostprocessMode::None;
        return true;
    }
    if (token == "nef")
    {
        outMode = OutputPostprocessMode::Nef;
        return true;
    }

    return false;
}
}
