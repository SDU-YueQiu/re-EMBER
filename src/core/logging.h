#pragma once

#include <string>
#include <utility>

namespace ember
{
    /**
     * @brief 全库统一日志级别。
     */
    enum class LogLevel
    {
        Off = 0, ///< 关闭全部日志。
        Error,   ///< 仅记录失败分支。
        Info,    ///< 记录阶段性流程信息。
        Debug    ///< 记录细粒度调试信息。
    };

    /**
     * @brief 日志所属模块类别。
     */
    enum class LogCategory
    {
        Io,          ///< OBJ 读写与 polygon soup 转换。
        BoolProblem, ///< subdivision 与布尔结果汇总。
        Bsp,         ///< 局部 BSP 构建。
        Geometry,    ///< 底层裁剪和几何诊断。
        Tracing      ///< WNV/WNTV 路径追踪相关诊断。
    };

    /**
     * @brief 设置全库日志输出级别。
     *
     * @param[in] level 新的日志级别。
     */
    void setLogLevel(LogLevel level);

    /**
     * @brief 恢复默认日志级别。
     *
     * 该操作会将日志级别重置为 `LogLevel::Off`。
     */
    void resetLogging();

    /**
     * @brief 立即发送一条日志事件。
     *
     * @param[in] level 日志级别。
     * @param[in] category 日志类别。
     * @param[in] scope 调用点范围说明。
     * @param[in] message 日志正文。
     */
    void emitLog(LogLevel level, LogCategory category, const char *scope, const std::string &message);

    namespace detail
    {
        bool isLogEnabled(LogLevel level);
    }

    /**
     * @brief 仅在级别启用时延迟构造日志正文。
     *
     * @tparam MessageBuilder 可调用对象类型，返回 `std::string`。
     * @param[in] level 日志级别。
     * @param[in] category 日志类别。
     * @param[in] scope 调用点范围说明。
     * @param[in] messageBuilder 惰性构造消息的回调。
     */
    template <typename MessageBuilder>
    void emitLogLazy(LogLevel level, LogCategory category, const char *scope, MessageBuilder &&messageBuilder)
    {
        if (!detail::isLogEnabled(level))
        {
            return;
        }

        emitLog(level, category, scope, std::forward<MessageBuilder>(messageBuilder)());
    }
}
