#pragma once

#include <functional>
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
     * @brief 一条完整日志事件。
     */
    struct LogEvent
    {
        LogLevel level = LogLevel::Off;
        LogCategory category = LogCategory::Io;
        std::string scope;
        std::string message;
    };

    /**
     * @brief 日志输出回调类型。
     */
    using LogSink = std::function<void(const LogEvent &)>;

    /**
     * @brief 设置全库日志级别。
     *
     * @param[in] level 新的日志级别。
     */
    void setLogLevel(LogLevel level);

    /**
     * @brief 读取当前全库日志级别。
     *
     * @return 当前日志级别。
     */
    LogLevel getLogLevel();

    /**
     * @brief 设置全库日志输出回调。
     *
     * 传入空回调时会恢复为默认日志 sink。
     *
     * @param[in] sink 新的日志输出回调。
     */
    void setLogSink(LogSink sink);

    /**
     * @brief 恢复默认日志配置。
     *
     * 该操作会将日志级别重置为 `LogLevel::Off`，并将 sink 恢复为默认日志输出。
     */
    void resetLogging();

    /**
     * @brief 判断给定级别当前是否启用。
     *
     * @param[in] level 待检查的日志级别。
     * @retval true 该级别日志当前允许输出。
     * @retval false 当前级别被关闭或低于全局阈值。
     */
    bool shouldLog(LogLevel level);

    /**
     * @brief 立即发送一条日志事件。
     *
     * @param[in] level 日志级别。
     * @param[in] category 日志类别。
     * @param[in] scope 调用点范围说明。
     * @param[in] message 日志正文。
     */
    void emitLog(LogLevel level, LogCategory category, const char *scope, const std::string &message);

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
        if (!shouldLog(level))
        {
            return;
        }

        emitLog(level, category, scope, std::forward<MessageBuilder>(messageBuilder)());
    }
}
