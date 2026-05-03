#include "logging.h"

#include <atomic>
#include <iostream>
#include <mutex>
#include <utility>

namespace ember
{
    namespace
    {
        const char *logLevelName(LogLevel level) noexcept
        {
            switch (level)
            {
            case LogLevel::Off:
                return "Off";
            case LogLevel::Error:
                return "Error";
            case LogLevel::Info:
                return "Info";
            case LogLevel::Debug:
                return "Debug";
            }

            return "Unknown";
        }

        const char *logCategoryName(LogCategory category) noexcept
        {
            switch (category)
            {
            case LogCategory::Io:
                return "Io";
            case LogCategory::BoolProblem:
                return "BoolProblem";
            case LogCategory::Bsp:
                return "Bsp";
            case LogCategory::Geometry:
                return "Geometry";
            case LogCategory::Tracing:
                return "Tracing";
            }

            return "Unknown";
        }

        void defaultLogSink(const LogEvent &event)
        {
            std::clog << "["
                      << logLevelName(event.level)
                      << "]["
                      << logCategoryName(event.category)
                      << "]["
                      << event.scope
                      << "] "
                      << event.message
                      << std::endl;
        }

        std::atomic<LogLevel> g_logLevel(LogLevel::Off);
        std::mutex g_sinkMutex;

        LogSink &sinkStorage()
        {
            static LogSink sink = defaultLogSink;
            return sink;
        }
    }

    void setLogLevel(LogLevel level)
    {
        g_logLevel.store(level, std::memory_order_release);
    }

    LogLevel getLogLevel()
    {
        return g_logLevel.load(std::memory_order_acquire);
    }

    void setLogSink(LogSink sink)
    {
        std::lock_guard<std::mutex> lock(g_sinkMutex);
        sinkStorage() = sink ? std::move(sink) : LogSink(defaultLogSink);
    }

    void resetLogging()
    {
        {
            std::lock_guard<std::mutex> lock(g_sinkMutex);
            sinkStorage() = defaultLogSink;
        }

        g_logLevel.store(LogLevel::Off, std::memory_order_release);
    }

    bool shouldLog(LogLevel level)
    {
        if (level == LogLevel::Off)
        {
            return false;
        }

        return static_cast<int>(level) <= static_cast<int>(g_logLevel.load(std::memory_order_acquire));
    }

    void emitLog(LogLevel level, LogCategory category, const char *scope, const std::string &message)
    {
        if (!shouldLog(level))
        {
            return;
        }

        LogSink sink;
        {
            std::lock_guard<std::mutex> lock(g_sinkMutex);
            sink = sinkStorage();
        }

        if (!sink)
        {
            return;
        }

        LogEvent event;
        event.level = level;
        event.category = category;
        event.scope = (scope != nullptr) ? scope : "";
        event.message = message;

        try
        {
            sink(event);
        }
        catch (...)
        {
            // 日志属于诊断旁路，不应影响主流程行为。
        }
    }
}
