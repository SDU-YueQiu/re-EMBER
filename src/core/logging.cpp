#include "logging.h"

#include <spdlog/logger.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <sstream>
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
            static const std::shared_ptr<spdlog::logger> logger = []()
            {
                auto sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
                auto result = std::make_shared<spdlog::logger>("ember", std::move(sink));
                result->set_pattern("%v");
                result->set_level(spdlog::level::trace);
                return result;
            }();

            std::ostringstream stream;
            stream << "["
                   << logLevelName(event.level)
                   << "]["
                   << logCategoryName(event.category)
                   << "]["
                   << event.scope
                   << "] "
                   << event.message;

            spdlog::level::level_enum spdlogLevel = spdlog::level::debug;
            switch (event.level)
            {
            case LogLevel::Error:
                spdlogLevel = spdlog::level::err;
                break;
            case LogLevel::Info:
                spdlogLevel = spdlog::level::info;
                break;
            case LogLevel::Debug:
                spdlogLevel = spdlog::level::debug;
                break;
            case LogLevel::Off:
                return;
            }

            logger->log(spdlogLevel, "{}", stream.str());
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
