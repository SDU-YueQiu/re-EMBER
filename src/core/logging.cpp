#include "logging.h"

#include <spdlog/logger.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <memory>
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

        spdlog::level::level_enum toSpdlogLevel(LogLevel level) noexcept
        {
            switch (level)
            {
            case LogLevel::Info:
                return spdlog::level::info;
            case LogLevel::Debug:
                return spdlog::level::debug;
            case LogLevel::Error:
                return spdlog::level::err;
            case LogLevel::Off:
                return spdlog::level::off;
            }

            return spdlog::level::off;
        }

        std::shared_ptr<spdlog::logger> emberLogger()
        {
            static const std::shared_ptr<spdlog::logger> logger = []()
            {
                auto sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
                auto result = std::make_shared<spdlog::logger>("ember", std::move(sink));
                result->set_pattern("%v");
                result->set_level(spdlog::level::off);
                return result;
            }();

            return logger;
        }
    }

    void setLogLevel(LogLevel level)
    {
        emberLogger()->set_level(toSpdlogLevel(level));
    }

    void resetLogging()
    {
        setLogLevel(LogLevel::Off);
    }

    bool detail::isLogEnabled(LogLevel level)
    {
        if (level == LogLevel::Off)
        {
            return false;
        }

        return emberLogger()->should_log(toSpdlogLevel(level));
    }

    void emitLog(LogLevel level, LogCategory category, const char *scope, const std::string &message)
    {
        if (!detail::isLogEnabled(level))
        {
            return;
        }

        try
        {
            emberLogger()->log(
                toSpdlogLevel(level),
                "[{}][{}][{}] {}",
                logLevelName(level),
                logCategoryName(category),
                (scope != nullptr) ? scope : "",
                message);
        }
        catch (...)
        {
            // 日志属于诊断旁路，不应影响主流程行为。
        }
    }
}
