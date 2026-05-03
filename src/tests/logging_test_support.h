#pragma once

#include "core/logging.h"

#include <algorithm>
#include <string>
#include <vector>

namespace ember::tests
{
    struct ScopedLogCapture
    {
        explicit ScopedLogCapture(LogLevel level = LogLevel::Off)
        {
            resetLogging();
            setLogSink([this](const LogEvent &event)
                {
                    events.push_back(event);
                });
            setLogLevel(level);
        }

        ~ScopedLogCapture()
        {
            resetLogging();
        }

        bool hasEvent(
            LogLevel level,
            LogCategory category,
            const std::string &scopeFragment,
            const std::string &messageFragment) const
        {
            return std::any_of(
                events.begin(),
                events.end(),
                [&](const LogEvent &event)
                {
                    const bool scopeMatches =
                        scopeFragment.empty() ||
                        event.scope.find(scopeFragment) != std::string::npos;
                    const bool messageMatches =
                        messageFragment.empty() ||
                        event.message.find(messageFragment) != std::string::npos;
                    return event.level == level &&
                           event.category == category &&
                           scopeMatches &&
                           messageMatches;
                });
        }

        std::vector<LogEvent> events;
    };
}
