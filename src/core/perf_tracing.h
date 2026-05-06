/**
 * @file perf_tracing.h
 * @brief 定义可选 Tracy 性能插桩的本地包装宏。
 */
#pragma once

#if defined(REEMBER_ENABLE_TRACY)

#include <tracy/Tracy.hpp>

/**
 * @brief 标记当前作用域为一个 Tracy zone。
 */
#define REEMBER_PROFILE_ZONE(nameLiteral) ZoneScopedN(nameLiteral)

/**
 * @brief 标记 math256 这类底层热点作用域为一个可单独开关的 Tracy zone。
 */
#if defined(REEMBER_ENABLE_TRACY_MATH)
#define REEMBER_PROFILE_MATH_ZONE(nameLiteral) ZoneScopedN(nameLiteral)
#else
#define REEMBER_PROFILE_MATH_ZONE(nameLiteral) ((void)0)
#endif

/**
 * @brief 按函数名标记当前作用域为一个 Tracy zone。
 */
#define REEMBER_PROFILE_FUNCTION() ZoneScoped

/**
 * @brief 向 Tracy 记录一个数值序列。
 */
#define REEMBER_PROFILE_VALUE(nameLiteral, value) TracyPlot(nameLiteral, static_cast<double>(value))

/**
 * @brief 向 Tracy 当前捕获流写入一段消息。
 */
#define REEMBER_PROFILE_MESSAGE(text, size) TracyMessage((text), (size))

/**
 * @brief 标记一个 Tracy 帧边界。
 */
#define REEMBER_PROFILE_FRAME(nameLiteral) FrameMarkNamed(nameLiteral)

#else

#define REEMBER_PROFILE_ZONE(nameLiteral) ((void)0)
#define REEMBER_PROFILE_MATH_ZONE(nameLiteral) ((void)0)
#define REEMBER_PROFILE_FUNCTION() ((void)0)
#define REEMBER_PROFILE_VALUE(nameLiteral, value) ((void)0)
#define REEMBER_PROFILE_MESSAGE(text, size) ((void)0)
#define REEMBER_PROFILE_FRAME(nameLiteral) ((void)0)

#endif
