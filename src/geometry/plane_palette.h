/**
 * @file plane_palette.h
 * @brief 定义叶片局部平面调色板和轻量句柄。
 */
#pragma once

#include "geometry/plane_geometry256.h"

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace ember
{
inline constexpr std::uint32_t kInvalidPlaneHandleIndex = std::numeric_limits<std::uint32_t>::max();
inline constexpr std::uint32_t kInvalidPolygonHandleIndex = std::numeric_limits<std::uint32_t>::max();

struct PlaneHandle
{
    std::uint32_t index = kInvalidPlaneHandleIndex;

    constexpr PlaneHandle() noexcept = default;
    explicit constexpr PlaneHandle(std::uint32_t value) noexcept : index(value) {}

    constexpr bool isValid() const noexcept
    {
        return index != kInvalidPlaneHandleIndex;
    }

    friend constexpr bool operator==(PlaneHandle lhs, PlaneHandle rhs) noexcept
    {
        return lhs.index == rhs.index;
    }

    friend constexpr bool operator!=(PlaneHandle lhs, PlaneHandle rhs) noexcept
    {
        return !(lhs == rhs);
    }
};

struct PolygonHandle
{
    std::uint32_t index = kInvalidPolygonHandleIndex;

    constexpr PolygonHandle() noexcept = default;
    explicit constexpr PolygonHandle(std::uint32_t value) noexcept : index(value) {}

    constexpr bool isValid() const noexcept
    {
        return index != kInvalidPolygonHandleIndex;
    }

    friend constexpr bool operator==(PolygonHandle lhs, PolygonHandle rhs) noexcept
    {
        return lhs.index == rhs.index;
    }

    friend constexpr bool operator!=(PolygonHandle lhs, PolygonHandle rhs) noexcept
    {
        return !(lhs == rhs);
    }
};

class PlanePalette
{
public:
    PlaneHandle intern(const Plane3i &plane)
    {
        const auto found = planeToHandle_.find(plane);
        if (found != planeToHandle_.end())
            return PlaneHandle(found->second);

        if (planes_.size() >= static_cast<std::size_t>(kInvalidPlaneHandleIndex))
            throw std::runtime_error("PlanePalette handle range exhausted.");

        const std::uint32_t index = static_cast<std::uint32_t>(planes_.size());
        planes_.push_back(plane);
        planeToHandle_.emplace(planes_.back(), index);
        return PlaneHandle(index);
    }

    const Plane3i &get(PlaneHandle handle) const
    {
        if (!handle.isValid() || handle.index >= planes_.size())
            throw std::runtime_error("PlanePalette received an invalid plane handle.");
        return planes_[handle.index];
    }

    std::size_t size() const noexcept
    {
        return planes_.size();
    }

    bool empty() const noexcept
    {
        return planes_.empty();
    }

    void clear() noexcept
    {
        planes_.clear();
        planeToHandle_.clear();
    }

private:
    struct PlaneHash
    {
        std::size_t operator()(const Plane3i &plane) const noexcept
        {
            std::uint64_t hash = 1469598103934665603ull;
            auto mix = [&](const Integer &value)
            {
                hash ^= integerLow64(value);
                hash *= 1099511628211ull;
            };
            mix(plane.a);
            mix(plane.b);
            mix(plane.c);
            mix(plane.d);
            return static_cast<std::size_t>(hash);
        }
    };

    struct PlaneEqual
    {
        bool operator()(const Plane3i &lhs, const Plane3i &rhs) const noexcept
        {
            return areSamePlaneEquation(lhs, rhs);
        }
    };

    std::vector<Plane3i> planes_;
    std::unordered_map<Plane3i, std::uint32_t, PlaneHash, PlaneEqual> planeToHandle_;
};
}
