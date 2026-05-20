/**
 * @file winding_number_f.h
 * @brief 定义 WNV 存储和二元布尔指示函数。
 */
// 环绕数和面分类时的指示函数。

#pragma once

#include <array>
#include <cstddef>
#include <initializer_list>
#include <memory>
#include <vector>

namespace ember
{
// TODO：当前仅实现两个网格的单一布尔运算，多网格、表达式计算需要写语法解析。

/**
 * @brief 环绕数向量。
 *
 * 当前布尔求解公开入口只支持二元操作，因此常见 WNV/WNTV 固定为两个分量。
 * 为避免热路径上每个二元 tag 都触发堆分配，前两个分量直接内联存储；
 * 若未来测试或调用方构造超过两个分量，则回退到动态存储以保留向量语义。
 */
class WNV
{
public:
    /// @brief 构造空 WNV。
    WNV() noexcept = default;

    /// @brief 拷贝构造，保留内联或动态存储形态。
    WNV(const WNV &other)
        : inlineValues_(other.inlineValues_),
          size_(other.size_)
    {
        if (other.overflow_)
            overflow_ = std::make_unique<std::vector<int>>(*other.overflow_);
    }

    WNV(WNV &&other) noexcept = default;

    /// @brief 从初始化列表构造 WNV。
    WNV(std::initializer_list<int> values)
    {
        assign(values);
    }

    /// @brief 构造 `count` 个分量且每个分量均为 `value` 的 WNV。
    WNV(std::size_t count, int value)
    {
        assign(count, value);
    }

    /// @brief 拷贝赋值，保留内联或动态存储形态。
    WNV &operator=(const WNV &other)
    {
        if (this == &other)
            return *this;

        inlineValues_ = other.inlineValues_;
        size_ = other.size_;
        if (other.overflow_)
            overflow_ = std::make_unique<std::vector<int>>(*other.overflow_);
        else
            overflow_.reset();
        return *this;
    }

    WNV &operator=(WNV &&other) noexcept = default;

    /// @brief 从初始化列表赋值。
    WNV &operator=(std::initializer_list<int> values)
    {
        assign(values);
        return *this;
    }

    /// @brief 重新写入 `count` 个值相同的分量。
    void assign(std::size_t count, int value)
    {
        size_ = count;
        if (count <= kInlineSize)
        {
            overflow_.reset();
            inlineValues_.fill(0);
            for (std::size_t i = 0; i < count; ++i)
                inlineValues_[i] = value;
            return;
        }

        overflow_ = std::make_unique<std::vector<int>>(count, value);
    }

    /// @brief 重新写入初始化列表给出的分量。
    void assign(std::initializer_list<int> values)
    {
        size_ = values.size();
        if (size_ <= kInlineSize)
        {
            overflow_.reset();
            inlineValues_.fill(0);
            std::size_t index = 0;
            for (int value : values)
                inlineValues_[index++] = value;
            return;
        }

        overflow_ = std::make_unique<std::vector<int>>(values.begin(), values.end());
    }

    /// @brief 返回当前分量数。
    std::size_t size() const noexcept
    {
        return size_;
    }

    /// @brief 判断是否没有分量。
    bool empty() const noexcept
    {
        return size_ == 0;
    }

    /// @brief 读取或改写指定分量。
    int &operator[](std::size_t index) noexcept
    {
        return overflow_ == nullptr ? inlineValues_[index] : (*overflow_)[index];
    }

    /// @brief 读取指定分量。
    const int &operator[](std::size_t index) const noexcept
    {
        return overflow_ == nullptr ? inlineValues_[index] : (*overflow_)[index];
    }

    friend bool operator==(const WNV &lhs, const WNV &rhs) noexcept
    {
        if (lhs.size_ != rhs.size_)
            return false;
        for (std::size_t i = 0; i < lhs.size_; ++i)
        {
            if (lhs[i] != rhs[i])
                return false;
        }
        return true;
    }

    friend bool operator!=(const WNV &lhs, const WNV &rhs) noexcept
    {
        return !(lhs == rhs);
    }

private:
    static constexpr std::size_t kInlineSize = 2u;

    std::array<int, kInlineSize> inlineValues_{};
    std::unique_ptr<std::vector<int>> overflow_;
    std::size_t size_ = 0;
};

enum BoolStatus
{
    OUT,
    IN
};

// 两网格布尔差运算指示函数，使用 int 指定被减网格和减网格；如 A-B，A 为被减网格。
inline BoolStatus f_diff(const WNV &wnv, int subed, int subor)
{
    return (wnv[subed] != 0 && wnv[subor] == 0) ? IN : OUT;
}

inline BoolStatus f_intersection(const WNV &wnv, int A, int B)
{
    return (wnv[A] != 0 && wnv[B] != 0) ? IN : OUT;
}

inline BoolStatus f_union(const WNV &wnv, int A, int B)
{
    return (wnv[A] != 0 || wnv[B] != 0) ? IN : OUT;
}
}
