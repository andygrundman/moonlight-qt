#pragma once

#include <cstddef>
#include <mutex>
#include <vector>

class FloatBuffer
{
  public:
    static constexpr std::size_t kDefaultCapacity = 512;

    explicit FloatBuffer(std::size_t capacity = kDefaultCapacity);

    void push(float value) noexcept;
    std::size_t copyInto(float* outBuffer, std::size_t outSize, float& out_min, float& out_max) const;
    void clear() noexcept;

    std::size_t capacity() const noexcept
    {
        return m_capacity;
    }

    bool is_full() const noexcept;
    std::size_t size() const noexcept;

    float average() const noexcept;
    double sum() const noexcept;

    void dump() const noexcept;

  private:
    static bool is_power_of_two(std::size_t x) noexcept;
    void recompute_min_max_unsafe() noexcept;

  private:
    std::vector<float> m_buffer;  // backing storage, length == capacity_
    const std::size_t m_capacity;  // power of two
    std::size_t m_count;  // number of valid elements (0..capacity_)
    std::size_t m_head;  // index where the next push will write
    float m_min;  // current minimum across valid window
    float m_max;  // current maximum across valid window
    double m_sum;  // running sum for O(1) average
    mutable std::mutex m_mtx;  // guards all mutable state
};
