#include "floatbuffer.h"

#include "SDL_compat.h"

#include <algorithm>
#include <cfloat>
#include <cstring>
#include <sstream>
#include <stdexcept>

FloatBuffer::FloatBuffer(std::size_t capacity) :
    m_buffer(capacity),
    m_capacity(capacity),
    m_count(0),
    m_head(0),
    m_min(FLT_MAX),
    m_max(-FLT_MAX),
    m_sum(0.0f)
{
	if (!is_power_of_two(m_capacity)) {
		throw std::invalid_argument("FloatBuffer capacity must be a power of two");
	}
}

void FloatBuffer::push(float value) noexcept
{
	std::lock_guard<std::mutex> lock(m_mtx);

	const bool was_full = (m_count == m_capacity);
	float evicted = 0.0f;

	if (was_full) {
		// We are about to overwrite the oldest value at m_head.
		evicted = m_buffer[m_head];
		m_sum -= static_cast<double>(evicted);
	} else {
		++m_count;
	}

	m_buffer[m_head] = value;
	m_head = (m_head + 1) & (m_capacity - 1);

	// Update cheap aggregates.
	m_sum += static_cast<double>(value);
	if (value < m_min) m_min = value;
	if (value > m_max) m_max = value;

	// If we evicted the current min or max, recompute
	if (was_full && (evicted == m_min || evicted == m_max)) {
		recompute_min_max_unsafe();
	}
}

std::size_t FloatBuffer::copyInto(float *outBuffer, std::size_t outSize, float &out_min, float &out_max) const
{
	std::lock_guard<std::mutex> lock(m_mtx);

	if (m_count == 0) {
		out_min = 0.0f;
		out_max = 0.0f;
		return 0;
	}

	std::size_t outLen = std::min(m_count, outSize);

	// Oldest element lives at tail.
	const std::size_t tail = (m_head + m_capacity - m_count) & (m_capacity - 1);
	const std::size_t first = std::min(m_capacity - tail, outLen);

	// First contiguous chunk.
	std::memcpy(outBuffer, m_buffer.data() + tail, first * sizeof(float));

	// If wrapped, copy remaining prefix from index 0.
	if (first < outLen) {
		std::memcpy(outBuffer + first, m_buffer.data(), (outLen - first) * sizeof(float));
	}

	out_min = m_min;
	out_max = m_max;
	return static_cast<int>(outLen);
}

void FloatBuffer::clear() noexcept
{
	std::lock_guard<std::mutex> lock(m_mtx);
	m_head = 0;
	m_count = 0;
	m_min = FLT_MAX;
	m_max = -FLT_MAX;
	m_sum = 0.0f;
	// Note: we intentionally do not zero m_buffer for performance.
}

std::size_t FloatBuffer::size() const noexcept
{
	std::lock_guard<std::mutex> lock(m_mtx);
	return m_count;
}

bool FloatBuffer::is_full() const noexcept
{
	std::lock_guard<std::mutex> lock(m_mtx);
	return m_count == m_capacity;
}

float FloatBuffer::average() const noexcept
{
	std::lock_guard<std::mutex> lock(m_mtx);
	return (m_count > 0) ? static_cast<float>(m_sum / static_cast<double>(m_count)) : 0.0f;
}

double FloatBuffer::sum() const noexcept
{
	std::lock_guard<std::mutex> lock(m_mtx);
	return (m_count > 0) ? m_sum : 0.0f;
}

bool FloatBuffer::is_power_of_two(std::size_t x) noexcept
{
	return x != 0 && (x & (x - 1)) == 0;
}

void FloatBuffer::recompute_min_max_unsafe() noexcept
{
	float mn = FLT_MAX;
	float mx = -FLT_MAX;

	const std::size_t tail = (m_head + m_capacity - m_count) & (m_capacity - 1);
	const std::size_t first = std::min(m_capacity - tail, m_count);

	const float *p0 = m_buffer.data() + tail;
	for (std::size_t i = 0; i < first; ++i) {
		const float v = p0[i];
		if (v < mn) mn = v;
		if (v > mx) mx = v;
	}

	if (first < m_count) {
		const float *p1 = m_buffer.data();
		for (std::size_t i = 0; i < (m_count - first); ++i) {
			const float v = p1[i];
			if (v < mn) mn = v;
			if (v > mx) mx = v;
		}
	}

	m_min = mn;
	m_max = mx;
}

void FloatBuffer::dump() const noexcept
{
	std::lock_guard<std::mutex> lock(m_mtx);

	if (m_count == 0) {
		SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "[FloatBuffer empty]");
		return;
	}

	std::ostringstream oss;
	oss << "[FloatBuffer size=" << m_count << "/" << m_capacity << "] ";

	// Start from oldest element
	std::size_t tail = (m_head + m_capacity - m_count) & (m_capacity - 1);
	std::size_t first = std::min(m_capacity - tail, m_count);

	// first contiguous segment
	for (std::size_t i = 0; i < first; ++i) {
		if (i > 0) oss << ',';
		oss << m_buffer[tail + i];
	}

	// wraparound segment
	for (std::size_t i = 0; i < m_count - first; ++i) {
		oss << ',' << m_buffer[i];
	}

	SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "%s", oss.str().c_str());
}
