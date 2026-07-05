#pragma once

// Nanosecond-resolution timing support for frame pacing. The term QPC is taken from Windows
// but is mapped through SDL to the highest resolution timer available on each platform.

#include "SDL_compat.h"

#if defined(_MSC_VER)
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#else
#include <time.h>
#include <errno.h>
#include <sched.h>
#endif

// Time helpers
static inline uint64_t QpcFreq() {
	static uint64_t f = [] {
		return SDL_GetPerformanceFrequency();
	}();
	return f;
}

static inline uint64_t QpcNow() {
	return SDL_GetPerformanceCounter();
}

static inline uint64_t UsToQpc(int64_t us) {
	const uint64_t f = QpcFreq();
	return (us / UINT64_C(1000000)) * f +
	       (us % UINT64_C(1000000)) * f / UINT64_C(1000000);
}

static inline uint64_t QpcToUs(uint64_t qpc) {
	const uint64_t f = QpcFreq();
	uint64_t q = qpc / f;
	uint64_t r = qpc % f;
	if (r < 0) {
		--q;
		r += f;
	}
	return q * UINT64_C(1000000) + (r * UINT64_C(1000000)) / f;
}

static inline double QpcToMsD(double qpc) {
	return qpc * 1000.0 / (double)QpcFreq();
}

static inline double QpcToMs(int64_t qpc) {
    return QpcToMsD(static_cast<double>(qpc));
}

static inline uint64_t MsToQpc(double ms) {
	const double us_d = ms * 1000.0;
	const uint64_t us = static_cast<uint64_t>(us_d >= 0.0 ? us_d + 0.5 : us_d - 0.5);
    return UsToQpc(us);
}

#if defined(_MSC_VER)
	#define YieldCPU YieldProcessor
#else
	static inline void YieldCPU()
	{
	#if defined(__aarch64__) || defined(__arm64__)
		__asm__ __volatile__("yield");
	#elif defined(__x86_64__) || defined(__i386__)
		__asm__ __volatile__("pause");
	#else
		// no-op fallback
	#endif
	}
#endif

// Sleep until approximately targetQpc, then busy-wait the rest.
// slackQpc is how early to stop sleeping.
static inline void SleepUntilQpc(uint64_t targetQpc,
								 int64_t sleepSlackUs = 1000)
{
	const uint64_t sleepSlackQpc = UsToQpc(sleepSlackUs);

	for (;;) {
		const uint64_t now = QpcNow();
		if (now >= targetQpc) {
			break;
		}

		const uint64_t remainQpc = targetQpc - now;
		if (remainQpc > sleepSlackQpc) {
			const uint64_t sleepQpc = remainQpc - sleepSlackQpc;

		#if defined(_MSC_VER)
			const int64_t f = QpcFreq();
			DWORD ms = (DWORD)((sleepQpc * 1000 + f / 2) / f);
			Sleep(ms);
		#else
			const uint64_t totalNs = QpcToUs(sleepQpc) * 1000ULL;
			struct timespec ts;
			ts.tv_sec = (time_t)(totalNs / 1000000000ULL);
			ts.tv_nsec = (long)(totalNs % 1000000000ULL);

			if (ts.tv_sec > 0 || ts.tv_nsec > 0) {
				while (nanosleep(&ts, &ts) == -1 && errno == EINTR) {}
			}
		#endif
			continue;
		}

		// yield/busy-wait the last 1ms
		while (QpcNow() < targetQpc) {
			YieldCPU();
		}
		break;
	}
}
