#include <utils/utils.h>
#include <algorithm>
#include <cmath>

namespace TimeUtils {
constexpr uint32_t LINUX_TIME_20200101 = 1577836800U;

constexpr int32_t FIRST_YEAR = 2020;
constexpr uint32_t DAYS_IN_2020_YEAR = 366U;

constexpr uint32_t DAYS_IN_YEAR = 365U;
constexpr uint32_t DAYS_IN_LEAP_YEAR(DAYS_IN_YEAR + 1U);
constexpr uint32_t DAYS_IN_WEEK = 7U;
constexpr uint32_t HOURS_IN_DAY = 24U;
constexpr uint32_t MINUTES_IN_HOUR = 60U;
constexpr uint32_t SECONDS_IN_MINUTE = 60;
constexpr uint32_t SECONDS_IN_HOUR = (SECONDS_IN_MINUTE * MINUTES_IN_HOUR);
constexpr uint32_t SECONDS_IN_DAY = (HOURS_IN_DAY * SECONDS_IN_HOUR);
constexpr uint32_t SECONDS_IN_WEEK = (DAYS_IN_WEEK * SECONDS_IN_DAY);

constexpr int days_in_month[] = {31, 28, 31, 30, 31, 30,
                                 31, 31, 30, 31, 30, 31};

static bool IsLeapYear(const int year) {
  return ((0 == (year % 4)) && (0 != (year % 100))) || (0 == (year % 400));
}

time_t utcToLinuxTime(const struct tm& utc) {
  auto yr = FIRST_YEAR;
  uint32_t days = 0;

  while (yr < utc.tm_year) {
    days += IsLeapYear(yr) ? DAYS_IN_LEAP_YEAR : DAYS_IN_YEAR;
    yr++;
  }

  for (int32_t i = 0; i < (utc.tm_mon - 1); i++) {
    days += days_in_month[i];
  }

  if (IsLeapYear(utc.tm_year) && (utc.tm_mon > 2)) {
    days += utc.tm_mday;
  } else {
    days += utc.tm_mday - 1;
  }

  return static_cast<time_t>(LINUX_TIME_20200101 + days * SECONDS_IN_DAY +
                             utc.tm_hour * SECONDS_IN_HOUR +
                             utc.tm_min * SECONDS_IN_MINUTE + utc.tm_sec);
}

uint64_t secondsToMilliseconds(const uint64_t seconds) {
  return seconds * 1000ULL;
}

uint64_t secondsToNanoseconds(const uint64_t seconds) {
  return seconds * 1000000000ULL;
}
}  // namespace TimeUtils

namespace Covariance {
double cov2ehe(const double cov_n_n, const double cov_n_e,
               const double cov_e_e) {
  const double mx_det = cov_n_n * cov_e_e - cov_n_e * cov_n_e;
  const double mx_mean_trace = (cov_n_n + cov_e_e) / 2.0;

  const double a = sqrt(mx_mean_trace * mx_mean_trace - mx_det);
  const double e1 = mx_mean_trace + a;
  const double e2 = mx_mean_trace - a;

  double ehe_squared = std::max(e1, e2);  // 39.35%
  ehe_squared *= 2.2952;                  // 68.27%

  return sqrt(ehe_squared);
}

double cov2ede() {
  //!! TODO
  return -1.0;
}

}  // namespace Covariance