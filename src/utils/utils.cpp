/*
 * Copyright (C) 2015-2023 Swift Navigation Inc.
 * Contact: https://support.swiftnav.com
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <utils/utils.h>
#include <algorithm>
#include <filesystem>

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
double covarianceToEstimatedHorizonatalError(
               const double cov_n_n, const double cov_n_e, const double cov_e_e) {
  const double mx_det = cov_n_n * cov_e_e - cov_n_e * cov_n_e;
  const double mx_mean_trace = (cov_n_n + cov_e_e) / 2.0;

  const double a = sqrt(mx_mean_trace * mx_mean_trace - mx_det);
  const double e1 = mx_mean_trace + a;
  const double e2 = mx_mean_trace - a;

  double ehe_squared = std::max(e1, e2);  // 39.35%
  ehe_squared *= 2.2952;                  // 68.27%

  return sqrt(ehe_squared);
}

double covarianceToEstimatedHorizonatalDirectionError(
       const double n, const double e, const double cov_n_n, const double cov_e_e ) {

  const double a = sqrt( n*n + e*e );
  const double c = sqrt( cov_n_n + cov_e_e );
  double ede_deg = atan2( c, a ) * 180.0 / M_PI;

  return ede_deg;
}

}  // namespace Covariance

namespace FileSystem {
bool createDir(const std::string& dir) {
  bool result = true;

  if (!std::filesystem::exists(dir))
    result = std::filesystem::create_directories(dir);

  return result;
}
}  // namespace FileSystem
