#pragma once

constexpr unsigned long long operator"" _ms(const unsigned long long seconds) {
  return seconds * 1000ULL;
}

constexpr unsigned long long operator"" _ns(const unsigned long long seconds) {
  return seconds * 1000000000ULL;
}
