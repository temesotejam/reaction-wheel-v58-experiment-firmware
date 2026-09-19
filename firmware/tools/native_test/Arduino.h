#pragma once
// Minimal String adapter for native serialization tests only.
#include <string>
typedef bool boolean;
class String {
 public:
  std::string value;
  String() = default;
  String(const char* s) : value(s) {}
  String(unsigned int n) : value(std::to_string(n)) {}
  String& operator+=(const char* s) { value += s; return *this; }
  String& operator+=(char c) { value += c; return *this; }
  String& operator+=(const String& s) { value += s.value; return *this; }
};
