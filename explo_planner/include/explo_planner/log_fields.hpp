#pragma once
/// @file log_fields.hpp
/// @brief A flat list of named values for one jsonl event, with no ROS in it.
///
/// Gen 34's team logic (team_core.hpp) is pure C++ and produces its events as
/// data; the node hands them to ExperimentLog, which writes them. Keeping the
/// field list here lets the unit tests and the in-process harness read the
/// same events the node logs.

#include <string>
#include <utility>
#include <vector>

namespace explo_planner {

struct LogField {
  enum class Type { kNum, kInt, kBool, kStr };
  std::string name;
  Type type = Type::kNum;
  double d = 0.0;
  long long i = 0;
  bool b = false;
  std::string s;

  static LogField num(std::string n, double v) {
    LogField f; f.name = std::move(n); f.type = Type::kNum; f.d = v; return f;
  }
  static LogField integer(std::string n, long long v) {
    LogField f; f.name = std::move(n); f.type = Type::kInt; f.i = v; return f;
  }
  static LogField boolean(std::string n, bool v) {
    LogField f; f.name = std::move(n); f.type = Type::kBool; f.b = v; return f;
  }
  static LogField str(std::string n, std::string v) {
    LogField f; f.name = std::move(n); f.type = Type::kStr; f.s = std::move(v); return f;
  }
};

using LogFields = std::vector<LogField>;

/// Look up a field by name; nullptr when absent. For tests and the harness.
inline const LogField* findField(const LogFields& fs, const std::string& name) {
  for (const LogField& f : fs)
    if (f.name == name) return &f;
  return nullptr;
}

}  // namespace explo_planner
