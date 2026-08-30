#include "explo_planner/fleet_identity.hpp"

#include <string>

namespace explo_planner {

namespace {
const std::string kNoName;
}  // namespace

int maskCount(uint32_t mask) {
  int n = 0;
  for (int i = 0; i < kMaxTeamSize; ++i) {
    if (mask & robotBit(i)) ++n;
  }
  return n;
}

uint32_t teamNamesHash(const std::vector<std::string>& names) {
  // FNV-1a 32-bit, byte-exact and endian-independent (it consumes bytes, and
  // the only arithmetic is on a uint32 accumulator whose overflow is defined).
  uint32_t h = 2166136261u;
  const auto feed = [&h](unsigned char c) {
    h ^= c;
    h *= 16777619u;
  };
  for (const std::string& n : names) {
    for (const char c : n) feed(static_cast<unsigned char>(c));
    // The NUL separator is what stops {"ab","c"} and {"a","bc"} hashing alike.
    feed(0u);
  }
  return h;
}

uint32_t FleetIdentity::allMask() const {
  uint32_t m = 0;
  for (int i = 0; i < size(); ++i) m |= robotBit(i);
  return m;
}

int FleetIdentity::idOf(const std::string& name) const {
  for (size_t i = 0; i < names.size(); ++i) {
    if (names[i] == name) return static_cast<int>(i);
  }
  return -1;
}

const std::string& FleetIdentity::nameOf(int id) const {
  if (id < 0 || id >= size()) return kNoName;
  return names[static_cast<size_t>(id)];
}

FleetIdentity makeFleetIdentity(const std::vector<std::string>& names,
                                const std::string& self_name) {
  FleetIdentity out;
  if (names.empty()) return out;  // unconfigured; not an error

  if (static_cast<int>(names.size()) > kMaxTeamSize) {
    out.error = "team_robot_names has " + std::to_string(names.size()) +
                " entries, more than the " + std::to_string(kMaxTeamSize) +
                "-robot cap; every mask in the world model is indexed by "
                "position and bits past the cap would be dropped silently";
    return out;
  }
  for (size_t i = 0; i < names.size(); ++i) {
    if (names[i].empty()) {
      out.error = "team_robot_names[" + std::to_string(i) +
                  "] is empty; an empty name can never match a robot_name, so "
                  "that fleet slot would be unclaimable";
      return out;
    }
    for (size_t j = i + 1; j < names.size(); ++j) {
      if (names[i] == names[j]) {
        out.error = "team_robot_names lists '" + names[i] +
                    "' twice (positions " + std::to_string(i) + " and " +
                    std::to_string(j) +
                    "); two robots sharing one id share one knowledge bit, and "
                    "each would be credited with the other's observations";
        return out;
      }
    }
  }

  int self = -1;
  for (size_t i = 0; i < names.size(); ++i) {
    if (names[i] == self_name) {
      self = static_cast<int>(i);
      break;
    }
  }
  if (self < 0) {
    std::string listed;
    for (size_t i = 0; i < names.size(); ++i) {
      if (i) listed += ", ";
      listed += names[i];
    }
    out.error = "robot_name '" + self_name +
                "' is not in team_robot_names [" + listed +
                "]; this robot has no id of its own, so every mask it "
                "published would omit itself";
    return out;
  }

  out.configured = true;
  out.self_id = self;
  out.names = names;
  out.team_hash = teamNamesHash(out.names);
  return out;
}

std::string requireFleetIdentity(const FleetIdentity& id,
                                 const char* feature_name) {
  if (id.configured) return {};
  if (!id.error.empty()) {
    return std::string(feature_name) +
           " needs fleet identity, and team_robot_names is invalid: " +
           id.error;
  }
  return std::string(feature_name) +
         " needs fleet identity, but team_robot_names is empty. Set it to the "
         "ordered list of every robot in this fleet (the same list, in the "
         "same order, on every robot) — a robot's numeric id is its position "
         "in that array, and it indexes every knowledge mask, gossip slot and "
         "cross-robot tie-break this feature relies on.";
}

}  // namespace explo_planner
