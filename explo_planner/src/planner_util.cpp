/// @file planner_util.cpp
/// @brief Definitions for the small pure planner helpers (see header).

#include "explo_planner/planner_util.hpp"

#include <algorithm>
#include <cctype>

namespace explo_planner {

uint8_t plannerTypeId(const std::string& planner_type) {
  if (planner_type == "eig")      return 0;
  if (planner_type == "entropy")  return 1;
  if (planner_type == "frontier") return 2;
  if (planner_type == "random")   return 3;
  if (planner_type == "ssmi")     return 4;
  return 255;
}

double navBudgetSec(double dist_m, double speed_est_mps, double safety_factor,
                    double min_sec, double max_sec) {
  double raw = (dist_m / std::max(speed_est_mps, 1e-3)) * safety_factor;
  return std::clamp(raw, min_sec, max_sec);
}

bool teamComplete(int active_peers, int expected_peers) {
  return expected_peers > 0 && active_peers >= expected_peers;
}

bool shouldRendezvous(bool rendezvous_enabled, bool have_anchor,
                      int active_peers, int expected_peers) {
  if (!rendezvous_enabled || expected_peers <= 0 || !have_anchor) return false;
  return !teamComplete(active_peers, expected_peers);
}

bool rendezvousWaitExpired(double waited_sec, double max_wait_sec) {
  return max_wait_sec > 0.0 && waited_sec >= max_wait_sec;
}

ReconnectMode reconnectModeFromString(const std::string& s, bool* known) {
  std::string t;
  t.reserve(s.size());
  for (const char c : s) {
    t.push_back(
        static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  if (known) *known = true;
  if (t == "rendezvous") return ReconnectMode::RENDEZVOUS;
  if (t == "pursuit")    return ReconnectMode::PURSUIT;
  if (t == "hybrid")     return ReconnectMode::HYBRID;
  if (known) *known = false;
  return ReconnectMode::RENDEZVOUS;
}

const char* reconnectModeName(ReconnectMode m) {
  // No default case, deliberately: adding a mode without a name here is a
  // compile warning rather than an unlabelled arm at analysis time.
  switch (m) {
    case ReconnectMode::RENDEZVOUS: return "rendezvous";
    case ReconnectMode::PURSUIT:    return "pursuit";
    case ReconnectMode::HYBRID:     return "hybrid";
  }
  return "unknown";
}

double pursuitBudgetSec(double trail_head_dist_m, double staleness_sec,
                        double speed_est_mps, double safety_factor,
                        double staleness_max_sec, double min_sec,
                        double max_sec) {
  if (max_sec <= 0.0) return 0.0;
  double freshness = 1.0;
  if (staleness_max_sec > 0.0) {
    if (staleness_sec >= staleness_max_sec) return 0.0;
    freshness =
        std::clamp(1.0 - staleness_sec / staleness_max_sec, 0.0, 1.0);
  }
  const double raw = (trail_head_dist_m / std::max(speed_est_mps, 1e-3)) *
                     safety_factor * freshness;
  // The ceiling wins over the floor: min_sec comes from the nav-timeout
  // family and max_sec from the pursuit family, so nothing orders them —
  // and std::clamp with lo > hi is UB. max_sec is the bound a WAITING
  // teammate relies on, so it must hold regardless of the floor.
  return std::clamp(raw, std::min(min_sec, max_sec), max_sec);
}

Eigen::Vector3f meetingPoint(const Eigen::Vector3f& self_at_contact,
                             const Eigen::Vector3f& peer_at_contact) {
  return 0.5f * (self_at_contact + peer_at_contact);
}

} // namespace explo_planner
