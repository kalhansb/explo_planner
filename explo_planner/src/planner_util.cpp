/// @file planner_util.cpp
/// @brief Definitions for the small pure planner helpers (see header).

#include "explo_planner/planner_util.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

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
  // A NaN distance (or speed, or safety factor -- std::max does not sanitise
  // NaN either) used to survive the clamp: std::clamp is written as
  // `v < lo ? lo : hi < v ? hi : v`, both comparisons answer false against NaN,
  // and the NaN is returned unchanged. That is the worst possible failure for a
  // WATCHDOG, because every later `elapsed > budget` test is then false too --
  // the NAVIGATE timeout never fires, the robot sits on a dead goal, and the
  // cell runs to max_steps. In the campaign record that is indistinguishable
  // from a genuinely slow cell, so a censored run is scored as a completed one.
  // An infinite raw budget already landed on the ceiling via the clamp; this
  // makes NaN do the same, which is the conservative reading of "we do not know
  // how far it is": still bounded, still fires, and never cuts a legitimate
  // long drive short at the floor. (2026-09-18)
  if (std::isnan(raw)) raw = max_sec;
  // The ceiling wins over the floor, for exactly the reason pursuitBudgetSec
  // spells out below: nothing orders min_sec against max_sec (they are separate
  // parameters), std::clamp with lo > hi is undefined behaviour, and in practice
  // the floor won -- a transposed pair produced a budget LONGER than the ceiling
  // the caller asked for, which is backwards for a timeout. Kept written the
  // same way in both functions so the two cannot drift apart.
  return std::clamp(raw, std::min(min_sec, max_sec), max_sec);
}

bool teamComplete(int active_peers, int expected_peers) {
  return expected_peers > 0 && active_peers >= expected_peers;
}

bool shouldRendezvous(bool reconnect_enabled, bool have_anchor,
                      int active_peers, int expected_peers) {
  if (!reconnect_enabled || expected_peers <= 0 || !have_anchor) return false;
  return !teamComplete(active_peers, expected_peers);
}

bool rendezvousWaitExpired(double waited_sec, double max_wait_sec) {
  return max_wait_sec > 0.0 && waited_sec >= max_wait_sec;
}

long long nextAgreedOccurrence(long long t_meet_ms, long long interval_ms,
                               double not_before_sec) {
  const long long not_before_ms =
      static_cast<long long>(std::llround(not_before_sec * 1000.0));
  // The agreed instant itself, whenever it is still ahead of the floor. Rolling
  // it forward here would skip the FIRST agreed meeting, which is the one the
  // whole team committed to and the only one a robot arming early can keep.
  if (t_meet_ms >= not_before_ms) return t_meet_ms;
  // Not a recurrence: one agreed instant, already past. Return it rather than
  // invent a later one — see the header for why a due-on-arrival appointment is
  // the safe answer and a fabricated instant is not.
  if (interval_ms <= 0) return t_meet_ms;
  // Ceiling division. (not_before_ms - t_meet_ms) is strictly positive by the
  // test above and interval_ms is strictly positive by the test above that, so
  // both operands are non-negative and the usual signed-truncation trap does
  // not apply: the +interval-1 form is exact, and it lands on t_meet_ms exactly
  // when the floor falls on an occurrence.
  const long long k =
      (not_before_ms - t_meet_ms + interval_ms - 1) / interval_ms;
  return t_meet_ms + k * interval_ms;
}

double arrivalShortfallSec(long long lead_ms, double max_lateness_sec) {
  // No estimate. Not "unreachable" — unknown, and the two must not share an
  // answer: a robot off the snapshot grid reads -1 here for a meeting it may be
  // standing next to, and rolling it forward on that would walk followers away
  // from appointments in exactly the far-apart case the arm exists to test.
  if (lead_ms < 0) return 0.0;
  // Sanitised, not propagated. Both of these reach llround() inside
  // nextAgreedOccurrence if they survive — undefined for a NaN — and a negative
  // budget would demand an early arrival from every robot, which is the
  // unconditional per-robot term the header forbids.
  double budget_sec = max_lateness_sec;
  if (!std::isfinite(budget_sec) || budget_sec < 0.0) budget_sec = 0.0;
  // std::max, not a branch, and the zero is exact: a robot inside its budget
  // adds NOTHING to the floor, so every robot that can attend the nearest
  // occurrence indexes the lattice from the same place and the team cannot
  // fork. The whole safety argument for putting a per-robot term here at all
  // rests on this line.
  return std::max(0.0, lead_ms / 1000.0 - budget_sec);
}

bool dwellConfirmed(bool eligible, double now_sec, double confirm_sec,
                    bool* armed, double* since_sec) {
  // Defensive rather than decorative: the two state words come from a caller
  // that has to pass the right pair, and a null here would be a silent
  // permanent "not yet" at a site that gates a reunion.
  if (armed == nullptr || since_sec == nullptr) return false;
  if (!eligible) {
    *armed = false;
    return false;
  }
  if (confirm_sec <= 0.0) return true;
  if (!*armed) {
    *armed     = true;
    *since_sec = now_sec;
    // The arming tick never fires. See the header: this is a dwell, not a
    // deadline, and returning true here would make the whole guard a no-op
    // wearing a parameter.
    return false;
  }
  return (now_sec - *since_sec) >= confirm_sec;
}

bool dwellHeld(bool eligible, double now_sec, double confirm_sec, bool armed,
               double since_sec) {
  if (!eligible) return false;
  if (confirm_sec <= 0.0) return true;
  if (!armed) return false;
  return (now_sec - since_sec) >= confirm_sec;
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

bool allocPeerPositionFresh(double age_sec, double max_age_sec) {
  // Unbounded first, and written as `<= 0` rather than `== 0` so a negative
  // parameter reads as "no limit" too — same convention as max_sec above.
  if (max_age_sec <= 0.0) return true;
  // A negative age is TeamModel's "no position held". Not fresh: under a live
  // TTL, "we know nothing" must never come out the same as "we just heard".
  if (age_sec < 0.0) return false;
  return age_sec <= max_age_sec;
}

Eigen::Vector3f meetingPoint(const Eigen::Vector3f& self_at_contact,
                             const Eigen::Vector3f& peer_at_contact) {
  return 0.5f * (self_at_contact + peer_at_contact);
}

} // namespace explo_planner
