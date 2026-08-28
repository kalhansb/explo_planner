#pragma once
/// @file home_trail.hpp
/// @brief Geometry of the outbound breadcrumb trail used by the homing ladder.
///
/// Three computations, pulled out of ExploPlannerNode because they are pure,
/// because they had no coverage at all, and above all because two of them MUST
/// agree: the approach watchdog scores the robot on the distance it still has
/// to cover, and the nav budget decides how long it may take to cover it. If
/// those two disagree about what "remaining" means, the ladder either fires
/// while the robot is making fine progress or never fires at all — and the
/// difference between those outcomes is a censored cell versus a completed one.
/// Duplicated inline in three places they would drift; here they cannot.
///
/// The shared invariant, relied on by every function below: crumb 0 IS home.
/// home_pos_ and home_trail_[0] are assigned from the same latest_pos_ in the
/// same pose callback, so the distance between them is identically zero. Both
/// selection functions therefore start at index 1 — see their comments for what
/// selecting crumb 0 actually does, which is worse than a rounding error in
/// both cases.

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <Eigen/Core>

namespace explo_planner {
namespace home_trail {

/// Index of the crumb nearest `pos`, searching from 1 (never home itself).
/// Returns 1 for a trail with a single usable crumb; the caller must not call
/// this with `trail.size() <= 1`, where there is no retrace to engage at all.
///
/// Selecting crumb 0 would put the planner in RETRACE while making the
/// republished goal, and the approach metric, identical to DIRECT: the robot is
/// re-sent the very goal that just failed twice, under a label saying
/// otherwise. It is also a one-way door — the escape branch keys off
/// `home_mode_ == RETRACE`, so a nominal retrace that never retraces sends
/// every later fire straight to an escape and deletes three rungs of the
/// ladder.
inline int nearestCrumbFromOne(const std::vector<Eigen::Vector3f>& trail,
                               const Eigen::Vector3f& pos) {
  int best_i = 1;
  float best = std::numeric_limits<float>::max();
  for (int i = 1; i < static_cast<int>(trail.size()); ++i) {
    const float d = (trail[i] - pos).head<2>().norm();
    if (d < best) { best = d; best_i = i; }
  }
  return best_i;
}

/// Distance from `pos` to crumb `idx`, plus the trail from `idx` down to home.
/// This is what the robot still has to drive while retracing, and it is the one
/// definition of "remaining" that both the watchdog and the nav budget use.
///
/// No trail[0]-to-home term: they are the same point (see the file comment), so
/// writing it out would only suggest a correction that is not happening.
/// `idx` outside [1, size) is a spent or unset trail and yields 0.
inline float remainingTrailDistance(const std::vector<Eigen::Vector3f>& trail,
                                    int idx, const Eigen::Vector3f& pos) {
  if (idx <= 0 || idx >= static_cast<int>(trail.size())) return 0.0f;
  float m = (pos - trail[idx]).head<2>().norm();
  for (int i = idx; i > 0; --i) {
    m += (trail[i] - trail[i - 1]).head<2>().norm();
  }
  return m;
}

/// Nearest crumb whose distance from `pos` falls inside [band_min, band_max],
/// skipping `exclude_idx` (the crumb a previous escape already used) and
/// searching from 1. Ties break toward the HIGHER index — the most recently
/// visited crumb, which is the one behind the robot. -1 when nothing qualifies,
/// which is the caller's signal to use the rotate-behind fallback.
///
/// From index 1 for a sharper reason than the retrace case: for a robot stalled
/// 1.5-6 m out, home sits squarely in the escape band, so crumb 0 would aim the
/// escape at the one goal already known to have trapped it — and AHEAD of it,
/// not behind, so the ~180 deg rotate-in-place the mechanism depends on never
/// happens. An escape onto home is a resend wearing an escape's label.
inline int pickBandCrumb(const std::vector<Eigen::Vector3f>& trail,
                         const Eigen::Vector3f& pos, float band_min_m,
                         float band_max_m, int exclude_idx) {
  int best_i = -1;
  float best_d = std::numeric_limits<float>::max();
  for (int i = 1; i < static_cast<int>(trail.size()); ++i) {
    if (i == exclude_idx) continue;
    const float d = (trail[i] - pos).head<2>().norm();
    if (d < band_min_m || d > band_max_m) continue;
    if (best_i < 0 || d < best_d - 1e-3f ||
        (std::fabs(d - best_d) <= 1e-3f && i > best_i)) {
      best_d = d;
      best_i = i;
    }
  }
  return best_i;
}

}  // namespace home_trail
}  // namespace explo_planner
