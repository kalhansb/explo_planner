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
/// Moved comments: doc/explo_planner_code_notes.md

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <Eigen/Core>

namespace explo_planner {
namespace home_trail {

/// Nearest crumb to pos, searching from 1 (never home); returns 1 for a single
/// usable crumb. Do not call with trail.size() <= 1. Crumb 0 would make RETRACE
/// resend the failed DIRECT goal. (notes: home-trail-nearest-crumb)
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

/// XY distance from pos to crumb idx plus the trail from idx down to home: the
/// one definition of remaining used by both the watchdog and the nav budget.
/// idx outside [1, size) yields 0. (notes: home-trail-remaining-distance)
inline float remainingTrailDistance(const std::vector<Eigen::Vector3f>& trail,
                                    int idx, const Eigen::Vector3f& pos) {
  if (idx <= 0 || idx >= static_cast<int>(trail.size())) return 0.0f;
  float m = (pos - trail[idx]).head<2>().norm();
  for (int i = idx; i > 0; --i) {
    m += (trail[i] - trail[i - 1]).head<2>().norm();
  }
  return m;
}

/// Nearest crumb whose distance from pos is in [band_min_m, band_max_m],
/// skipping exclude_idx and crumb 0; ties go to the higher (more recent) index.
/// -1 if none: use the rotate-behind fallback.
/// (notes: home-trail-escape-band-crumb)
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
