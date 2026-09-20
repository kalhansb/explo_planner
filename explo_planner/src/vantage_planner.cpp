#include "explo_planner/vantage_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "explo_planner/map_cache.hpp"

namespace explo_planner {

float VantagePlanner::standoffFor(float radius) const {
  const float r = std::max(radius, 0.0f);
  float d = r + cfg_.standoff_m;
  // Keep the trunk surface beyond the min range and the trunk axis within the
  // max range. lo > hi only for a trunk wider than the sensor envelope; fall
  // back to max_range (std::clamp is UB when lo > hi).
  const float lo = cfg_.fov_min_range + r;
  const float hi = cfg_.fov_max_range;
  if (lo > hi) return hi;
  return std::clamp(d, lo, hi);
}

std::vector<CandidateViewpoint> VantagePlanner::generateVantages(
    const Eigen::Vector3f& center, float radius) const {
  std::vector<CandidateViewpoint> out;
  const int n = std::max(cfg_.n_vantages, 0);
  if (n == 0) return out;
  out.reserve(static_cast<size_t>(n));

  const float d = standoffFor(radius);
  const float step = 2.0f * static_cast<float>(M_PI) / static_cast<float>(n);
  for (int i = 0; i < n; ++i) {
    const float theta = cfg_.start_angle_rad + static_cast<float>(i) * step;
    CandidateViewpoint vp;
    vp.position = Eigen::Vector3f(center.x() + d * std::cos(theta),
                                  center.y() + d * std::sin(theta),
                                  cfg_.robot_z);
    // Face the trunk: the heading from the vantage back to the centre is
    // theta + pi.
    vp.yaw = theta + static_cast<float>(M_PI);
    vp.is_vantage = true;
    out.push_back(vp);
  }
  return out;
}

bool VantagePlanner::lineOfSightClear(const Eigen::Vector3f& from,
                                      const Eigen::Vector3f& center,
                                      float radius,
                                      const MapCache& map) const {
  // March horizontally at the sightline height toward the trunk axis.
  const Eigen::Vector3f target(center.x(), center.y(), from.z());
  Eigen::Vector3f delta = target - from;
  const float dist = delta.norm();
  if (dist < 1e-3f) return true;  // already at the axis
  const Eigen::Vector3f dir = delta / dist;

  const float res = static_cast<float>(map.resolution());
  const float step = res > 0.0f ? res : 0.1f;
  // Stop one voxel short of the trunk surface: samples from there inward are
  // the trunk itself (an expected hit), not occluders.
  const float surface = std::max(radius, 0.0f) + step;
  const float stop = dist - surface;
  if (stop <= 0.0f) return true;  // standoff is at/inside the trunk surface

  // Exact voxel traversal (Amanatides-Woo), NOT a fixed sample spacing.
  //
  // The reason it has to be exact: NO fixed spacing can guarantee one sample
  // per crossed voxel. A ray clips the corner of a voxel for an arbitrarily
  // short length of travel, so the minimum traversal length is 0 no matter the
  // bearing, while any fixed spacing is positive. Spacing by `res` skips
  // roughly 25% of the crossed voxels on a diagonal bearing; spacing by
  // `res / L1(dir)` — the MEAN traversal length, which is what the previous
  // revision here marched by — narrows the gap but does not close it, and its
  // comment claimed a guarantee it did not deliver. Worked example at res 0.1
  // on the (2,1)/sqrt(5) bearing: the ray occupies the voxel at (1,1) for
  // 0.0559 of travel while that spacing is 0.0745, and the occluder inside it
  // is still reported CLEAR. A miss here sends the exploitation planner to a
  // viewpoint that cannot see the trunk it was chosen for: in a 270-pair trial
  // 8 of 40 genuinely occluded vantage/trunk pairs came back clear.
  //
  // So step boundary-to-boundary instead. `t_exit` is the smallest of the three
  // per-axis crossing parameters, i.e. exactly where the ray leaves the voxel
  // it is currently in, and the sample is taken at the MIDPOINT of the interval
  // — strictly interior, so it cannot land on a boundary and read a neighbour.
  // Every crossed voxel gets exactly one sample and none gets two, which is the
  // guarantee the old comment asserted. Iterations are the number of voxels the
  // ray actually crosses, so this is also the cheapest sampling that is
  // correct.
  //
  // Voxel boundaries are taken as floor(p / res) * res, in float.
  //
  // THIS IS NOT EXACTLY THE KEYING MapCache USES, and the comment here asserted
  // that it was until 2026-09-18. What getVoxel actually does is widen the
  // float sample position to double and hand it to Bonxai, which keys on
  // floor(x * inv_resolution) with inv_resolution = 1.0 / resolution held as a
  // double. Measured, not reasoned: over a scan of seven bearings the float
  // divide here disagrees with that by a WHOLE VOXEL on ~13 of every 800 nice
  // round coordinates at res 0.1 — x = 0.7f keys to 6 in the grid and 7 here —
  // and the float `t_delta` accumulation drifts the later boundaries up by
  // ~1.5e-7 per step on top of that. So this is a real disagreement, not a
  // rounding curiosity, and the line above overstated the agreement.
  //
  // IT IS LEFT AS IT IS, DELIBERATELY, and the naive repair is worse. Widening
  // the entry point to double (`double(from[a]) + d * double(t_begin)`) is the
  // obvious fix and it is WRONG: the position this loop actually samples is
  // built in float by Eigen (`from + dir * t`), so a double entry point is a
  // different point from the one the grid will be asked about. At
  // from.x = 0.1f that variant puts the ray in voxel 1 while the float sample
  // lands in voxel 2. Any correct version has to form the entry point in float
  // — the same arithmetic as the sample — and only then widen for the keying,
  // and it needs to be checked against a dense ground-truth sweep of the float
  // ray rather than against these twelve tests, all of which pass under every
  // variant tried.
  //
  // DEFERRED ON PURPOSE. The whole lineOfSightClear path is unreachable in a
  // campaign: run_campaign.sh launches every cell with EXPLOIT=0, so
  // exploitation_enabled_ is false and all four call sites
  // (onPeerExploitIntent, doExploitPlan x2, doExploitDwell) are gated off. No
  // banked or planned result can move on this, which is why it is not worth
  // taking an unmeasured change into a build. Do not "tidy" it without the
  // ground-truth sweep.
  //
  // COVERAGE. Four LoS tests march along +x (LineOfSightClearOnEmptyMap,
  // LineOfSightBlockedByOccluder, TrunkSurfaceIsCarvedOut,
  // OccluderJustBeyondTrunkSurfaceBlocks) — the one family of bearings where
  // every sampling scheme agrees, so they could never have caught the
  // skipped-voxel defect this traversal replaced. FOUR MORE now do the off-axis
  // work, and this comment named only the first of them until 2026-09-18:
  // LineOfSightOffAxisOccluderIsNotSkipped (a crossed voxel must be read),
  // LineOfSightDoesNotReadVoxelsTheRayMisses (an uncrossed neighbour must not
  // be), LineOfSightAtFortyFiveDegreesCrossesOnlyTheDiagonal (the exact-corner
  // case, the only bearing reaching the multi-axis advance below) and
  // LineOfSightOnANegativeBearingReadsItsFinalVoxel (which is what makes the
  // midpoint sample load-bearing rather than a style choice). New coverage
  // still has to be off-axis; it is just no longer true that there is one test.
  //
  // MUTATION STATUS (2026-09-18, 17 mutants): killed are the tied-axis advance
  // `<=`->`<` (it stops advancing and the loop never terminates), sampling at
  // t_enter or t_exit instead of the midpoint, and dropping the one-voxel pad
  // from `surface`. THIRTEEN SURVIVED. One is a documented equivalent (the
  // textbook single-axis `break`, see the comment on
  // LineOfSightAtFortyFiveDegreesCrossesOnlyTheDiagonal). The rest are real
  // gaps with no test behind them: the own-cell carve-out (`t_begin = 0`
  // survives), floor vs trunc on negative coordinates, the `occ_stop`
  // comparison being `>=` rather than `>`, dropping the `stop` clamp on t_exit,
  // both loop-guard strictnesses, the d == 0 early-out (which every call takes,
  // since the ray is horizontal by construction), and standoffFor's `lo`
  // dropping the radius term. Deferred with the keying, same reason.
  constexpr float kInf = std::numeric_limits<float>::infinity();
  float t_max[3], t_delta[3];
  // Begin one step out from the vantage: the vantage cell itself is the free
  // pose we are standing on. That leading offset is a full resolution and is
  // the own-cell carve-out, unrelated to the traversal below.
  const float t_begin = step;
  if (t_begin >= stop) return true;
  for (int a = 0; a < 3; ++a) {
    const float d = dir[a];
    if (d == 0.0f) {
      t_max[a] = kInf;
      t_delta[a] = kInf;
      continue;
    }
    const float p = from[a] + d * t_begin;
    const float cell = std::floor(p / step);
    // Leaving through the far face for d > 0, the near face for d < 0.
    const float bound = (d > 0.0f ? cell + 1.0f : cell) * step;
    t_max[a] = t_begin + (bound - p) / d;
    t_delta[a] = step / std::abs(d);
  }

  float t_enter = t_begin;
  while (t_enter < stop) {
    const float t_exit = std::min({t_max[0], t_max[1], t_max[2], stop});
    const Eigen::Vector3f p = from + dir * (0.5f * (t_enter + t_exit));
    if (map.getVoxel(p).p_occ >= cfg_.occ_stop) return false;
    if (t_exit >= stop) break;
    // Advance every axis whose boundary this crossing was, not just one: on an
    // exact corner crossing two or three of them coincide, and advancing one
    // would leave the others reporting a boundary already behind us.
    for (int a = 0; a < 3; ++a)
      if (t_max[a] <= t_exit) t_max[a] += t_delta[a];
    t_enter = t_exit;
  }
  return true;
}

}  // namespace explo_planner
