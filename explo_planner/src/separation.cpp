// Moved comments: doc/explo_planner_code_notes.md
#include "explo_planner/separation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace explo_planner {

std::string SeparationTerm::configure(const Config& cfg) {
  // Start from the defaults, so every refusing return below leaves a term that
  // is off and whose config() reads as off, rather than one half-configured
  // from the values that were just rejected.
  cfg_ = Config{};
  enabled_ = false;

  // Radius and max-age are validated even at weight 0: every arm logs peer
  // distance and eligible-peer count with them, so the control arm must
  // measure on the same bounds. (notes: separation-validate-unconditionally)
  if (!std::isfinite(cfg.radius_m) || cfg.radius_m <= 0.0) {
    return "separation_radius_m=" + std::to_string(cfg.radius_m) +
           " is not a positive distance; the term is off and its diagnostics "
           "fall back to the defaults";
  }
  // Radius must lie in [1e-3, 1e6] m, not just be positive: the ramp divides
  // by it in float, so near zero the discount goes NaN and near float overflow
  // it goes constant. (notes: separation-radius-bounds-2)
  if (cfg.radius_m < 1e-3 || cfg.radius_m > 1e6) {
    return "separation_radius_m=" + std::to_string(cfg.radius_m) +
           " is outside [1e-3, 1e6] m; the term is off. Radii near zero make "
           "the discount NaN and radii near float overflow make it constant, "
           "and both look like a working term in the manifest";
  }
  if (!std::isfinite(cfg.max_age_sec) || cfg.max_age_sec <= 0.0) {
    return "separation_max_age_sec=" + std::to_string(cfg.max_age_sec) +
           " is not a positive duration, so no peer position could ever be "
           "fresh enough to repel; the term is off. Turning it off here is "
           "the honest reading — the alternative is a term that is on in the "
           "manifest and inert in the binary";
  }
  if (!std::isfinite(cfg.weight)) {
    return "separation_weight is not finite; the term is off";
  }
  if (cfg.weight < 0.0 || cfg.weight > 1.0) {
    return "separation_weight=" + std::to_string(cfg.weight) +
           " is outside [0, 1]; the term is off. A negative weight is an "
           "ATTRACTION to the teammate, not a weak repulsion, and above 1 it "
           "drives the utility negative — neither is a separation term";
  }

  cfg_ = cfg;
  // weight == 0 is off BY REQUEST — the shipped default and the control arm of
  // any campaign that runs the term — so it keeps the caller's radius and
  // max-age for the diagnostics and returns no refusal.
  enabled_ = (cfg.weight > 0.0);
  return std::string();
}

bool SeparationTerm::eligibleAnchor(double position_age_sec) const {
  // Negative means "no position held", which is not the same as "a very fresh
  // one" — the caller's clock difference can only be >= 0 — so it is refused
  // explicitly rather than left to fall out of the comparison below.
  if (!(position_age_sec >= 0.0)) return false;   // also refuses NaN
  return position_age_sec <= cfg_.max_age_sec;
}

float SeparationTerm::nearestAnchorDist(float x, float y,
                                        const std::vector<Anchor>& anchors) {
  if (anchors.empty()) return -1.0f;
  // A candidate with a non-finite position has no meaningful distance to
  // anything. Reported as "no anchor" rather than as inf, so a caller that
  // logs this column cannot mistake a broken candidate for a distant peer.
  if (!std::isfinite(x) || !std::isfinite(y)) return -1.0f;

  float best = std::numeric_limits<float>::infinity();
  bool  any  = false;
  for (const Anchor& a : anchors) {
    if (!std::isfinite(a.x) || !std::isfinite(a.y)) continue;
    const float dx = x - a.x;
    const float dy = y - a.y;
    const float d  = std::sqrt(dx * dx + dy * dy);
    if (d < best) { best = d; any = true; }
  }
  return any ? best : -1.0f;
}

float SeparationTerm::discount(float x, float y,
                               const std::vector<Anchor>& anchors) const {
  if (!enabled_) return 1.0f;
  const float d = nearestAnchorDist(x, y, anchors);
  if (d < 0.0f) return 1.0f;   // no usable anchor: nothing to be repelled by

  const float r = static_cast<float>(cfg_.radius_m);
  // 1 on top of the peer, falling linearly to 0 at r and staying there.
  const float closeness = 1.0f - std::min(d / r, 1.0f);
  const float s = 1.0f - static_cast<float>(cfg_.weight) * closeness;
  // The algebra cannot leave [1-weight, 1] for finite inputs, but the clamp
  // costs nothing and means no caller has to re-derive that to trust the
  // range this function's contract promises.
  return std::clamp(s, 0.0f, 1.0f);
}

float SeparationTerm::apply(float utility, float discount) {
  if (!std::isfinite(utility) || utility <= 0.0f) return utility;
  if (!std::isfinite(discount)) return utility;
  return utility * discount;
}

}  // namespace explo_planner
