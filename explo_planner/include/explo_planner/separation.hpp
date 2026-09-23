#pragma once
/// @file separation.hpp
/// @brief Soft team-separation discount applied to the exploration utility.
///
/// WHY THIS EXISTS. Across campaigns cr3/cr4/cr5 the single best predictor of
/// how long a two-robot run took was not which reconnection policy the arm ran
/// — every arm contrast on finish time came back unresolved — but how much of
/// the run the two robots spent close together. The fraction of time within
/// 20 m correlates with finish time at Spearman rho = 0.677, and it is
/// significant at n = 46 cells while no arm mean is. Separation is the thing
/// that moves the endpoint; the policies mostly move connectivity.
///
/// Nothing in the planner acts on it. There are two team-aware mechanisms and
/// neither disperses:
///
///   - MinPos (coordination.hpp) is a hard veto over a claim disc around the
///     peer's GOAL. It is structurally almost inert: a wider claim radius
///     widens what gets TESTED, not what gets vetoed, and `rejected_by_minpos`
///     works out to roughly one event per sixty-cell campaign. A veto that
///     fires once a campaign cannot be the lever.
///
///   - The global allocator (global_allocator.hpp) assigns different tour
///     cells to different robots, which EXCLUDES — this robot is steered to
///     its own cells — but does not DISPERSE: within the assigned
///     neighbourhood nothing prefers the far side of it to the near side, and
///     the neighbourhood is a 3x3 of cells, so there is real room to sit
///     either 5 m or 30 m from the partner while obeying the allocator
///     exactly.
///
/// This unit is the missing continuous term: every candidate, every tick, a
/// smooth discount for being close to a teammate. It is deliberately NOT a
/// veto — a veto is what MinPos already is, and the reason MinPos does not
/// bite is that a binary test over a small disc is almost never the thing that
/// decides a pick. A discount competes with the information term on every
/// candidate instead of waiting for an overlap that rarely happens.
///
/// THE ANCHOR IS THE PEER'S POSITION, NOT ITS GOAL, and that is a decision
/// rather than a convenience. The peer's goal is what MinPos already contests,
/// and folding it in here would make the two mechanisms measure each other:
/// one campaign could not then say whether a change came from softening the
/// veto or from adding dispersion. The peer's POSITION is also the quantity
/// the rho = 0.677 finding is actually about — robot-to-robot separation — and
/// it is ground the peer has just finished sweeping, so backing away from it
/// avoids re-covering as a side effect.
///
/// SHAPE. Linear ramp, multiplicative:
///
///   s(c) = 1 - weight * max(0, 1 - d(c) / radius)
///
/// where d(c) is the distance from candidate c to the NEAREST eligible peer.
/// s = 1 - weight on top of a teammate, rising linearly to s = 1 at `radius`
/// and staying there. Multiplicative rather than additive so it composes with
/// the SSMI-normalised utility U = info / (eps + cost)^gamma without needing to
/// know that utility's units, which change with gamma; a subtractive penalty
/// would have to be re-tuned every time the exponent moved.
///
/// Linear rather than Gaussian: the finding is stated as a threshold ("time
/// within 20 m"), the ramp reaches exactly 1 at exactly that radius with no
/// tail past it, and there is no evidence in the data that would justify
/// picking a kernel width on top of the radius. One shape parameter is one
/// thing to defend.
///
/// This unit is pure: no ROS, no clock, no map. The caller decides which peers
/// are eligible (see the freshness note on Anchor) and does the applying.
/// Moved comments: doc/explo_planner_code_notes.md

#include <cstddef>
#include <string>
#include <vector>

namespace explo_planner {

class SeparationTerm {
public:
  struct Config {
    /// Discount at zero distance, in [0, 1]. 0 disables the term outright and
    /// is the default, so a binary built with this unit reproduces the
    /// pre-separation planner bit-for-bit until an experiment asks for it.
    /// 1 means a candidate standing on a teammate scores exactly zero.
    double weight = 0.0;

    /// Distance at which the discount has fully decayed, metres. configure()
    /// refuses, not clamps, values outside [1e-3, 1e6]: d / radius is computed
    /// in float. (notes: separation-radius-bounds)
    double radius_m = 20.0;

    /// Maximum POSITION age, seconds, at which a peer still repels; about two
    /// TeamWorld heartbeats plus margin. A stale position would push this robot
    /// away from where the teammate was. (notes: separation-max-age)
    double max_age_sec = 10.0;
  };

  /// One teammate this robot may be repelled by, map-frame XY. Eligibility is
  /// the caller's job: a position exists, its POSITION age (not last-heard age)
  /// is within max_age_sec, and the peer has not finished.
  /// (notes: separation-anchor-eligibility)
  struct Anchor {
    float x = 0.0f;
    float y = 0.0f;
  };

  SeparationTerm() = default;

  /// Returns an empty string if usable, else why the term was DISABLED;
  /// out-of-range values disable, never clamp. weight == 0 is OFF, not an
  /// error; radius_m and max_age_sec are still validated and kept for
  /// diagnostics. (notes: separation-configure-refuses)
  std::string configure(const Config& cfg);

  /// Whether the term will actually change any score. False when weight is 0,
  /// and false after a refused configure().
  bool enabled() const { return enabled_; }

  /// The configuration in force — the values the planner will actually use,
  /// which is what belongs in a manifest. After a refused configure() this is
  /// the default (disabled) one, not the values that were rejected.
  const Config& config() const { return cfg_; }

  /// Whether a teammate with this position age may be an anchor.
  /// position_age_sec is a POSITION age; negative means no position held.
  /// Independent of enabled(): the eligible-peer count is logged in every arm.
  /// (notes: separation-eligible-anchor)
  bool eligibleAnchor(double position_age_sec) const;

  /// Distance from (x, y) to the nearest anchor, or -1 when there are none.
  /// Static and independent of configuration: the planner logs this for every
  /// tick, including ticks on which the term is off, so a campaign can measure
  /// how close the robots were without having to run the treatment.
  static float nearestAnchorDist(float x, float y,
                                 const std::vector<Anchor>& anchors);

  /// The multiplier for a candidate at (x, y): 1 when the term is off, when
  /// there are no anchors, or when the nearest is at least `radius_m` away;
  /// down to `1 - weight` on top of the nearest anchor. Never negative and
  /// never above 1.
  float discount(float x, float y, const std::vector<Anchor>& anchors) const;

  /// Scales only a finite, strictly positive utility: unreachable candidates
  /// carry -inf (-inf * 0 is NaN), and a negative utility scaled below 1 would
  /// turn the discount into a reward. (notes: separation-apply-guard)
  static float apply(float utility, float discount);

private:
  Config cfg_{};        ///< weight 0 until a successful configure().
  bool   enabled_ = false;
};

}  // namespace explo_planner
