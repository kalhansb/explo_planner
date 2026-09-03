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

    /// Distance at which the discount has fully decayed, metres. Default 20,
    /// the radius the correlation is stated at. Bounded to [1e-3, 1e6] by
    /// configure(): the term computes d / radius in FLOAT, where a radius
    /// below about 1e-38 rounds to zero and a candidate sitting exactly on a
    /// teammate then evaluates 0/0 and logs a NaN discount, while a radius
    /// above about 3e38 rounds to infinity and flattens the discount to a
    /// constant across every candidate. Neither is a plausible request; both
    /// are refused rather than clamped, for the reason configure() gives.
    double radius_m = 20.0;

    /// How old a peer position may be and still repel, seconds. NOT a
    /// convenience bound — a separation term driven by a stale position pushes
    /// this robot away from where the teammate WAS, which after a long outage
    /// is uncorrelated with where it is and may be the exact ground it has
    /// since left uncovered.
    ///
    /// The default (10 s) is two TeamWorld heartbeats plus margin. It is
    /// deliberately short, and under the 2026-09-03 radio that short bound is
    /// self-consistent rather than restrictive: with a 30 m range horizon a
    /// teammate is only HEARD while it is close, which is exactly when this
    /// term should be acting. When the link is down the peer is beyond the
    /// horizon, further away than `radius_m` anyway, and the term correctly
    /// goes quiet instead of guessing.
    double max_age_sec = 10.0;
  };

  /// One teammate this robot is willing to be repelled by, in map-frame XY.
  ///
  /// ELIGIBILITY IS THE CALLER'S JOB and there are three parts to it, none of
  /// which this unit can see: the position must exist, it must be fresher than
  /// max_age_sec measured as a POSITION age (not as "when did we last hear
  /// anything about this robot" — a relayed status update refreshes the second
  /// while leaving the first untouched), and the peer must not have finished.
  /// A finished teammate is parked and covering nothing, so being repelled by
  /// it is pure loss.
  struct Anchor {
    float x = 0.0f;
    float y = 0.0f;
  };

  SeparationTerm() = default;

  /// Install a configuration. Returns "" when the term is usable as given, or
  /// a human-readable reason it was DISABLED. Out-of-range values disable
  /// rather than clamp: `weight = 3` and `radius_m = -20` are not weak
  /// requests for separation, they are typos, and a term that quietly ran at
  /// some clamped value would put a number in the manifest that the binary did
  /// not use. Silence is the failure mode this project keeps paying for, so
  /// the refusal is returned for the caller to log.
  ///
  /// The one exception is `weight == 0`, which is OFF by request and returns
  /// "" — it is the default and not an error. Note that `radius_m` and
  /// `max_age_sec` are still validated in that case and still kept: they are
  /// what the caller's separation DIAGNOSTICS are measured on, in every arm
  /// including the untreated one.
  std::string configure(const Config& cfg);

  /// Whether the term will actually change any score. False when weight is 0,
  /// and false after a refused configure().
  bool enabled() const { return enabled_; }

  /// The configuration in force — the values the planner will actually use,
  /// which is what belongs in a manifest. After a refused configure() this is
  /// the default (disabled) one, not the values that were rejected.
  const Config& config() const { return cfg_; }

  /// Whether a teammate with this position age may be used as an anchor.
  ///
  /// This lives here rather than in the caller for one reason: `max_age_sec`
  /// is otherwise a knob that is parsed, range-checked, written into the run
  /// manifest and then consumed entirely outside anything a unit test can
  /// reach. That is the precise shape of a failure this project has already
  /// paid for more than once — a value that the manifest swears the run used
  /// and the binary never read — and a review of this very file found that
  /// replacing the age comparison with a constant would pass the whole suite.
  /// Moving the comparison into the unit makes the knob testable.
  ///
  /// `position_age_sec` is a POSITION age (see Anchor), and a negative value
  /// means no position is held at all. Independent of `enabled()`: the
  /// eligible-peer count is logged in every arm, treated or not.
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

  /// Apply a multiplier to a utility, safely.
  ///
  /// Only a FINITE, STRICTLY POSITIVE utility is scaled, and both halves of
  /// that guard are load-bearing:
  ///
  ///   - Unreachable candidates carry U = -inf so they sort to the bottom.
  ///     -inf * 0 is NaN, and a NaN score is a candidate whose position in the
  ///     sort depends on which comparisons the sort happens to make. The sort
  ///     is NaN-safe, but "sorts somewhere defined" is not the same as "is not
  ///     produced", and a NaN here would be produced by exactly the
  ///     configuration an experiment is most likely to run (weight = 1).
  ///
  ///   - A negative utility scaled by a factor below 1 gets LARGER, so a
  ///     discount would become a reward. Nothing currently produces one, which
  ///     is precisely why it would go unnoticed if something started to.
  static float apply(float utility, float discount);

private:
  Config cfg_{};        ///< weight 0 until a successful configure().
  bool   enabled_ = false;
};

}  // namespace explo_planner
