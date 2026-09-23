#pragma once
/// @file pursuit_predictor.hpp
/// @brief Where the missing peer WILL be when I could get there, given the tour
///        it was last heard driving. See docs/mtare_evolution_plan.md §3.7.
///
/// THE IDEA THIS FILE EXISTS TO EXPRESS: a chase aimed at where the peer WAS is
/// a chase that arrives late by construction. The shipped trail — the peer's
/// last declared goal, then its last heard pose — is a record of the past, and
/// every second of staleness moves the quarry further from it. But a peer under
/// the global allocator is not wandering: it is driving an ordered list of
/// cells, and it broadcast that list. Propagating it forward and intercepting
/// costs one Markov chain and no new message.
///
/// WHAT THIS IS NOT: an agreement mechanism. Unlike global_allocator.hpp and
/// rendezvous_scheduler.hpp, nothing here has to be bit-identical across
/// processes — the prediction is one robot's private guess about another, it
/// reaches no shared decision, and the peer never learns it was made. That is
/// why this file uses doubles freely where those two spend real design effort
/// on integer quantisation, and the distinction is worth stating because the
/// opposite mistake (assuming the agreement property extends here) would make a
/// reader trust a prediction on both ends of a link that only exists on one.
///
/// THE FLOOR IS THE EXISTING BEHAVIOUR. Every refusal path returns an invalid
/// target and the caller drives the legacy trail. §3.7 is explicit that pursuit
/// must never get WORSE than the current implementation, so a model that is
/// unsure is required to say so rather than to hand back its best guess: a
/// confident-looking intercept computed from a tour the peer abandoned ten
/// minutes ago sends the chase somewhere nobody has ever been, whereas the
/// trail at least names a place the peer provably was. `min_probability` is
/// that rule made numeric.
///
/// THE MODEL (G/I/O), and an honesty note about the port. mTARE's `pursuit_mdp`
/// expands the peer's route into a node chain with three transitions, which
/// this file reconstructs from the plan's description (§1: we port CONCEPTS,
/// not code; there is no mTARE checkout in this tree to copy semantics from).
/// The three are named here for what they do, and every probability that
/// governs them is a named parameter rather than a literal:
///
///   G — go:       advance to the next cell on the tour. Its rate is the leg's
///                 own duration, so a long leg is not exited in one step.
///   I — in place: stay in the current cell for another step. This is the
///                 peer WORKING the cell, which is most of what it is doing —
///                 a cell is a unit of exploration, not a waypoint.
///   O — off route: leave the tour entirely, and absorb. Re-planning, a
///                 blacklisted goal, a homing trigger, a mission return, or
///                 simply a tour superseded by an allocation this robot never
///                 heard. It is a HAZARD, not an event: the longer the record
///                 has been stale, the more of the distribution has drained
///                 into it, and a sufficiently old tour predicts nothing at
///                 all. That is the intended behaviour and it is what makes
///                 `min_probability` degrade gracefully into the trail rather
///                 than cutting off at a staleness threshold.
///
/// Probability is conserved exactly: the mass on the chain plus the mass in O
/// sums to one at every step. `step()` is exposed so that invariant can be
/// tested where it holds — at the transition — rather than only at the end of a
/// propagation, where a compensating pair of errors would hide.
/// Moved comments: doc/explo_planner_code_notes.md

#include <string>
#include <vector>

#include "explo_planner/cell_world.hpp"

namespace explo_planner {

/// The intercept, plus everything needed to re-derive it from a log.
struct PursuitTarget {
  /// Cell to chase, or -1 when the model refused (see `refused`).
  int   cell = -1;
  /// Its centre, in the planning frame — what the caller actually drives to.
  float x = 0.0f, y = 0.0f;

  /// P(peer is in `cell` when I could be there). The argmax value.
  double p = 0.0;
  /// Mass still on the tour at that horizon, i.e. 1 - P(off route). The ratio
  /// p / p_on_route separates "the peer is probably gone" from "the peer is
  /// probably still on the tour but spread over many cells" — two situations
  /// with the same `p` and different remedies.
  double p_on_route = 0.0;

  /// My drive to `cell` at the configured speed, and the total horizon the
  /// chain was propagated to (record age + that drive). Milliseconds.
  long long my_travel_ms = -1;
  long long horizon_ms   = -1;

  /// Tour index the chain started from, and the index that won. anchor ==
  /// winner means "intercept where it already was", which is the correct
  /// answer for a fresh record and a slow peer, not a degenerate one.
  int anchor_index = -1;
  int win_index    = -1;

  /// Admissible candidates scored, and the two ways a tour node was dropped.
  /// A prediction with `candidates == 1` is an argmax over nothing.
  int candidates          = 0;
  int rejected_invalid    = 0;  ///< cell id off the grid
  int rejected_horizon    = 0;  ///< peer.age_sec + my drive exceeds
                                ///< max_horizon_sec — NOT the drive alone

  /// Non-empty when there is no prediction. The caller MUST fall back to the
  /// legacy trail; this is a normal outcome, not a fault.
  std::string refused;

  bool valid() const { return cell >= 0 && refused.empty(); }
};

class PursuitPredictor {
public:
  struct Config {
    /// Nominal drive speeds, m/s. The peer's governs how fast the chain
    /// advances along its tour; mine governs the horizon I score it at. They
    /// are separate parameters because they are separate claims — I know my own
    /// measured speed and I am guessing at the peer's.
    double peer_speed_mps = 0.5;
    double my_speed_mps   = 0.5;

    /// Seconds the peer spends working a cell before moving on (the I
    /// transition). Zero turns the model into pure translation, predicting the
    /// peer far ahead. (notes: pursuit-dwell-sec)
    double dwell_sec = 45.0;

    /// Chain step, seconds; cost is linear in (horizon / step) x tour length.
    /// Candidates are scored at floor(horizon / step) steps, so a step
    /// comparable to the inter-cell drive blurs the intercept.
    /// (notes: pursuit-step-sec)
    double step_sec = 5.0;

    /// Seconds after which half the mass has drained off the tour into O; a
    /// half-life, so staleness degrades the answer continuously. <= 0 disables
    /// the hazard (the peer never leaves the tour).
    /// (notes: pursuit-offroute-half-life)
    double offroute_half_life_sec = 180.0;

    /// Bounds the prediction horizon, peer.age_sec + my drive (not the drive
    /// alone), in seconds. Candidates over it are dropped, not scored at a
    /// clamped horizon. (notes: pursuit-max-horizon)
    double max_horizon_sec = 600.0;

    /// Refuse below this probability. The floor is the legacy trail, and a
    /// prediction that cannot beat a coin flip against it is not an
    /// improvement — see the header. Zero would make every chase MDP-driven
    /// including the ones with no information behind them.
    double min_probability = 0.10;
  };

  /// What this robot last heard FIRST-HAND about the peer. Second-hand does not
  /// appear here and cannot: tours are not gossiped (§3.2, TeamWorld.msg), so a
  /// peer only ever reachable through a relay has no tour on record and this
  /// unit refuses — which is the documented degradation, not a gap.
  struct PeerTrack {
    /// Cell ids in visit order, exactly as they arrived in `my_tour`. Empty is
    /// the normal early-run state and the normal state with the allocator off.
    std::vector<int> tour;

    /// The peer's position when that tour was received. Used only to anchor
    /// the chain; with `have_position` false the chain starts at the tour head,
    /// which is where the peer said it was going next.
    double x = 0.0, y = 0.0;
    bool   have_position = false;

    /// Local age of the record, seconds. LOCAL: this is a receipt-to-now
    /// interval on our own clock, never a difference of two clocks' stamps —
    /// the same rule TeamModel and the rendezvous scheduler follow, and for the
    /// same reason (field clocks have been observed hours apart).
    double age_sec = 0.0;
  };

  /// Predict the intercept. `my_cell` is the cell this robot is standing in;
  /// -1 (off grid) is a refusal, because without it there is no travel time and
  /// the whole objective is a travel time.
  static PursuitTarget predict(const CellWorld& world,
                               const PeerTrack& peer,
                               int my_cell,
                               const Config& cfg);

  /// Advance the chain one step in place. p is index-aligned with tour and
  /// p_off is the absorbed mass; their sum is invariant.
  /// (notes: pursuit-step-conservation)
  static void step(const CellWorld& world, const std::vector<int>& tour,
                   std::vector<double>& p, double& p_off, const Config& cfg);

  /// Tour index whose cell centre is nearest (x, y), or -1 for an empty tour /
  /// unconfigured world. Straight-line, deliberately: this places the peer on
  /// its own route, and a roadmap distance would answer a question about
  /// driving that nobody asked.
  static int anchorIndex(const CellWorld& world, const std::vector<int>& tour,
                         double x, double y);
};

}  // namespace explo_planner
