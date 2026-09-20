#pragma once
/// @file global_allocator.hpp
/// @brief Solve-same-take-own cell allocation. Every robot merges what it has
///        heard, solves the SAME problem over the shared cell world, and drives
///        only its own tour. See docs/mtare_evolution_plan.md §3.4.
///
/// The load-bearing mTARE idea, and the reason there is no auction, no leader
/// and no negotiation anywhere in this file: agreement is a CONSEQUENCE of both
/// robots running identical arithmetic over a converged world, not something
/// they talk their way into. Divergent world views produce divergent
/// allocations and that is tolerated — everyone re-solves every cycle, and
/// MinPos deconfliction plus the goal blacklists remain underneath as the
/// collision backstop. The worst case is duplicated cell work, which is the
/// redundancy the campaign already measures.
///
/// That premise puts an unusual demand on this code: it must be deterministic
/// ACROSS PROCESSES, not merely reproducible within one. Two robots running the
/// same binary over equal worlds must produce bit-identical tours, so:
///
///   * every cost is integer-quantised (millimetres) before any comparison.
///     Floating-point addition is not associative, and two robots that
///     accumulate the same route in a different order can disagree in the last
///     bit — which, at a tie, flips an entire assignment. Quantising at the
///     leaf makes the comparison exact.
///   * every tie breaks on (cell id, robot id, position), a total order over
///     things both robots agree on. "Whichever came first" is not a tie-break;
///     it is a dependency on container iteration order.
///   * no clock, no random, no address-derived value participates. There is
///     nothing to seed and nothing to drift.
///
/// WHAT THIS DELIBERATELY IS NOT: a good VRP solver. It replaces mTARE's 80 ms
/// OR-Tools call with greedy makespan-balanced insertion plus a capped 2-opt.
/// A better tour that two robots computed differently is worth less here than a
/// mediocre tour they both computed identically.

#include <cstdint>
#include <string>
#include <vector>

#include "explo_planner/cell_world.hpp"

namespace explo_planner {

/// One vehicle in the allocation problem.
struct AllocRobot {
  int  id       = -1;     ///< fleet id, the index used by known_by masks
  int  cell     = -1;     ///< cell it is in now; -1 drops it from the problem
  bool in_comms = true;   ///< false engages the no-comms mask (see Config)
  bool finished = false;  ///< true removes it and returns its cells to the pool
};

/// The result: one ordered tour per robot that got any cells, plus enough
/// bookkeeping to explain an empty one. An empty tour is a NORMAL outcome
/// (nothing left to explore, or every remaining cell masked away from this
/// robot) and callers must fall back to unrestricted planning rather than
/// treating it as a fault.
struct Allocation {
  /// tours[i] belongs to robots[i] of the input, index-aligned so a caller
  /// never has to search by id. Empty vector = this robot was given nothing.
  std::vector<std::vector<int>> tours;

  /// Route cost per robot in the quantised unit (mm), index-aligned as above.
  std::vector<long long> costs_mm;

  /// Candidate cells that no robot could legally take — every vehicle was
  /// either masked out of them or dropped. Reported rather than silently
  /// discarded: a large number here means the comms mask is starving the
  /// allocation, which looks identical to "there was nothing to do".
  std::vector<int> unassigned;

  /// Non-empty when the problem was refused wholesale. Distinguishes "solved,
  /// and the answer is nothing to do" from "could not solve", which otherwise
  /// present identically as empty tours.
  std::string refused;

  /// R3 / §3.6. FNV-1a over the PROBLEM this solve was given, not over its
  /// answer: the vehicle set sorted by id as `(id, cell, in_comms, finished)`,
  /// then the candidate cell ids in ascending order, then the world's
  /// `edgeHash()`, then `cfg.comms_mask` and `cfg.max_candidates`. Two robots
  /// that solve the same problem must produce the same value; two robots that
  /// produce different tours with the SAME value have found a determinism bug
  /// in the solver, and two robots with different values were never solving
  /// the same problem and their disagreement is not the allocator's fault.
  ///
  /// `cfg.polish_passes` is deliberately NOT in it, and the two that are were
  /// added 2026-09-18. The line is drawn at whether a field changes the
  /// PROBLEM or the approach to it: comms_mask changes the feasible assignment
  /// set, max_candidates decides refused-versus-solved, and 2-opt passes change
  /// only the tours. That last exclusion is what keeps the second reading above
  /// meaningful -- two robots differing only in polish_passes really are
  /// getting different answers to one problem, and folding it in would restate
  /// that as "different problems" and lose it. See the digest block in
  /// global_allocator.cpp for the full argument, and
  /// AllocHash.SolverConfigSplitsOnWhatChangesTheProblem for the assertions.
  ///
  /// That distinction is the whole point. `shared_hash` was being used as the
  /// join key for "both robots saw the same world", and it cannot bear that --
  /// for a STRUCTURAL reason, which is the one to rely on: `shared_hash`
  /// covers cell statuses, while the problem also contains a vehicle set (not
  /// part of the world at all) and a cost matrix (never exchanged). Equal
  /// `shared_hash` therefore does not mean the same problem, so restricting an
  /// agreement analysis to it does not restrict it to the same problem, and
  /// every such analysis to date is unsound for that reason.
  ///
  /// An earlier version of this comment quantified the leak as "4.5% at N=2
  /// and 23% at N=4". **Do not cite those numbers.** They could not be
  /// reproduced from the banked campaigns -- no denominator, filter or time
  /// window was recorded with them, five reconstructions disagreed and three
  /// of the five inverted the N ordering. A leak that looks like a clean
  /// team-size gradient is also the exact shape of an artifact this project
  /// has been caught by before. The structural argument above needs no
  /// measurement and is not weakened by withdrawing one.
  ///
  /// Computed INSIDE solve(), from the same values the solve used, rather than
  /// re-derived by the caller. A digest of the inputs that is assembled a
  /// second time somewhere else is a digest of a second thing that is believed
  /// to be equal, and drift between the two would be invisible in exactly the
  /// way this is supposed to make visible.
  ///
  /// Pure instrumentation. Nothing branches on it.
  unsigned int alloc_hash = 0;

  /// The `edgeHash()` component of `alloc_hash`, logged separately so a
  /// disagreement can be attributed. If two robots differ in `alloc_hash` but
  /// agree here, they disagreed about the fleet, the candidate set, or one of
  /// the two config terms; if they differ here too, their local maps produced
  /// different traversability and no amount of status exchange would have
  /// fixed it.
  ///
  /// That third arm is checkable without a second digest WHENEVER P3 IS ON:
  /// both values are written to the run params (`global_alloc_comms_mask`,
  /// `global_alloc_max_candidates`), so compare those across the two robots'
  /// logs and eliminate them before reaching for the fleet or the candidates.
  /// Every arm of every campaign runs global_alloc_enable=true, so in practice
  /// they are always there.
  ///
  /// WITH P3 OFF THEY ARE BOTH ABSENT, and this said "written to the run
  /// params" flat until 2026-09-18. The two addParam calls sit inside `if
  /// (global_alloc_enable_)` (explo_planner_node.cpp), while the solver is
  /// still reached by the §3.6 gate and by the rendezvous scheduler, which
  /// re-read max_candidates on their own paths and log nothing;
  /// `global_alloc_comms_mask` is not even declared there, on purpose, because
  /// the gate overwrites it per solve. So an allocator-off configuration
  /// produces edge_hash values with no record of the config that made them.
  /// Nothing to fix while no such campaign exists — but do not read the
  /// absence of the params as "the defaults were used".
  unsigned int edge_hash = 0;

  /// The first cell of robot `self_id`'s tour — the focus cell — or -1 if it
  /// has no tour. This is the only value doPlan needs from the whole solve.
  int focusFor(int self_id, const std::vector<AllocRobot>& robots) const;
};

class GlobalAllocator {
public:
  struct Config {
    /// Apply the no-comms mask: a robot that is out of comms may only be
    /// assigned cells whose known_by already includes it. The port of mTARE's
    /// INF-masking in GetDistanceMatricesNoComms, and the thing that makes an
    /// `assume_comms` counterfactual mean something — the cost gap between a
    /// masked and an unmasked solve IS the reconnection value P4 gates on.
    ///
    /// Default false, so the plain solve is the unmasked one.
    bool comms_mask = false;

    /// 2-opt passes over each tour after assignment. Capped because this runs
    /// every planning cycle and an uncapped local search has no bound anyone
    /// has measured; 0 disables it. The polish is deterministic (first
    /// improving move in (i, j) order, not best), so it cannot become a source
    /// of cross-robot disagreement.
    int polish_passes = 2;

    /// Refuse to solve above this many candidate cells. The assignment loop is
    /// O(n^2 * robots * tour) and n is the EXPLORING set, not the whole grid —
    /// typically tens. A grid-sized candidate set means something upstream is
    /// wrong (every cell stuck EXPLORING because a threshold is unreachable in
    /// this world — see cell_world.hpp's Config notes), and grinding through it
    /// for seconds inside a planning tick would present as a hung planner. It
    /// is a refusal, with a reason, rather than a silent truncation to the
    /// first N: a truncated allocation is one both robots would compute
    /// differently the moment their candidate sets differ by one cell.
    int max_candidates = 256;
  };

  /// Solve. `world` supplies the cell statuses, the known_by masks and the
  /// distances; `robots` is the vehicle set in any order (it is sorted
  /// internally by id, so the caller's order cannot affect the result).
  ///
  /// Candidate cells are those with status EXPLORING or EXPLORING_BY_OTHERS.
  /// Both, per §3.4: restricting to first-hand EXPLORING would have the two
  /// robots solving provably different problems immediately after every
  /// exchange, which destroys the solve-same premise this whole design rests
  /// on. A cell one robot covered itself and one it learned about are the same
  /// work to divide.
  static Allocation solve(const CellWorld& world,
                          const std::vector<AllocRobot>& robots,
                          const Config& cfg);

  /// Cost between two cells in the quantised unit, exposed because it is the
  /// determinism-critical primitive and testing it through solve() alone would
  /// leave the quantisation boundary untested.
  ///
  /// Falls back to straight-line centroid distance when the cell graph reports
  /// `b` unreachable from `a`, so an over-aggressive edge probe DEGRADES the
  /// ranking rather than removing cells from consideration entirely. An
  /// unreachable cell that vanished from the problem would be a cell no robot
  /// is ever sent to clear, and it would stay EXPLORING forever.
  static long long costMm(const CellWorld& world, int a, int b);

  /// Open-route cost of `tour` driven from cell `start`, in the quantised unit.
  /// Open, not closed: the robots are not coming back, so there is no closing
  /// leg — adding one would inflate every cost by a term depending only on the
  /// tour's last cell, biasing the makespan balance toward tours that happen to
  /// end near their origin.
  ///
  /// Exposed because §3.5's rendezvous objective is a DIFFERENCE OF MAKESPANS,
  /// and a difference is only meaningful if both sides come from the same cost
  /// function. A second, private copy in the scheduler would be free to drift
  /// from this one — and the symptom of that drift is a penalty that ranks
  /// cells the allocator would not, which is invisible in every log.
  static long long routeCostMm(const CellWorld& world, int start,
                               const std::vector<int>& tour);
};

/// The staleness rule's decision (§3.4): should a focus cell that keeps
/// producing no admissible candidate be written off as COVERED?
///
/// A free function, and not simply an `if` in doPlan, because of what the
/// answer costs when it is wrong. A demotion is a FIRST-HAND COVERED, and the
/// merge guard exists precisely to stop a peer lowering one of those again —
/// so a spurious demotion writes that ground off for the whole team, for the
/// rest of the run, with no path back. That is a decision worth being able to
/// test at its boundary rather than one to read out of a 900-line function.
///
/// `skips` counts consecutive planning ticks on which this cell was the focus
/// and the goal came from somewhere else. `k` is the threshold. `status` must
/// be the status read AFTER a fresh census: the whole point of the re-measure
/// is that a cell which has quietly been cleared must not be demoted, and
/// passing the pre-census status would silently remove that protection while
/// leaving every caller looking correct.
bool shouldDemoteStaleFocus(int skips, int k, CellStatus status);

} // namespace explo_planner
