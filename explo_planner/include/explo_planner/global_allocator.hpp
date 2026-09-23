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
/// Moved comments: doc/global_allocator_notes.md

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

  /// The robot has left the frontier (homing or done, TeamWorld mode >=
  /// MODE_HOMING); a strict superset of finished. It decides only membership in
  /// the allocation problem; finished keeps its other readers.
  /// (notes: alloc-off-frontier)
  bool off_frontier = false;
};

/// One ordered tour per robot, plus bookkeeping. An empty tour is normal
/// (nothing left, or every cell masked away): callers fall back to unrestricted
/// planning rather than treating it as a fault.
/// (notes: alloc-empty-tour-is-normal)
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

  /// FNV-1a digest of the problem, not the answer: vehicles by id, candidate
  /// cells, edgeHash(), comms_mask, max_candidates; polish_passes deliberately
  /// excluded. Computed inside solve(). Instrumentation only.
  /// (notes: alloc-hash-problem-digest)
  unsigned int alloc_hash = 0;

  /// The edgeHash() part of alloc_hash, logged separately: robots differing
  /// here had different traversability; agreeing here but not on alloc_hash
  /// means fleet, candidates or config differed.
  /// (notes: alloc-edge-hash-attribution)
  unsigned int edge_hash = 0;

  /// The first cell of robot `self_id`'s tour — the focus cell — or -1 if it
  /// has no tour. This is the only value doPlan needs from the whole solve.
  int focusFor(int self_id, const std::vector<AllocRobot>& robots) const;
};

class GlobalAllocator {
public:
  struct Config {
    /// An out-of-comms robot may only be assigned cells whose known_by already
    /// includes it (mTARE's INF-masking). Default false: the plain solve is
    /// unmasked. (notes: alloc-comms-mask)
    bool comms_mask = false;

    /// 2-opt passes per tour after assignment, capped because it runs every
    /// planning cycle; 0 disables. Takes the first improving move in (i, j)
    /// order, so it stays deterministic across robots.
    /// (notes: alloc-polish-passes)
    int polish_passes = 2;

    /// Refuse, with a reason, above this many candidate cells instead of
    /// truncating: a truncated set would differ between robots. A grid-sized
    /// candidate set means something upstream is wrong.
    /// (notes: alloc-max-candidates-refusal)
    int max_candidates = 256;
  };

  /// Solve over world's statuses, known_by masks and distances; robots in any
  /// order (sorted by id inside). Candidates are EXPLORING and
  /// EXPLORING_BY_OTHERS cells, both, so robots solve the same problem.
  /// (notes: alloc-solve-candidates)
  static Allocation solve(const CellWorld& world,
                          const std::vector<AllocRobot>& robots,
                          const Config& cfg);

  /// Cost between two cells in mm. When b is unreachable from a it falls back
  /// to straight-line centroid distance, so the cell drops in rank instead of
  /// leaving the problem. (notes: alloc-costmm-fallback)
  static long long costMm(const CellWorld& world, int a, int b);

  /// Open-route cost (no closing leg) of tour driven from start, in mm. The
  /// rendezvous scheduler uses this same function for its makespan difference;
  /// do not keep a private copy. (notes: alloc-route-cost-open)
  static long long routeCostMm(const CellWorld& world, int start,
                               const std::vector<int>& tour);
};

/// Should a focus cell that keeps yielding no admissible candidate be demoted
/// to COVERED? skips: ticks it was the focus but the goal came from elsewhere;
/// k: the threshold. status must be read after a fresh census.
/// (notes: alloc-stale-focus-demotion)
bool shouldDemoteStaleFocus(int skips, int k, CellStatus status);

} // namespace explo_planner
