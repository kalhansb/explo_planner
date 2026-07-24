#pragma once
/// @file target_queue.hpp
/// @brief Pure (no-ROS) queue of tree targets to perceptively exploit.
///
/// The exploitation behaviour is driven by a stream of tree targets. Today a
/// time-based scheduler publishes a preselected list on the TreeTarget topic;
/// a real detector can publish the identical message later. The node converts
/// each incoming TreeTarget into a TargetQueue::ingest() call, so this class is
/// deliberately ROS-free: it is just the data structure + lifecycle, and is
/// unit-tested in isolation (test_target_queue.cpp).
///
/// Lifecycle: a target is PENDING when ingested, becomes ACTIVE while the
/// planner circles it, and DONE once enough vantages are dwelled (success) or
/// no further vantage is reachable (partial). DONE targets are kept in the
/// queue (not erased) so re-reports of an already-finished tree dedup against
/// them; the small queue size makes this cheap.

#include <cstdint>
#include <deque>
#include <vector>

#include <Eigen/Core>

namespace explo_planner {

struct Target {
  uint32_t id = 0;
  Eigen::Vector3f center = Eigen::Vector3f::Zero();  ///< Trunk centre, map frame.
  float radius = 0.0f;   ///< Estimated trunk radius (m); <=0 treated as a point.
  float height = 0.0f;   ///< Optional trunk extent (m); informational.

  enum class Status : uint8_t { PENDING, ACTIVE, DONE };
  Status status = Status::PENDING;

  /// XY positions of vantages already dwelled at for this target — local
  /// dwells plus canonical ring positions merged from peers. Vantages are
  /// re-generated every plan tick, so "already visited" is tracked by proximity
  /// to these recorded positions (see isVantageVisited) rather than by a
  /// transient per-tick index. Z is ignored in the comparison.
  std::vector<Eigen::Vector3f> visited_vantages;

  /// Count of clear-LoS dwells credited to this target — the TEAM union, not
  /// just local dwells (peer dwells merge in via mergePeerDwells). The success
  /// criterion is reached once this hits min_vantages_required.
  int clear_los_dwells = 0;

  /// Bitmask of vantage indices dwelled with clear LoS (bit i = ring index i).
  /// The ring is deterministic from (center, radius), so indices are globally
  /// meaningful across robots; this is what gets broadcast in RobotIntent and
  /// what makes local + peer credit idempotent to merge (an index counts once
  /// no matter how many robots dwell it or how often the mask is re-received).
  /// Indices >= 32 fall back to the plain counter (no team sharing).
  uint32_t clear_mask = 0;
};

class TargetQueue {
public:
  TargetQueue() = default;

  /// Add a new target unless it duplicates an existing one. A candidate is a
  /// duplicate when it shares an id with any queued target (DONE included) OR
  /// its centre lies within `dedup_radius_m` (XY) of an existing target's
  /// centre — a re-reported tree. Returns true iff a new target was enqueued.
  bool ingest(uint32_t id, const Eigen::Vector3f& center, float radius,
              float height, float dedup_radius_m);

  /// True iff at least one target is not DONE (PENDING or ACTIVE).
  bool hasPending() const;

  /// Number of non-DONE targets.
  size_t pendingCount() const;

  /// Promote the next non-DONE target to ACTIVE and return it; returns the
  /// already-ACTIVE target if one exists, or nullptr if none remain.
  Target* activate();

  /// The currently ACTIVE target, or nullptr.
  Target* active();
  const Target* active() const;

  /// Mark the ACTIVE target DONE and clear the active slot. No-op if none.
  void markActiveDone();

  /// Record that the robot dwelled at a vantage of the ACTIVE target.
  /// `vantage_index` is the deterministic ring index of the vantage (< 32 for
  /// team credit sharing; -1 / out-of-range falls back to counter-only).
  /// A clear-LoS dwell on an index already credited (e.g. merged from a peer
  /// that dwelled it first) does not double-count.
  void recordVantageDwell(const Eigen::Vector3f& vantage_xy, bool los_clear,
                          int vantage_index = -1);

  /// Merge a peer's clear-LoS dwelled vantage-index mask into the target with
  /// `target_id` (team quota: the union of everyone's dwells counts toward
  /// success). `ring` is the locally generated vantage ring for that target —
  /// canonical positions for newly credited indices are appended to
  /// visited_vantages so isVantageVisited() skips angles a peer already
  /// captured. Idempotent; returns true iff any new index was credited.
  /// No-op on DONE or unknown targets.
  bool mergePeerDwells(uint32_t target_id, uint32_t peer_mask,
                       const std::vector<Eigen::Vector3f>& ring);

  /// True iff a (freshly generated) vantage at `vantage_xy` is within
  /// `visited_tol_m` (XY) of a vantage already dwelled at for the ACTIVE
  /// target. False if there is no active target.
  bool isVantageVisited(const Eigen::Vector3f& vantage_xy,
                        float visited_tol_m) const;

  size_t size() const { return targets_.size(); }
  const std::deque<Target>& targets() const { return targets_; }

private:
  // deque so push_back in ingest() never invalidates references/pointers to
  // existing elements (active() handed out to the node stays valid).
  std::deque<Target> targets_;
  int active_idx_ = -1;  ///< Index of the ACTIVE target, or -1 if none.
};

}  // namespace explo_planner
