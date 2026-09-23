// Moved comments: doc/explo_planner_code_notes.md
#include "explo_planner/reconnect_gate.hpp"

#include <algorithm>

namespace explo_planner {

int unsharedCellCount(const CellWorld& world,
                      const std::vector<MissingPeer>& missing) {
  if (!world.configured()) return 0;

  // One mask holding every peer we would actually be going out to meet. A cell
  // counts as unshared if ANY of them is missing from it, so the test is
  // "peers_mask is not a subset of known_by", which is one AND per cell rather
  // than a loop over peers per cell.
  uint32_t peers_mask = 0;
  for (const MissingPeer& p : missing) {
    // Finished peers are ignored (§3.6): they will never explore again, so
    // telling them anything changes no plan. Ids outside the mask's width
    // cannot be represented and are dropped rather than aliased onto bit
    // (id % 32), which would silently credit one peer with another's knowledge.
    if (p.finished || p.id < 0 || p.id >= 32) continue;
    peers_mask |= (1u << p.id);
  }
  if (peers_mask == 0) return 0;

  int n = 0;
  for (int id = 0; id < world.size(); ++id) {
    const CellWorld::Cell& c = world.cell(id);
    // UNSEEN carries no information: "I have not looked there either" is not
    // news, and counting it would make the gate vacuously true over the whole
    // unexplored map for the entire run.
    if (c.status == CellStatus::UNSEEN) continue;
    if ((peers_mask & ~c.known_by) != 0) ++n;
  }
  return n;
}

long long makespanMm(const Allocation& a) {
  long long m = 0;
  for (const long long c : a.costs_mm) m = std::max(m, c);
  return m;
}

GateVerdict evaluateReconnectGate(const CellWorld& world,
                                  const std::vector<AllocRobot>& robots,
                                  const std::vector<MissingPeer>& missing,
                                  const GlobalAllocator::Config& cfg) {
  GateVerdict v;

  if (!world.configured()) {
    v.refused  = "world-unconfigured";
    v.dispatch = true;
    return v;
  }

  // An empty missing list means the trigger's beacon view and
  // TeamModel::inComms disagree, not that the partner knows everything: fail
  // open with a named refusal, never a silent stay.
  // (notes: gate-empty-missing-fails-open)
  if (missing.empty()) {
    v.refused  = "no-missing-peer-identified";
    v.dispatch = true;
    return v;
  }

  // Price only unfinished, addressable peers. Finished peers are a computed no
  // and can leave a clean no-dispatch; an id outside [0,32) is not
  // representable in known_by, so the gate fails open.
  // (notes: gate-priced-peer-set)
  std::vector<MissingPeer> live;
  for (const MissingPeer& p : missing) {
    if (p.id < 0 || p.id >= 32) {
      v.refused  = "peer-id-out-of-mask-range";
      v.dispatch = true;
      return v;
    }
    if (p.finished) continue;
    live.push_back(p);
  }

  // --- 1. knowledge ------------------------------------------------------
  v.unshared_cells = unsharedCellCount(world, live);
  v.knowledge      = v.unshared_cells > 0;
  if (!v.knowledge) {
    // The one clean suppression: every missing peer already holds the current
    // status of every cell we have an opinion about. Nothing said here changes
    // any plan, so the leg is pure cost and there is no point pricing it.
    v.dispatch = false;
    return v;
  }

  // --- 2. the leg --------------------------------------------------------
  // Our own vehicle, by id. selfId() is the world's owner, which is the robot
  // deciding — the allocator's `robots` may list it anywhere.
  const int self_id = world.selfId();
  int self_cell = -1;
  bool self_present = false;
  for (const AllocRobot& r : robots) {
    if (r.id != self_id) continue;
    self_present = true;
    self_cell    = r.cell;
  }
  if (!self_present || !world.grid().valid(self_cell)) {
    v.refused  = "self-unlocatable";
    v.dispatch = true;
    return v;
  }

  // Cost out to the nearest missing peer we can locate. NEAREST, not the sum
  // and not the furthest: the manoeuvre goes to one of them, and pricing it as
  // if it visited all would suppress every multi-peer case on a cost the
  // dispatch would never pay. With N<=3 and one peer this is just the one.
  long long leg = -1;
  int leg_peer_id = -1;
  for (const MissingPeer& p : live) {
    if (!world.grid().valid(p.cell)) continue;
    const long long d = GlobalAllocator::costMm(world, self_cell, p.cell);
    if (leg < 0 || d < leg) { leg = d; leg_peer_id = p.id; }
  }
  if (leg < 0) {
    // Someone is missing but not located: pursuit's own fallback ladder handles
    // that. The gate has no basis to price it and must not turn unknown into
    // do-not-go, so it fails open. (notes: gate-peer-position-unknown)
    v.refused  = "peer-position-unknown";
    v.dispatch = true;
    return v;
  }
  v.leg_mm = leg;
  v.leg_peer_id = leg_peer_id;

  // --- 3. the two futures ------------------------------------------------
  // The two solves differ only in whether the peer leg was priced to can be
  // given cells it has never heard of; value and cost sides must name that same
  // peer. cfg.comms_mask is ignored. (notes: gate-two-futures-one-peer)
  std::vector<AllocRobot> apart = robots;
  for (AllocRobot& r : apart) {
    for (const MissingPeer& p : live) {
      if (r.id == p.id) r.in_comms = false;
    }
  }
  // re differs from apart in exactly one vehicle, the peer leg was priced to.
  // Built from apart, not robots, because robots carries the caller's in_comms
  // bits for the other missing peers. (notes: gate-re-vehicle-set)
  std::vector<AllocRobot> re = apart;
  for (AllocRobot& r : re) {
    if (r.id == leg_peer_id) r.in_comms = true;
  }

  // Both solves run with comms_mask on and differ only in the vehicle set: with
  // the flag clear GlobalAllocator skips masking wholesale, so only this
  // expresses exactly one peer coming back. (notes: gate-both-solves-masked)
  GlobalAllocator::Config no_cfg = cfg;
  no_cfg.comms_mask = true;
  GlobalAllocator::Config re_cfg = cfg;
  re_cfg.comms_mask = true;

  const Allocation no_plan = GlobalAllocator::solve(world, apart, no_cfg);
  if (!no_plan.refused.empty()) {
    v.refused  = "no-comms-solve:" + no_plan.refused;
    v.dispatch = true;
    return v;
  }
  const Allocation re_plan = GlobalAllocator::solve(world, re, re_cfg);
  if (!re_plan.refused.empty()) {
    v.refused  = "assume-comms-solve:" + re_plan.refused;
    v.dispatch = true;
    return v;
  }

  // Structurally zero while self is in the vehicle set: this robot is always
  // in_comms, so it is exempt from the known-by test and can take any cell.
  // Kept so the column keeps its meaning.
  // (notes: gate-unassigned-structurally-zero)
  v.unassigned = static_cast<int>(no_plan.unassigned.size());
  v.c_no_mm    = makespanMm(no_plan);
  v.c_re_mm    = leg + makespanMm(re_plan);

  // STRICT. See the header: `<=` would dispatch on every tie, and the largest
  // tie class is the degenerate one where there is nothing left to explore and
  // both makespans are zero.
  v.dispatch = v.c_re_mm < v.c_no_mm;
  return v;
}

}  // namespace explo_planner
