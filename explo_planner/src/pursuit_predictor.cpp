#include "explo_planner/pursuit_predictor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "explo_planner/global_allocator.hpp"

namespace explo_planner {

namespace {

constexpr double kMmPerM = 1000.0;

/// Metres between two cells, through the allocator's own cost model.
///
/// Reused rather than reimplemented for the same reason the rendezvous
/// scheduler reuses routeCostMm: a second distance function is free to drift
/// from the first, and the symptom of that drift is a chase aimed at a cell the
/// allocator would never have put on the tour — invisible in every log, because
/// both numbers look reasonable in isolation. costMm also carries the
/// unreachable-cell fallback, so an over-aggressive edge probe degrades the
/// ranking instead of deleting nodes from the chain.
double cellDistM(const CellWorld& w, int a, int b) {
  return static_cast<double>(GlobalAllocator::costMm(w, a, b)) / kMmPerM;
}

/// P(the G transition fires this step) for tour node `i`.
///
/// A leg takes `dwell + drive` seconds, and a step is `step_sec` of it, so the
/// per-step advance probability is the ratio. That makes the sojourn time
/// geometric with the right MEAN — it is not a claim that the peer's dwell is
/// memoryless, it is the coarsest model with the correct first moment, which is
/// all a 5-second-resolution intercept can use.
///
/// Returns 0 at the tail: a peer at the end of its tour has nowhere on the
/// record to advance to. Its mass then sits on the last cell and drains only
/// through O, so a long-finished tour predicts nothing rather than predicting,
/// with confidence, that the peer is parked on its last cell forever.
double pGo(const CellWorld& w, const std::vector<int>& tour, size_t i,
           const PursuitPredictor::Config& cfg) {
  if (i + 1 >= tour.size()) return 0.0;
  const double drive = cellDistM(w, tour[i], tour[i + 1]) /
                       std::max(cfg.peer_speed_mps, 1e-6);
  const double leg = std::max(cfg.dwell_sec + drive, 1e-6);
  return std::min(1.0, std::max(0.0, cfg.step_sec / leg));
}

}  // namespace

int PursuitPredictor::anchorIndex(const CellWorld& world,
                                  const std::vector<int>& tour,
                                  double x, double y) {
  if (!world.configured() || tour.empty()) return -1;
  int    best = -1;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < tour.size(); ++i) {
    if (!world.grid().valid(tour[i])) continue;
    float cx = 0.0f, cy = 0.0f;
    world.grid().centre(tour[i], cx, cy);
    const double dx = static_cast<double>(cx) - x;
    const double dy = static_cast<double>(cy) - y;
    const double d2 = dx * dx + dy * dy;
    // Strict <, so the EARLIEST nearest node wins a tie. A tour that revisits
    // a cell would otherwise anchor at the later visit and skip the work in
    // between, predicting the peer ahead of itself.
    if (d2 < best_d2) { best_d2 = d2; best = static_cast<int>(i); }
  }
  return best;
}

void PursuitPredictor::step(const CellWorld& world,
                            const std::vector<int>& tour,
                            std::vector<double>& p, double& p_off,
                            const Config& cfg) {
  if (tour.empty() || p.size() != tour.size()) return;

  // O first, as a hazard on ALL surviving mass, then G/I split the remainder.
  // Order matters and this one is deliberate: applying the hazard only to the
  // mass that stayed put would make a fast tour immune to abandonment, which
  // is exactly backwards — the peer most likely to have re-planned is the one
  // that has had time to finish what it was doing.
  // A non-positive half-life is the hazard switched OFF, not one clamped to an
  // epsilon — see the Config comment. Written as an exact 1.0 rather than an
  // exp() of something tiny, because the chain's test case is an exact
  // binomial and a survive of 1 - 3e-9 is not 1.
  const double survive = cfg.offroute_half_life_sec > 0.0
      ? std::exp(-std::log(2.0) * cfg.step_sec / cfg.offroute_half_life_sec)
      : 1.0;

  std::vector<double> next(p.size(), 0.0);
  for (size_t i = 0; i < p.size(); ++i) {
    if (p[i] <= 0.0) continue;
    const double live = p[i] * survive;
    p_off += p[i] - live;
    const double g = pGo(world, tour, i, cfg);
    next[i] += live * (1.0 - g);
    if (g > 0.0) next[i + 1] += live * g;
  }
  p.swap(next);
}

PursuitTarget PursuitPredictor::predict(const CellWorld& world,
                                        const PeerTrack& peer,
                                        int my_cell,
                                        const Config& cfg) {
  PursuitTarget out;

  // Every refusal below is a fall-through to the legacy trail, and each one
  // names itself: "the model had nothing to say" and "the model was never
  // asked" are different runs, and a single empty answer for both would make
  // an arm that silently never predicted look like one that predicted badly.
  if (!world.configured()) { out.refused = "cell world is not configured"; return out; }
  if (peer.tour.empty())   { out.refused = "no tour on record"; return out; }
  if (!world.grid().valid(my_cell)) {
    out.refused = "own cell is off the grid";
    return out;
  }
  if (!(cfg.step_sec > 0.0) || !(cfg.peer_speed_mps > 0.0) ||
      !(cfg.my_speed_mps > 0.0)) {
    out.refused = "predictor is disabled by configuration";
    return out;
  }
  if (!(peer.age_sec >= 0.0) || !std::isfinite(peer.age_sec)) {
    // A negative age is a clock that ran backwards, not a very fresh record.
    // Believing it would propagate the chain a negative number of steps.
    out.refused = "record age is not a usable interval";
    return out;
  }

  // Anchor. Without a position the chain starts at the tour head — the peer's
  // focus cell, i.e. where it told us it was going next — which is the most
  // this robot knows and is never worse than assuming index 0 arbitrarily,
  // because index 0 IS that cell.
  const int anchor = peer.have_position
      ? anchorIndex(world, peer.tour, peer.x, peer.y)
      : (world.grid().valid(peer.tour.front()) ? 0 : -1);
  if (anchor < 0) {
    out.refused = "no valid cell on the peer's tour";
    return out;
  }
  out.anchor_index = anchor;

  // --- candidate horizons ------------------------------------------------
  //
  // Scored at (record age + MY drive to it), not at the record age alone:
  // §3.7's objective is to intercept where the peer will be WHEN I GET THERE,
  // and the two differ by minutes at these speeds. Nodes BEFORE the anchor are
  // still candidates — the peer may have been driving toward the anchor rather
  // than away from it, and dropping them would bias every intercept forward.
  struct Cand { size_t i; int cell; long long travel_ms; long long horizon_ms;
                size_t stepk; };
  std::vector<Cand> cands;
  cands.reserve(peer.tour.size());
  size_t max_stepk = 0;
  for (size_t i = 0; i < peer.tour.size(); ++i) {
    const int c = peer.tour[i];
    if (!world.grid().valid(c)) { ++out.rejected_invalid; continue; }
    const double travel_s = cellDistM(world, my_cell, c) / cfg.my_speed_mps;
    // The AGE is inside the horizon, so a record older than max_horizon_sec
    // rejects every candidate and the whole prediction refuses. That is the
    // intended reading of the parameter: it bounds how far ahead the model is
    // allowed to claim anything, and a stale record is already that far ahead.
    const double horizon_s = peer.age_sec + travel_s;
    if (horizon_s > cfg.max_horizon_sec) { ++out.rejected_horizon; continue; }
    const size_t k = static_cast<size_t>(horizon_s / cfg.step_sec);
    max_stepk = std::max(max_stepk, k);
    cands.push_back({i, c,
                     static_cast<long long>(std::llround(travel_s * 1000.0)),
                     static_cast<long long>(std::llround(horizon_s * 1000.0)),
                     k});
  }
  out.candidates = static_cast<int>(cands.size());
  if (cands.empty()) {
    out.refused = "every tour cell is off the grid or beyond the horizon";
    return out;
  }

  // --- one forward pass, snapshotting each candidate at its own step ------
  //
  // A fresh chain per candidate would be O(cands x steps x tour) for exactly
  // the same numbers; the distribution does not depend on which candidate is
  // being scored, only on how far it is propagated. So propagate once and read
  // each candidate off as the pass goes by it.
  std::vector<double> p(peer.tour.size(), 0.0);
  p[static_cast<size_t>(anchor)] = 1.0;
  double p_off = 0.0;

  std::vector<double> score(cands.size(), 0.0);
  std::vector<double> on_route(cands.size(), 0.0);
  for (size_t k = 0; k <= max_stepk; ++k) {
    if (k > 0) step(world, peer.tour, p, p_off, cfg);
    for (size_t j = 0; j < cands.size(); ++j) {
      if (cands[j].stepk != k) continue;
      score[j]    = p[cands[j].i];
      on_route[j] = 1.0 - p_off;
    }
  }

  // Argmax, with a total tie-break: probability, then the SOONER arrival, then
  // the earlier tour position, then the lower cell id. Sooner-first is not
  // cosmetic — two cells the peer is equally likely to be in are not equally
  // good chases, because the near one re-forms the link earlier and leaves
  // budget for a second attempt.
  size_t best = 0;
  bool   have = false;
  for (size_t j = 0; j < cands.size(); ++j) {
    if (!have) { best = j; have = true; continue; }
    const double dp = score[j] - score[best];
    if (dp > 0.0) { best = j; continue; }
    if (dp < 0.0) continue;
    if (cands[j].horizon_ms != cands[best].horizon_ms) {
      if (cands[j].horizon_ms < cands[best].horizon_ms) best = j;
      continue;
    }
    if (cands[j].i != cands[best].i) {
      if (cands[j].i < cands[best].i) best = j;
      continue;
    }
    if (cands[j].cell < cands[best].cell) best = j;
  }

  out.p            = score[best];
  out.p_on_route   = on_route[best];
  out.my_travel_ms = cands[best].travel_ms;
  out.horizon_ms   = cands[best].horizon_ms;
  out.win_index    = static_cast<int>(cands[best].i);

  // The floor rule, last: everything above is reported even when the answer is
  // refused, so a log can show WHAT was rejected and how close it came. A
  // refusal with p = 0.09 against a 0.10 threshold and one with p = 0.001 are
  // different findings about the model.
  if (!(out.p >= cfg.min_probability)) {
    out.refused = "best intercept probability below min_probability";
    return out;
  }

  out.cell = cands[best].cell;
  world.grid().centre(out.cell, out.x, out.y);
  return out;
}

}  // namespace explo_planner
