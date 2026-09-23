#pragma once
// The gen-34 in-process harness world (DESIGN_gen34.md §8.10 step 4): N
// TeamCores, a 2-D toy world and a radio with range, trunks, drops and backlog.
//
// What is modelled, and what is not:
//   * Robots move in straight lines at a fixed speed toward whatever their
//     drive says. Trunks block the radio, not the robots. Exploring is a walk
//     between random waypoints in the robot's own sector; "finished" latches at
//     a per-robot time drawn from the seed.
//   * Map frames are numbered per sender, as scovox does. Each directed link is
//     a reliable FIFO that drains a fixed number of frames per second while the
//     link is up, with a small per-frame loss (a gap) and a capped backlog.
//     A robot's own dscovox sees its own frames at once.
//   * Beacons are best-effort: delivered in the same second when the link is
//     up, less a per-beacon drop rate. The emulator's best_effort_priority is
//     assumed on, so a map backlog does not starve them.
//   * A robot can be made stuck (it stops moving) from a given time.
// The world is deterministic given the seed.

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "explo_planner/fleet_identity.hpp"
#include "explo_planner/team_core.hpp"

namespace toy {

using explo_planner::gen34::Activity;
using explo_planner::gen34::Arm;
using explo_planner::gen34::Beacon;
using explo_planner::gen34::ChaseGateVerdict;
using explo_planner::gen34::ChaseGateView;
using explo_planner::gen34::DriveKind;
using explo_planner::gen34::PlanSolve;
using explo_planner::gen34::TeamConfig;
using explo_planner::gen34::TeamCore;
using explo_planner::gen34::TeamEvent;
using explo_planner::gen34::TeamOracle;
using explo_planner::gen34::TeamRobotView;
using explo_planner::gen34::TickInputs;
using explo_planner::gen34::TickOutputs;
using explo_planner::gen34::Vec2;
using explo_planner::gen34::dist;

struct Trunk {
  Vec2 c;
  double r = 0.4;
};

inline double segPointDist(const Vec2& a, const Vec2& b, const Vec2& p) {
  const double vx = b.x - a.x, vy = b.y - a.y;
  const double L2 = vx * vx + vy * vy;
  double t = L2 > 1e-12 ? ((p.x - a.x) * vx + (p.y - a.y) * vy) / L2 : 0.0;
  t = std::max(0.0, std::min(1.0, t));
  return std::hypot(a.x + t * vx - p.x, a.y + t * vy - p.y);
}

struct World {
  double size = 100.0;        // square [0,size]^2
  double cell = 10.0;
  double range = 30.0;
  std::vector<Trunk> trunks;
  bool blocked(const Vec2& a, const Vec2& b) const {
    for (const Trunk& t : trunks)
      if (segPointDist(a, b, t.c) < t.r) return true;
    return false;
  }
  bool linkUp(const Vec2& a, const Vec2& b) const {
    return dist(a, b) <= range && !blocked(a, b);
  }
  int cellAt(const Vec2& p) const {
    const int n = static_cast<int>(size / cell);
    int cx = std::min(n - 1, std::max(0, static_cast<int>(p.x / cell)));
    int cy = std::min(n - 1, std::max(0, static_cast<int>(p.y / cell)));
    return cy * n + cx;
  }
  Vec2 cellCenter(int id) const {
    const int n = static_cast<int>(size / cell);
    return {(id % n + 0.5) * cell, (id / n + 0.5) * cell};
  }
};

class ToyOracle : public TeamOracle {
 public:
  ToyOracle(const World* w, std::mt19937* rng, const double* now)
      : w_(w), rng_(rng), now_(now) {}
  PlanSolve solvePlan(const std::vector<TeamRobotView>& team) override {
    ++solves;
    if (solve_hook) return solve_hook(team);
    PlanSolve s;
    Vec2 c{0, 0};
    int k = 0;
    for (const auto& r : team)
      if (r.known) { c.x += r.position.x; c.y += r.position.y; ++k; }
    if (k == 0) { s.refused = "no-robots"; return s; }
    c.x /= k;
    c.y /= k;
    // Early in the run there are no tours: provisional, as the node's centroid.
    s.provisional = *now_ < provisional_until;
    if (!s.provisional) {
      // "Solved": a cell toward the world centre from the team centroid.
      c.x = 0.5 * (c.x + w_->size / 2);
      c.y = 0.5 * (c.y + w_->size / 2);
    }
    s.ok = true;
    s.cell = w_->cellAt(c);
    s.center = w_->cellCenter(s.cell);
    return s;
  }
  double pathDistance(const Vec2& from, int, const Vec2& center) override {
    return dist(from, center) * 1.3;
  }
  bool standable(const Vec2& p) override {
    if (p.x < 0 || p.y < 0 || p.x > w_->size || p.y > w_->size) return false;
    for (const Trunk& t : w_->trunks)
      if (dist(p, t.c) < t.r + 0.6) return false;
    return true;
  }
  bool lineOfSight(const Vec2& a, const Vec2& b) override { return !w_->blocked(a, b); }
  ChaseGateVerdict chaseGate(const std::vector<ChaseGateView>& missing) override {
    ChaseGateVerdict v;
    ++gate_calls;
    std::uniform_real_distribution<double> u(0, 1);
    const double r = u(*rng_);
    if (r < gate_refuse_p) { v.refused = "toy-refused"; return v; }
    v.dispatch = r < gate_refuse_p + gate_dispatch_p;
    v.peer = missing.empty() ? -1 : missing.front().id;
    return v;
  }
  bool intercept(const ChaseGateView& peer, Vec2* out) override {
    if (!intercept_ok || !peer.known) return false;
    // Toward the peer's sector centre, a guess the predictor might make.
    *out = {0.5 * (peer.last_position.x + w_->size / 2),
            0.5 * (peer.last_position.y + w_->size / 2)};
    return true;
  }

  std::function<PlanSolve(const std::vector<TeamRobotView>&)> solve_hook;
  long long solves = 0;
  double provisional_until = 120.0;
  double gate_refuse_p = 0.05;
  double gate_dispatch_p = 0.7;
  bool intercept_ok = true;
  long long gate_calls = 0;

 private:
  const World* w_;
  std::mt19937* rng_;
  const double* now_;
};

struct Robot {
  std::unique_ptr<ToyOracle> oracle;
  std::unique_ptr<TeamCore> core;
  Vec2 pos, home;
  double finish_at = 1e18;
  bool finished = false;
  double stuck_from = 1e18;
  Vec2 wp;
  bool have_wp = false;
  Vec2 sector_lo, sector_hi;
  uint64_t sent = 0;
  std::vector<uint64_t> rcvd;   // by source
  std::vector<uint64_t> gaps;   // by source
  TickOutputs out;
  double next_frame = 0.0;
  std::vector<TeamEvent> events;
  bool keep_events = false;
  // Activity trace for the property checks: (time, activity).
  std::vector<std::pair<double, Activity>> trace;
};

struct Link {
  std::deque<uint64_t> q;       // reliable frames i -> j
};

struct Params {
  int n = 2;
  Arm arm = Arm::kOff;
  uint32_t seed = 1;
  double dt = 1.0;
  double speed = 0.4;
  double frames_per_sec_link = 20.0;
  double frame_loss = 0.002;
  size_t backlog_cap = 4000;
  double beacon_drop = 0.05;
  double horizon = 3.0 * 3600.0;
  double finish_min = 600.0, finish_max = 1800.0;
  int trunks = 30;
  // Tests may override the TeamConfig after the defaults are filled.
  std::function<void(TeamConfig&)> tweak;
  // Per-(now, i, j) link override: return 0 to force down, 1 to force up, -1
  // to use the world.
  std::function<int(double, int, int)> link_override;
  // Sees every beacon that the link would deliver, (now, sender, receiver,
  // beacon); may edit it, and returns false to drop it.
  std::function<bool(double, int, int, Beacon&)> beacon_filter;
};

class Sim {
 public:
  explicit Sim(const Params& p) : p_(p), rng_(p.seed) {
    std::uniform_real_distribution<double> u(0, 1);
    for (int t = 0; t < p_.trunks; ++t)
      world_.trunks.push_back({{5 + 90 * u(rng_), 5 + 90 * u(rng_)}, 0.3 + 0.3 * u(rng_)});
    robots_.resize(p_.n);
    links_.resize(p_.n * p_.n);
    const Vec2 base{50, 5};
    for (int i = 0; i < p_.n; ++i) {
      Robot& r = robots_[i];
      r.oracle = std::make_unique<ToyOracle>(&world_, &rng_, &now_);
      TeamConfig c;
      c.arm = p_.arm;
      c.self_id = i;
      c.n = p_.n;
      c.team_hash = 0xC0FFEEu;
      c.grid_hash = 0xBEEFu;
      if (p_.tweak) p_.tweak(c);
      r.core = std::make_unique<TeamCore>(c, r.oracle.get());
      r.home = {base.x + 3.0 * i, base.y};
      r.pos = r.home;
      r.finish_at = p_.finish_min + (p_.finish_max - p_.finish_min) * u(rng_);
      const double w = world_.size / p_.n;
      r.sector_lo = {w * i, 10};
      r.sector_hi = {w * (i + 1), world_.size};
      r.rcvd.assign(p_.n, 0);
      r.gaps.assign(p_.n, 0);
    }
  }

  Robot& robot(int i) { return robots_[i]; }
  World& world() { return world_; }
  double now() const { return now_; }
  std::mt19937& rng() { return rng_; }

  bool linkUp(int i, int j) {
    if (p_.link_override) {
      const int o = p_.link_override(now_, i, j);
      if (o == 0) return false;
      if (o == 1) return true;
    }
    return world_.linkUp(robots_[i].pos, robots_[j].pos);
  }

  bool allDone() const {
    for (const Robot& r : robots_)
      if (r.out.activity != Activity::kDone) return false;
    return true;
  }

  // One second of the world.
  void step() {
    std::uniform_real_distribution<double> u(0, 1);
    const int n = p_.n;
    // Sensing: frames.
    for (int i = 0; i < n; ++i) {
      Robot& r = robots_[i];
      if (now_ >= r.next_frame) {
        ++r.sent;
        r.rcvd[i] = r.sent;
        for (int j = 0; j < n; ++j) {
          if (j == i) continue;
          Link& L = links_[i * n + j];
          L.q.push_back(r.sent);
          if (L.q.size() > p_.backlog_cap) L.q.pop_front();
        }
        r.next_frame = now_ + (r.out.activity == Activity::kExplore ? 1.0 : 5.0);
      }
      if (!r.finished && now_ >= r.finish_at) r.finished = true;
    }
    // Reliable drain.
    for (int i = 0; i < n; ++i)
      for (int j = 0; j < n; ++j) {
        if (i == j) continue;
        Link& L = links_[i * n + j];
        if (L.q.empty() || !linkUp(i, j)) continue;
        int budget = static_cast<int>(p_.frames_per_sec_link * p_.dt);
        Robot& rj = robots_[j];
        while (budget-- > 0 && !L.q.empty()) {
          const uint64_t s = L.q.front();
          L.q.pop_front();
          if (u(rng_) < p_.frame_loss) continue;
          if (s > rj.rcvd[i] + 1) rj.gaps[i] += s - rj.rcvd[i] - 1;
          rj.rcvd[i] = std::max(rj.rcvd[i], s);
        }
      }
    // Decide.
    for (int i = 0; i < n; ++i) {
      Robot& r = robots_[i];
      TickInputs in;
      in.now = now_;
      in.pose = r.pos;
      in.have_pose = true;
      in.finished = r.finished;
      in.sent_seq = r.sent;
      in.rcvd_seq = r.rcvd;
      in.seq_gaps = r.gaps;
      in.home = r.home;
      in.have_home = true;
      r.out = r.core->tick(in);
      if (r.trace.empty() || r.trace.back().second != r.out.activity)
        r.trace.push_back({now_, r.out.activity});
      auto ev = r.core->drainEvents();
      if (on_events) on_events(i, ev);
      if (r.keep_events)
        for (auto& e : ev) r.events.push_back(std::move(e));
    }
    // Beacons.
    for (int i = 0; i < n; ++i) {
      const Beacon b0 = robots_[i].core->beacon();
      for (int j = 0; j < n; ++j) {
        if (j == i || !linkUp(i, j)) continue;
        if (u(rng_) < p_.beacon_drop) continue;
        Beacon b = b0;
        if (p_.beacon_filter && !p_.beacon_filter(now_, i, j, b)) continue;
        robots_[j].core->onBeacon(b, now_);
      }
    }
    // Move.
    for (int i = 0; i < n; ++i) move(robots_[i], u);
    now_ += p_.dt;
  }

  std::function<void(int, const std::vector<TeamEvent>&)> on_events;

 private:
  void move(Robot& r, std::uniform_real_distribution<double>& u) {
    if (now_ >= r.stuck_from) return;
    Vec2 goal = r.pos;
    const auto& d = r.out.drive;
    if (d.kind == DriveKind::kLeg) {
      goal = d.point;
    } else if (d.kind == DriveKind::kExplore) {
      if (!r.have_wp || dist(r.pos, r.wp) < 1.0) {
        r.wp = {r.sector_lo.x + (r.sector_hi.x - r.sector_lo.x) * u(rng_),
                r.sector_lo.y + (r.sector_hi.y - r.sector_lo.y) * u(rng_)};
        r.have_wp = true;
      }
      goal = r.wp;
    }
    const double L = dist(r.pos, goal);
    const double s = p_.speed * p_.dt;
    if (L <= s) r.pos = goal;
    else { r.pos.x += (goal.x - r.pos.x) / L * s; r.pos.y += (goal.y - r.pos.y) / L * s; }
  }

  Params p_;
  std::mt19937 rng_;
  World world_;
  std::vector<Robot> robots_;
  std::vector<Link> links_;
  double now_ = 0.0;
};

}  // namespace toy
