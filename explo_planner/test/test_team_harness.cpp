// Gen-34 in-process harness (DESIGN_gen34.md §8.10 step 4): N TeamCores in the
// toy world of team_toy_sim.hpp, run as whole missions.
//
// Properties, over seeded missions per arm per team size (TEAM_HARNESS_SEEDS,
// default 1000):
//   P1 termination: every robot reaches Done before the horizon.
//   P2 met means exchanged: a booking's pair_met with j comes only while the
//      exchange with j is done on a live contact.
//   P3 bounded waits: while an activity has a wait bound, the clock never
//      passes it by more than one tick.
//   P4 no flip-flop: at most 6 activity changes in any 60 s. A single short
//      stint is allowed (a link at the edge of range comes and goes); an
//      activity that alternates tick by tick is not. Short stints (A -> B -> A
//      with B under 3 s) are counted in the summary line.
//   P5 no slot burn: a full meeting is never more than one interval ahead of
//      its slot.
//   P6 gossip bound (§11.1): after every tick, the highest version in a
//      robot's plan table is its own plan's.
// Replays:
//   cell 12: a finished robot at the cell, its peer stuck out of reach, goes
//      home after two missed slots.
//   plan split: one robot misses the v2 renewal; odd slots go to the
//      laggard's cell (the plan table), which brings the team back to one
//      cell, and the robot adopts v2.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "team_toy_sim.hpp"

using explo_planner::findField;
using explo_planner::LogField;
using namespace toy;
using explo_planner::gen34::Booking;
using explo_planner::gen34::Plan;

namespace {

int seedsFromEnv() {
  const char* s = std::getenv("TEAM_HARNESS_SEEDS");
  if (!s) return 1000;
  const int v = std::atoi(s);
  return v > 0 ? v : 1000;
}

const std::string& action(const TeamEvent& e) {
  static const std::string none;
  const LogField* f = findField(e.fields, "action");
  return f ? f->s : none;
}

struct Verdict {
  bool done = false;
  std::vector<std::string> faults;
  double end = 0.0;
  std::map<std::string, long long> counts;   // summed over robots
  long long short_stints = 0;
};

Verdict runMission(const Params& p) {
  Sim sim(p);
  Verdict v;
  auto fault = [&](const std::string& s) {
    if (v.faults.size() < 8) v.faults.push_back(s);
  };
  sim.on_events = [&](int i, const std::vector<TeamEvent>& ev) {
    for (const TeamEvent& e : ev) {
      // P5, slot burn: a full meeting more than one interval ahead of its
      // slot spends a future slot and pushes the next real meeting out.
      if (e.kind == "booking" && action(e) == "full_met") {
        const Plan& pl = sim.robot(i).core->plan();
        const int k = static_cast<int>(findField(e.fields, "slot")->i);
        if (pl.slotTime(k) - sim.now() > pl.interval + 1e-9) {
          ++v.counts["met_ahead"];
          std::ostringstream o;
          o << "P5 t=" << sim.now() << " robot " << i << " full_met slot " << k << " "
            << pl.slotTime(k) - sim.now() << " s ahead";
          fault(o.str());
        }
      }
      if (e.kind != "booking" || action(e) != "pair_met") continue;
      const int j = static_cast<int>(findField(e.fields, "peer")->i);
      const TeamCore& c = *sim.robot(i).core;
      if (!c.exchanged(j) || !c.inContact(j)) {
        std::ostringstream o;
        o << "P2 t=" << sim.now() << " robot " << i << " pair_met " << j
          << " without a done exchange on contact";
        fault(o.str());
      }
    }
  };
  while (sim.now() < p.horizon) {
    sim.step();
    const double now = sim.now() - p.dt;   // the tick just run
    for (int i = 0; i < p.n; ++i) {
      const TeamCore& c = *sim.robot(i).core;
      uint32_t top = 0;
      for (int j = 0; j < p.n; ++j) top = std::max(top, c.knownPlan(j).version);
      if (top != c.plan().version) {
        std::ostringstream s;
        s << "P6 t=" << now << " robot " << i << " table holds v" << top
          << " but the plan is v" << c.plan().version;
        fault(s.str());
      }
      const TickOutputs& o = sim.robot(i).out;
      if (o.wait_bound >= 0.0 && now > o.wait_bound + p.dt + 1e-9) {
        std::ostringstream s;
        s << "P3 t=" << now << " robot " << i << " " << activityName(o.activity)
          << " past its bound " << o.wait_bound;
        fault(s.str());
      }
    }
    if (sim.allDone()) { v.done = true; break; }
  }
  v.end = sim.now();
  if (!v.done) {
    std::ostringstream s;
    s << "P1 not all done by " << p.horizon << ":";
    for (int i = 0; i < p.n; ++i)
      s << " r" << i << "=" << activityName(sim.robot(i).out.activity);
    fault(s.str());
  }
  for (int i = 0; i < p.n; ++i) {
    const auto& tr = sim.robot(i).trace;
    for (size_t k = 1; k + 1 < tr.size(); ++k)
      if (tr[k + 1].second == tr[k - 1].second && tr[k + 1].first - tr[k].first < 3.0)
        ++v.short_stints;
    // tr[0] is the first activity, not a change.
    for (size_t k = 1, lo = 1; k < tr.size(); ++k) {
      while (tr[k].first - tr[lo].first >= 60.0) ++lo;
      if (k - lo + 1 > 6) {
        std::ostringstream s;
        s << "P4 robot " << i << " " << (k - lo + 1) << " activity changes in 60 s ending t="
          << tr[k].first << " (" << activityName(tr[k - 1].second) << "->"
          << activityName(tr[k].second) << ")";
        fault(s.str());
        break;
      }
    }
    for (const auto& kv : sim.robot(i).core->counts()) {
      v.counts[kv.first] += kv.second;
      const auto& names = TeamCore::counterNames();
      if (std::find(names.begin(), names.end(), kv.first) == names.end())
        fault("counter '" + kv.first + "' is missing from TeamCore::counterNames()");
    }
  }
  return v;
}

void sweep(Arm arm, int n) {
  const int seeds = seedsFromEnv();
  int failed = 0;
  std::map<std::string, long long> totals;
  double worst_end = 0.0;
  long long short_stints = 0;
  for (int s = 1; s <= seeds; ++s) {
    Params p;
    p.n = n;
    p.arm = arm;
    p.seed = static_cast<uint32_t>(s * 7919 + n * 131 + static_cast<int>(arm));
    const Verdict v = runMission(p);
    worst_end = std::max(worst_end, v.end);
    short_stints += v.short_stints;
    for (const auto& kv : v.counts) totals[kv.first] += kv.second;
    if (!v.faults.empty()) {
      if (++failed <= 5) {
        std::ostringstream o;
        o << armName(arm) << " n=" << n << " seed=" << p.seed << ":";
        for (const auto& f : v.faults) o << "\n    " << f;
        ADD_FAILURE() << o.str();
      }
    }
  }
  EXPECT_EQ(failed, 0) << armName(arm) << " n=" << n << ": " << failed << "/" << seeds;
  // The mechanisms the arm owns must have fired somewhere in the sweep.
  if (armBooks(arm)) {
    EXPECT_GT(totals["booking_full_met"], 0) << armName(arm) << " n=" << n;
    EXPECT_GT(totals["plan_renewed"], 0) << armName(arm) << " n=" << n;
  } else {
    EXPECT_EQ(totals["booking_booked"], 0) << armName(arm) << " n=" << n;
  }
  if (armChases(arm)) {
    EXPECT_GT(totals["chase_start"], 0) << armName(arm) << " n=" << n;
  } else {
    EXPECT_EQ(totals["chase_start"], 0) << armName(arm) << " n=" << n;
  }
  std::ostringstream o;
  o << armName(arm) << " n=" << n << " seeds=" << seeds << " worst_end=" << worst_end
    << " short_stints=" << short_stints;
  for (const char* k : {"booking_full_met", "booking_missed", "booking_partial",
                        "booking_patience", "booking_backstop", "booking_encounter",
                        "booking_renewal_unseen", "booking_split_alternate",
                        "booking_renewed", "booking_depart_deferred",
                        "met_ahead", "chase_start", "chase_done", "chase_failed", "chase_limit",
                        "chase_pre-empted", "reconnect_moves", "reconnect_fallback",
                        "booking_retargeted", "booking_retarget_late", "exchange_done",
                        "exchange_gave_up", "exchange_done_late", "homing_gave_up",
                        "team_finished"})
    o << " " << k << "=" << totals[k];
  std::cout << "[ harness  ] " << o.str() << std::endl;
}

}  // namespace

TEST(TeamHarness, Off)        { for (int n = 2; n <= 4; ++n) sweep(Arm::kOff, n); }
TEST(TeamHarness, Pursuit)    { for (int n = 2; n <= 4; ++n) sweep(Arm::kPursuit, n); }
TEST(TeamHarness, Rendezvous) { for (int n = 2; n <= 4; ++n) sweep(Arm::kRendezvous, n); }
TEST(TeamHarness, Hybrid)     { for (int n = 2; n <= 4; ++n) sweep(Arm::kHybrid, n); }

// ── Replays ─────────────────────────────────────────────────────────────────

// Cell 12 of the gen-33 campaign: robot 0 finished and at the meeting cell,
// robot 1 out of reach for good. Robot 0 attends two slots, then goes home.
TEST(TeamHarnessReplay, Cell12FinishedRobotGoesHomeAfterTwoMissedSlots) {
  Params p;
  p.n = 2;
  p.arm = Arm::kRendezvous;
  p.seed = 12;
  p.trunks = 0;
  p.finish_min = p.finish_max = 400.0;
  // Together for the first 130 s (the plan firms at 120 s), then never again.
  p.link_override = [](double now, int, int) { return now < 130.0 ? 1 : 0; };
  Sim sim(p);
  sim.robot(1).stuck_from = 130.0;
  for (int i = 0; i < 2; ++i) sim.robot(i).keep_events = true;
  while (sim.now() < 3 * 3600.0 && !sim.allDone()) sim.step();
  ASSERT_TRUE(sim.allDone());
  const auto& ev = sim.robot(0).events;
  int missed = 0, met = 0;
  double home_at = -1.0;
  std::string home_reason;
  for (const TeamEvent& e : ev) {
    if (e.kind == "booking" && action(e) == "missed") ++missed;
    if (e.kind == "booking" && action(e) == "met") ++met;
    if (e.kind == "homing" && action(e) == "start" && home_at < 0) {
      home_reason = findField(e.fields, "reason")->s;
      home_at = 1.0;
    }
  }
  EXPECT_EQ(met, 0);
  EXPECT_EQ(home_reason, "missed-streak");
  EXPECT_GE(sim.robot(0).core->missedStreak(), 2);
  // Robot 0 attended from its finish (400 s): at most two slots after it and
  // one before it can have been missed.
  EXPECT_LE(missed, 3);
  EXPECT_EQ(sim.robot(0).out.activity, Activity::kDone);
  EXPECT_TRUE(sim.robot(0).core->homeDone());
}

// A three-robot team meets; robot 2 does not hear the v2 renewal before the
// team disperses. Robots 0 and 1 know robot 2 is still on v1 and send odd
// slots to v1's cell, where robot 2 still goes; that meeting heals the split.
//
// Staging: a 12 m radio, so robots hear each other only at meetings (in the
// full world a chance encounter heals a split directly, before any slot). Every
// beacon to robot 2 shows v1 from the renewal until robot 2 and a peer have
// both left for the same later slot at the same cell: its plan fields, and
// every entry of its table, or the table would outrank robot 2's plan (P6).
TEST(TeamHarnessReplay, PlanSplitHealsByAlternation) {
  Params p;
  p.n = 3;
  p.arm = Arm::kRendezvous;
  p.seed = 34;
  p.trunks = 0;
  p.finish_min = p.finish_max = 9000.0;   // nobody finishes during the replay
  p.horizon = 8000.0;
  // Two fixed cells 40 m apart, so v2 is not v1's cell.
  const Vec2 a{30, 60}, b{70, 60};
  int solve_count = 0;
  bool renewed = false, lifted = false;
  Plan v1;
  int renew_slot = -1, lift_slot = -1, lift_cell = -1;
  p.beacon_filter = [&](double, int, int to, Beacon& bc) {
    if (to != 2 || !renewed || lifted) return true;
    if (bc.plan.version > v1.version) bc.plan = v1;
    for (size_t k = 0; k < bc.plan_known_version.size(); ++k) {
      if (bc.plan_known_version[k] <= v1.version) continue;
      bc.plan_known_version[k] = v1.version;
      bc.plan_known_cell[k] = v1.cell;
      bc.plan_known_center[k] = v1.center;
    }
    return true;
  };
  Sim sim(p);
  sim.world().range = 12.0;
  sim.robot(0).oracle->solve_hook = [&](const std::vector<TeamRobotView>&) {
    PlanSolve s;
    s.ok = true;
    s.provisional = false;
    const Vec2 c = (solve_count++ == 0) ? a : b;
    s.cell = sim.world().cellAt(c);
    s.center = sim.world().cellCenter(s.cell);
    return s;
  };
  sim.on_events = [&](int i, const std::vector<TeamEvent>& ev) {
    if (i != 0 || renewed) return;
    for (const TeamEvent& e : ev)
      if (e.kind == "plan" && action(e) == "renewed") {
        renewed = true;
        v1 = sim.robot(0).core->previousPlan();
        renew_slot = sim.robot(0).core->plan().renewed_at_slot;
      }
  };
  bool healed = false;
  int p6_faults = 0;
  while (sim.now() < p.horizon) {
    sim.step();
    // P6 here too: robot 2's table must not outrank its plan, which it would
    // if the filter rewrote the plan fields and not the table.
    for (int i = 0; i < p.n; ++i) {
      const TeamCore& c = *sim.robot(i).core;
      uint32_t top = 0;
      for (int j = 0; j < p.n; ++j) top = std::max(top, c.knownPlan(j).version);
      if (top != c.plan().version && p6_faults++ == 0)
        ADD_FAILURE() << "P6 t=" << sim.now() - p.dt << " robot " << i
                      << " table holds v" << top << " but the plan is v"
                      << c.plan().version;
    }
    if (renewed && !lifted) {
      const Booking& b2 = sim.robot(2).core->booking();
      for (int i = 0; i < 2 && !lifted; ++i) {
        const Booking& bi = sim.robot(i).core->booking();
        if (b2.held && b2.departed && b2.slot > renew_slot && bi.held && bi.departed &&
            bi.slot == b2.slot && bi.cell == b2.cell) {
          lifted = true;
          lift_slot = b2.slot;
          lift_cell = b2.cell;
        }
      }
    }
    if (lifted && sim.robot(2).core->plan().version >= 2) { healed = true; break; }
  }
  ASSERT_TRUE(renewed) << "no renewal in the replay window";
  ASSERT_TRUE(lifted) << "robot 2 and a peer never left for the same meeting";
  EXPECT_EQ(lift_cell, v1.cell) << "the shared meeting was not at v1's cell";
  EXPECT_EQ(lift_slot % 2, 1) << "the shared meeting was not an odd slot";
  EXPECT_TRUE(healed) << "robot 2 never adopted v2";
  EXPECT_EQ(sim.robot(2).core->plan().cell, sim.robot(0).core->plan().cell);
  long long alternate = 0;
  for (int i = 0; i < 2; ++i) {
    auto it = sim.robot(i).core->counts().find("booking_split_alternate");
    if (it != sim.robot(i).core->counts().end()) alternate += it->second;
  }
  EXPECT_GT(alternate, 0) << "the alternation never sent a slot to v1's cell";
}
