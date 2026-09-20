#pragma once
/// @file rendezvous_scheduler.hpp
/// @brief Where and when a separated pair should meet, derived from the tours
///        they were already driving. See docs/mtare_evolution_plan.md §3.5.
///
/// THE IDEA THIS FILE EXISTS TO EXPRESS: a meeting is not a detour to a
/// landmark, it is a CONSTRAINT ON THE TOURS the robots were already going to
/// drive. Two robots sweeping a shared ROI pass near each other repeatedly; the
/// rendezvous should be the cheapest of those space-time near-misses, not a
/// third place both must pay to reach.
///
/// That is a reaction to two measured failures, not a preference:
///
///   * The v2/v3 design optimised a geometric minimax 1-centre over frontier
///     cells — the distance BETWEEN FRONTIERS, a quantity no robot pays. It is
///     blind to where the robots are, where they are going, and what the trip
///     costs. mTARE's own version is worse: it offsets past the robot rows of
///     its distance matrix, so robot positions are excluded from the objective
///     by construction and nothing bounds the meeting point's distance from the
///     team (§3.5.1).
///   * The then-shipped fallback — the midpoint of the last-contact pose pair —
///     sits BEHIND both robots on ground they have already covered. In campaign
///     mh1 the value gate declined 100 of 110 evaluations, every one on cost and
///     none for lack of knowledge, which is precisely the signature of a
///     destination worth nothing: reconnecting there can only cost more than
///     not reconnecting. (The midpoint was removed outright on 2026-09-16 — see
///     the removal notes in dispatchReconnect. The FLOOR MACHINERY HERE IS NOT
///     GONE WITH IT and is not test-only: since 2026-09-18 the node fills
///     `floor_cell` with the centroid of the team's own vehicle cells on every
///     solve. Read `floor_cell` on `solve` below for why that is a different
///     quantity from the midpoint and what it is there to prevent.)
///
/// So the objective here is TOUR-RELATIVE. Score a candidate cell by the
/// makespan penalty of forcing every robot's tour through it, and take the
/// argmin. The robot whose tour already contains the cell pays nothing; the
/// other pays one insertion. Both terms are the allocator's own `routeCostMm`,
/// so there is no second cost model to keep consistent with the first.
///
/// THIS FILE IS NOT THE AGREEMENT PROTOCOL. It used to be — the claim here was
/// that no protocol was needed at all, because global_allocator.hpp guarantees
/// tours are bit-identical ACROSS PROCESSES (integer-mm quantisation, a total
/// tie-break order, no clock, no random, no address-derived value), so a
/// rendezvous derived purely from that output would be bit-identical too, and
/// agreement would be a CONSEQUENCE of identical arithmetic over a shared
/// world.
///
/// THE ARITHMETIC HELD. THE SHARED WORLD DID NOT. Every determinism property
/// below is still true and still enforced; what was false was the premise that
/// the two robots feed this function the same inputs. They do not: each solves
/// over its own privately merged map, and a merge is a function of what the
/// radio happened to deliver. Measured on the banked ts3 n2+n3 cells, where
/// each robot solved privately at its own arming instant: 21 of 64 separated
/// pairs picked the same cell, and the median t_meet disagreement was 91 s at
/// N=2 and 125 s at N=3. Nine pairs disagreed with IDENTICAL `shared_hash` —
/// the candidate COUNT differed in 5 of 19 — so the hash was never a complete
/// witness of a shared world either.
///
/// So the agreement protocol now lives ONE LEVEL UP, in the node: the robot
/// with the lowest fleet id calls this function, publishes the resulting
/// (cell, interval, t_meet) triple on TeamWorld, and everyone else adopts those
/// three integers verbatim and echoes them back until all N hold the same
/// triple. See TeamWorld.msg's rendezvous block and the node's P5 state block.
/// What that makes THIS file is the proposer's search, not a replicated
/// computation.
///
/// THE INSTANT IS ON THE WIRE, NOT RE-DERIVED (2026-09-17). Until then only
/// (cell, interval) were published and each robot supplied its own origin —
/// the moment IT saw the team stop being mutually whole. At N=2 that is one
/// physical event and the two origins landed 0.84 s apart. At N>=3 it is not
/// one event at all, because the graph comes apart edge by edge and each robot
/// sees a different edge go first: the ts4 N=3 cell put three robots' origins
/// at 13.18 / 34.18 / 33.98 s, so three robots holding a byte-identical
/// interval kept it 21 s apart. Publishing the instant removes the per-robot
/// term outright and leaves only the robots' mission-clock baselines, measured
/// ~1.2 s apart across the same cells.
///
/// The determinism discipline is kept anyway, because it is what makes a
/// proposal auditable offline and because a solve that varied with vector
/// order would be untestable:
///
///   * every quantity that reaches a comparison is integral (mm, ms);
///   * every tie breaks on cell id, ascending;
///   * the vehicle set is walked in fleet-id order, so a caller's vector order
///     cannot reach the result;
///   * nothing here reads a clock. `mission_elapsed_ms` is a parameter, and it
///     is MISSION-ELAPSED rather than an absolute stamp because field clocks
///     drift hours apart (the 2026-07-06 bunker/curt bags were 4531 s apart
///     while recording simultaneously). The node passes its own mission-elapsed
///     ms at proposal time, so `t_meet_ms` comes out in the one frame every
///     robot in the fleet can evaluate without a clock exchange.
///
/// WHAT THIS DELIBERATELY DOES NOT DO: get called after contact is lost. Once
/// the worlds diverge the tours diverge, so a re-derived rendezvous would
/// differ between robots — which is the one thing the design cannot survive.
/// The node solves only while the team reads COMPLETE and freezes the agreed
/// pair the instant it does not; see §3.5 and the node's `reconnect_rec_` for
/// the same discipline applied to the pose pair.

#include <cstdint>
#include <string>
#include <vector>

#include "explo_planner/cell_world.hpp"
#include "explo_planner/global_allocator.hpp"

namespace explo_planner {

/// An appointment: a cell to meet in and a mission-elapsed time to be there.
///
/// Everything that produced it is carried alongside, for the same reason the
/// reconnection gate carries its arithmetic: an appointment nobody can
/// re-derive offline is one nobody can audit, and the whole design rests on
/// two robots computing the same numbers from different processes.
struct RendezvousPlan {
  /// The agreed cell, or -1 when none could be derived (see `refused`).
  int cell = -1;

  /// Mission-elapsed milliseconds at which both robots should be at `cell`.
  /// -1 when there is no plan.
  long long t_meet_ms = -1;

  /// Makespan penalty of the winning cell, in the allocator's quantised unit.
  /// Zero is normal and good: it means the winner was already on the critical
  /// robot's tour and the meeting costs the team nothing.
  long long penalty_mm = -1;

  /// The interval the solve landed on. ON A PLAN `RendezvousScheduler::solve`
  /// RETURNED it is `t_meet_ms - mission_elapsed_ms` by construction — there is
  /// one assignment site and it writes the two together. This is the integer
  /// that goes on the wire and the one every consumer means by "the interval".
  ///
  /// THAT IDENTITY DOES NOT HOLD ON THE NODE'S `appointment_`, which is this
  /// struct reused as a carrier rather than a solve result (see `capped` below
  /// for the same caveat on the derived fields). Arming copies `interval_ms`
  /// off the wire and overwrites `t_meet_ms` with the first agreed occurrence
  /// not already past at arming (nextAgreedOccurrence — a local countdown
  /// until generation 23, now-plus-notice until generation 25) — so the
  /// difference there depends on when this robot armed and has nothing to do
  /// with this field. Anything that recovers one of the pair from the other
  /// is reading a solve-time invariant on a struct that is no longer a
  /// solve's output.
  ///
  /// IT IS THE RECURRENCE PERIOD: `t_meet_ms` is the first occurrence of a
  /// meeting repeating every `interval_ms`, and a separating robot keeps the
  /// next occurrence at or after its own departure floor. That was true, was
  /// briefly not (generation 19 replaced the timetable with a local countdown),
  /// and is true again since generation 23 — see nextAgreedOccurrence in
  /// planner_util.hpp, which is the only reader.
  ///
  /// The integer is derived, goes on the wire, has to match byte for byte
  /// across the fleet, and is logged — it is the agreement evidence the
  /// campaign gate scores, and the reachability floor that sets it is the
  /// honest statement of "how long the furthest robot needs". That floor is
  /// what keeps the arrival stagger inside one occurrence, so a change to it
  /// is a change to the meeting, not only to the evidence.
  long long interval_ms = -1;

  /// The raw tour term — max over robots of "time to reach `cell` along its
  /// own tour" — before the lead floor raised it or the divergence cap pulled
  /// it in. Kept separate because it is the only one of the three that answers
  /// "was this meeting nearly free?", and because the slowest robot's lateness
  /// under a cap is `tour_interval_ms - interval_ms`.
  long long tour_interval_ms = -1;

  /// The floor raised the interval above what the objective AND the cap between
  /// them asked for: the tours put the occurrences closer together than the
  /// direct drive or `min_interval_ms` allow, so the period is set by
  /// reachability rather than by the objective.
  ///
  /// AGAINST `min(tour_interval_ms, max_interval_ms)`, NOT AGAINST THE TOUR
  /// TERM ALONE (2026-09-18). Measuring it against the tour term alone reads
  /// false whenever the cap had already pulled the ask below the floor, which is
  /// the one configuration where the floor is doing something the caller needs
  /// to know about.
  ///
  /// EXPECT THIS TO BE TRUE ON MOST ROWS, and do not read it as a defect. The
  /// objective deliberately finds the cheapest space-time near-miss, so the
  /// tour term is small by design — it was 28.284 s at N=2 in the ts4 smoke,
  /// one diagonal grid step. A campaign where this is always FALSE is the
  /// surprising one.
  bool floored = false;

  /// True when the cap pulled the period in below what the tours imply. The far
  /// robot is then structurally late for each occurrence by `tour_interval_ms -
  /// interval_ms`; the barrier's wait absorbs it, which is the whole reason the
  /// cap is set from that wait. Reported rather than hidden because a run where
  /// this is always true is one where the cap, not the objective, is choosing
  /// the meeting time.
  ///
  /// A LIVE COLUMN AGAIN (2026-09-17). It was false on every row the node
  /// produced while `max_interval_ms` carried the old divergence cap, which was
  /// fed from a last-contact quantity that does not exist at proposal time.
  /// The field now carries the findability bound instead and the node passes
  /// `reconnect_midrun_max_wait_sec`, so the cap really can bind.
  ///
  /// IT CANNOT PUSH THE PERIOD BELOW THE REACHABILITY FLOOR — a period nobody
  /// can drive in is not made findable by being short — so when the cap asks
  /// for less than the floor, the period stops at the floor. This still reads
  /// true in that case, because the cap did pull the period in from the tours;
  /// it just did not get all it asked for.
  ///
  /// THE ONE SHAPE WHERE THE FINDABILITY INEQUALITY IS BROKEN is BOTH FLAGS
  /// TRUE, which is the same statement as `interval_ms > max_interval_ms`: the
  /// cap cut, the floor refused the cut, and the team is further apart than the
  /// barrier is willing to wait — so a robot that misses an occurrence is not
  /// guaranteed to find anyone at the next one. The scheduler reports it rather
  /// than pretending otherwise, and the node warns on it (the warn keys on the
  /// inequality directly, so it holds whatever these two flags say).
  ///
  /// The two are NOT mutually exclusive and never were. A revision on
  /// 2026-09-18 briefly made them so by deriving both from the returned value,
  /// which collapsed this shape to "neither bound bound"; see the derivation in
  /// rendezvous_scheduler.cpp.
  bool capped = false;

  /// The committed cell IS the caller's `floor_cell`. NOT the midpoint: that
  /// floor is gone, and the one the node passes today is the centroid of the
  /// team's own vehicle cells (see `floor_cell` on `solve`). Historical
  /// `floor_won` rates quoted in planner_util.hpp and in the node's removal
  /// notes were measured against the MIDPOINT and do not carry over.
  ///
  /// READ IT AS AN IDENTITY, NOT AS A VERDICT (2026-09-18). This used to say
  /// "no tour cell was worth its detour, so the team agreed to meet at the
  /// floor instead of anywhere a tour goes", and that reading is not sound. The
  /// candidate scan SKIPS the floor cell and re-adds it unconditionally, so a
  /// cell that is both on a tour and the floor appears exactly once, as the
  /// floor — and if it then wins the argmin at penalty 0 it wins as the cell it
  /// always was, with the floor mechanism contributing nothing. The same solve
  /// reports floor_won=0 with floor_cell=-1 and floor_won=1 with floor_cell set
  /// to the cell it was going to pick anyway, which is enough to inflate any
  /// rate computed from this column. `penalty_mm` is the quantity that says
  /// whether the choice cost anything; use the two together.
  bool floor_won = false;

  /// Admissible candidates considered, and the two reasons a candidate was
  /// dropped. A plan with `candidates == 1` is one where only the floor
  /// survived, which is a different situation from one where the floor won on
  /// merit, and the P5 non-vacuity rule needs to tell them apart.
  int candidates = 0;
  int rejected_unreachable = 0;
  int rejected_excluded = 0;

  /// Non-empty when no appointment could be derived at all. The caller must
  /// fall back to its pre-P5 behaviour rather than treating this as "meet at
  /// cell -1"; an empty plan is a NORMAL outcome early in a run, before the
  /// allocator has any tours to speak of.
  std::string refused;

  bool valid() const { return cell >= 0 && refused.empty(); }
};

class RendezvousScheduler {
public:
  struct Config {
    /// Conservative drive speed, in mm/s. Integer so that travel times
    /// quantise exactly and two robots converting the same distance land on
    /// the same millisecond. A float speed would reintroduce, at the last
    /// division, precisely the cross-process disagreement the allocator spent
    /// its whole design removing.
    ///
    /// Non-positive disables the scheduler outright (every solve refuses),
    /// because a zero speed makes every arrival time infinite and there is no
    /// safe value to substitute.
    long long speed_mm_s = 500;

    /// Safety multiplier on travel time, in thousandths. 1200 = allow 1.2x the
    /// estimated drive. It exists because the estimate is a straight-line-ish
    /// graph distance at a nominal speed and the real drive is neither.
    ///
    /// TWO READERS, AND THEY HAVE TO BE THE SAME CONSTANT: the reachability
    /// floor in solve(), which sizes the lattice spacing to cover the furthest
    /// robot's marked-up drive, and ExploPlannerNode::appointmentLeadMs, which
    /// is how much lead a robot actually takes. Pricing the lead higher than
    /// the floor would mean a robot needing more notice than the spacing the
    /// team sized for it — the one shortfall a rung roll cannot fix, because
    /// the next rung is only one spacing away.
    ///
    /// The name is the generation-19 departure test's and that test is gone
    /// (see the class comment); it is kept because the quantity is the same one
    /// and renaming it would break every banked params row for no gain.
    int depart_safety_milli = 1200;

    /// Lower bound on the interval, in ms. <= 0 means no floor.
    ///
    /// IT STOPS THE INTERVAL DEGENERATING. The objective below minimises the
    /// detour, so its winner is the nearest space-time near-miss and its raw
    /// interval can be a single grid step — 28.284 s at N=2 in the ts4 smoke,
    /// and nothing prevents a candidate the whole team is already standing on
    /// from producing nearly zero.
    ///
    /// WHY THAT STILL MATTERS WITH THE SCHEDULE GONE (generation 19). It used
    /// to matter because the interval was the recurrence period and a period of
    /// a few seconds is not a schedule. Nothing recurs now, so the argument is
    /// narrower and worth stating rather than inheriting: `interval_ms` is
    /// exchanged, compared for exact equality and scored, and it is read as
    /// "how long the furthest robot needs". A near-zero value is a false
    /// statement of that, and the floor is what keeps the logged integer
    /// meaning what every consumer takes it to mean.
    ///
    /// The node feeds this from `rendezvous_interval_sec` (300 s), which exists
    /// for exactly this and nothing else. It used to come from
    /// `rendezvous_proposal_period_sec` "for no deeper reason than that it is
    /// the coarsest cadence the protocol already has", and that accident cost
    /// something real: the proposal period also bounds how stale the snapshot
    /// the punctuality estimate is costed against may be, so the two could not
    /// be raised together. They are now two numbers because they bound two
    /// things.
    long long min_interval_ms = 0;

    /// Upper bound on the interval, in ms. <= 0 means uncapped.
    ///
    /// REPURPOSED (2026-09-17), and the new reason is much stronger than the
    /// old one. It used to carry a map-divergence model — meet sooner because
    /// the maps are separating fast — which the node never fed (it passed 0)
    /// because the model's inputs do not exist at proposal time and do not
    /// survive N > 2.
    ///
    /// It is now the FINDABILITY BOUND. The arithmetic has not changed since,
    /// but WHAT IT BOUNDS HAS (generation 19), and the two readings are easy to
    /// confuse because the inequality looks identical.
    ///
    ///     v6, the schedule:  period <= wait cap. A robot arming too close to
    ///     occurrence k arrives late by less than one period; its peers, having
    ///     arrived at k, are still standing there, so a missed occurrence is
    ///     harmless.
    ///
    ///     v8, the schedule again: furthest drive <= wait cap. After the
    ///     reachability floor in solve() the interval IS the furthest robot's
    ///     direct drive, so the same comparison now asks whether the last robot
    ///     to set off can still arrive before the ones already waiting give up
    ///     — which, the floor being what it is, is also the v6 reading.
    ///
    /// THE NODE PASSES `rendezvous_appointment_wait_sec` (2026-09-19), the wait
    /// a robot standing at the AGREED CELL actually spends. It passed
    /// `reconnect_midrun_max_wait_sec` until generation 29 on the argument that
    /// the two barriers should not drift apart in a yaml — but they are not the
    /// same barrier, and the robots this bound is about are the ones already
    /// standing at the cell, whom doReturnSync holds on the appointment wait.
    /// The mid-run wait belongs to a PURSUIT reunion, and no pursuit consults
    /// this schedule at all.
    ///
    /// SO IT IS 0 = UNCAPPED ON THE CAMPAIGN DEFAULT, `capped` false on every
    /// row, and that is the honest answer rather than a dead column: a barrier
    /// nobody walks away from cannot be outrun by any interval, so there is no
    /// findability bound left to state. The node says so once at startup so the
    /// constant is legible as configuration rather than as a bug.
    ///
    /// THE FLOOR OUTRANKS THIS CAP in solve(), so the bound is not guaranteed;
    /// when it fails, the node warns once with the numbers (see
    /// deriveRendezvousProposal). Cutting below the drive would shrink the
    /// integer without shortening the journey.
    long long max_interval_ms = 0;

    /// Cells to drop from consideration: a no-show write-off list.
    ///
    /// THE NODE NO LONGER USES THIS. It hard-clears the vector before every
    /// solve (explo_planner_node.cpp, deriveRendezvousProposal) and the
    /// mechanism is retired, so `RendezvousPlan::rejected_excluded` is 0 on
    /// every row a campaign will produce. Kept only because the unit tests
    /// exercise it and because deleting a Config field is a wire-compatibility
    /// event for no gain.
    ///
    /// It was retired for a reason that outlives it: the list was PER ROBOT,
    /// and a per-robot input to a value the whole team must share is the exact
    /// mistake gen 9 made everywhere else. Nor can it return as a shared input
    /// — the team would first have to agree on the write-offs, which is the
    /// same agreement problem one level down. Anti-deadlock is now the wait cap
    /// and the one-appointment-per-outage rule instead, both of which are
    /// local, terminating, and need nobody's consent.
    ///
    /// When it is non-empty it NEVER applies to the floor (the loop skips
    /// `floor_cell` and re-adds it unconditionally afterwards). That is not a
    /// property of what the floor happens to be — it is the point of having
    /// one: if exclusion could empty the candidate set, the no-show handler
    /// would have reinvented the deadlock it existed to prevent. The floor the
    /// node supplies is the team's own vehicle centroid, which is reachable by
    /// construction for whoever is nearest it, but this guarantee holds for any
    /// floor the caller passes.
    std::vector<int> exclude;
  };

  /// Derive the appointment.
  ///
  /// `robots` is the vehicle set exactly as it was handed to
  /// GlobalAllocator::solve, and `alloc` is that solve's result — index-aligned
  /// with `robots`, which is what lets this function attribute a tour to a
  /// starting cell without searching by id. Handing in an allocation solved
  /// from a DIFFERENT vehicle set is the one misuse that cannot be detected
  /// here and will silently produce a plan neither robot can keep.
  ///
  /// `floor_cell` is the caller's GUARANTEED CANDIDATE: a grid cell admitted
  /// unconditionally, exempt from `Config::exclude` and from the reachability
  /// rule, so that a solve cannot refuse merely because the tours were empty.
  /// Pass -1 to supply none — and understand that -1 means the candidate set
  /// CAN be empty and `solve` can then refuse with nothing to fall back on.
  /// That is not hypothetical: it cost the first ts4 smoke its whole N=4 rung.
  ///
  /// It is NOT the last-contact midpoint. That quantity was removed on
  /// 2026-09-16 and does not even exist at proposal time — the proposal is
  /// derived while the team is still connected, so there is no last contact to
  /// take a midpoint of. What the node passes today is the centroid of the
  /// snapshot's own vehicle cells: defined for any N, drawn from the frozen
  /// shared snapshot rather than anything private, and in-ROI by convexity.
  /// There is no "raw midpoint" for a caller to keep using.
  ///
  /// `mission_elapsed_ms` is the shared clock. It is the ONLY time input, and
  /// it is a parameter rather than a call to now() so this stays a pure
  /// function of values both robots hold.
  static RendezvousPlan solve(const CellWorld& world,
                              const std::vector<AllocRobot>& robots,
                              const Allocation& alloc,
                              int floor_cell,
                              long long mission_elapsed_ms,
                              const Config& cfg);

  /// Time to cover `mm` at the configured speed, in ms. Integer division:
  /// truncation is deterministic and costs at most a millisecond, where a
  /// double would cost cross-process agreement.
  static long long travelMs(long long mm, long long speed_mm_s);

  // THE DEPARTURE RULE IS GONE (generation 19, 2026-09-17); THE SCHEDULE CAME
  // BACK (generation 23).
  //
  // `occurrenceAtOrAfter` turned one frozen (phase, period) pair into a
  // standing meeting, and `shouldDepart` made each robot leave early enough
  // that a staggered departure produced a synchronised arrival. Both were
  // deleted here. The recurrence came back as nextAgreedOccurrence in
  // planner_util.hpp, a pure function with executable tests, because N robots
  // departing on private countdowns for an instant they never discussed is not
  // a rendezvous.
  //
  // THE LEAVE-EARLY RULE CAME BACK TOO (generation 29, 2026-09-19), in the node
  // rather than here: ExploPlannerNode::appointmentDue departs when
  // `now + appointmentLeadMs >= t_meet_ms`, degrading to the bare comparison
  // when the robot is off the snapshot grid and cannot price its own drive. It
  // is NOT a restoration of shouldDepart's arithmetic — it reads a live lead
  // each tick against an instant the team agreed, where shouldDepart aimed at a
  // deadline each robot had computed privately.
  //
  // WHAT THE SCHEDULE LOOKS LIKE NOW. A robot whose reconnect trigger fires
  // signs up to the first agreed occurrence it can still ARRIVE at within
  // rendezvous_max_lateness_sec, leaves in time to be there, and waits until
  // the team is whole. The residual spread is still absorbed by an unbounded
  // barrier wait rather than by an inequality between a period and a drive —
  // but it is now the spread of robots that could not make the rung at all,
  // not the spread of everyone's drive distance.
  //
  // WHY IT WAS REMOVED, IN ONE MEASUREMENT — and see nextAgreedOccurrence in
  // planner_util.hpp for why the measurement is still real and no longer
  // decides the question. The timetable had to satisfy three inequalities at
  // once — worst-case drive <= period <= barrier wait cap, AND arming spread <=
  // period — and no period satisfies all three: the worst-case in-ROI drive is
  // ~283 s (100x100 m at 0.5 m/s), 340 s with the safety markup, against a
  // 240 s cap. The N=3 smoke showed the failure directly: three robots armed
  // 16.1 / 52.5 / 67.4 s apart against a 30 s period, so they selected three
  // DIFFERENT occurrences (t+34 / t+64 / t+94) off byte-identical integers.
  // Two of them met; the third arrived to an empty cell.
  //
  // `depart_safety_milli` survives this deletion because it is also the
  // multiplier on the reachability floor in solve() — and, since generation 29,
  // on the node's departure lead, deliberately the same constant so a robot
  // never needs more lead than the spacing the floor sized for it.
  // `depart_margin_ms` did not survive, and is gone with shouldDepart, its only
  // reader.
};

// ---------------------------------------------------------------------------
// THE HANDSHAKE, as opposed to the solve above.
// ---------------------------------------------------------------------------
//
// Deciding WHERE to meet is the scheduler's job and it is a search. Deciding
// that the whole team is holding the same answer is a different job, it is a
// state machine, and until 2026-09-17 it lived entirely inside a 14k-line ROS
// node where nothing could reach it. That is not an aesthetic complaint: the
// two defects that killed the N>=3 rendezvous arm were both in these few lines,
// both survived three code reviews, and neither was reproducible in under ten
// minutes of wall clock because the only way to run the code was to run a cell.
//
// What is here is ONLY the decisions — no ROS, no clock, no I/O, no proposal
// type of its own. The node keeps its control flow, its logging and its own
// nested RendezvousProposal; it delegates the two questions that turned out to
// be hard. A test can then enumerate the transitions exhaustively, which is
// what `RendezvousHandshake` in test_rendezvous_scheduler.cpp does.
struct RendezvousHandshake {
  /// What a non-proposer should do with the triple the proposer is currently
  /// publishing.
  ///
  /// THE PERMITTED SEQUENCE IS empty -> P -> R -> R' -> R'' ..., where P is the
  /// centroid placeholder (`provisional`) and each R is a tour-informed choice.
  /// The first two steps are free-standing; every step after them must be
  /// EARNED, and what earns one is the team having actually kept the meeting R
  /// names (generation 29 — the user's rule is that the next place and time are
  /// agreed before anyone resumes exploring). A robot signals that it is owed a
  /// replacement with `held_reagree_due`, and outside that window a second
  /// generation is still the protocol violation it has always been.
  ///
  /// So the count of adoptions in a run is no longer two: it is two plus one
  /// per meeting kept. What remains invariant — and is the property the commit
  /// gate rests on — is that a robot only ever moves off R when it is expecting
  /// to, so the fleet cannot be walked onto a new generation mid-outage.
  enum class Adopt {
    kIgnore,     ///< Nothing to adopt, or already holding exactly this.
    kTake,       ///< Hold nothing; take the proposer's triple.
    kUpgrade,    ///< Hold the placeholder and the proposer has moved off it.
    kReagree,    ///< Kept the last meeting; this is the replacement it is owed.
    kConflict,   ///< The proposer is publishing a triple that is none of those.
  };

  /// @param peer_valid        the proposer is publishing a usable triple
  /// @param peer_provisional  ...and it is flagged as the centroid placeholder
  /// @param held_valid        this robot already holds a triple
  /// @param held_provisional  ...and what it holds is the placeholder
  /// @param peer_equals_held  the two triples' three integers are identical
  /// @param held_reagree_due  this robot kept the meeting it holds and is
  ///                          waiting to be told the next place and time
  ///
  /// The flags are passed rather than the triples so that this cannot acquire
  /// an opinion about what a triple IS. Note in particular that
  /// `peer_equals_held` is deliberately independent of the provisional flags:
  /// the protocol's commit comparison is over the three integers alone, and a
  /// robot that cleared its own flag without the numbers changing has not
  /// changed what it will drive to.
  ///
  /// `held_reagree_due` is the CALLER'S to clear, and it must clear it on
  /// kReagree. Left set it degrades to "accept any replacement forever", which
  /// is the fleet-split this enum's refusals exist to prevent.
  static Adopt adopt(bool peer_valid, bool peer_provisional,
                     bool held_valid, bool held_provisional,
                     bool peer_equals_held, bool held_reagree_due);

  /// Update what peer `id` has been HEARD to hold, given its newest message.
  ///
  /// The latch exists because at N>=3 the echoes do not coincide — requiring
  /// them to was a deadlock (see maintainRendezvousProposal) — so each is
  /// recorded when it arrives and the commit counts records. The cost of that
  /// is that a record can be contradicted later, and THIS FUNCTION IS WHERE THE
  /// CONTRADICTION IS ACTED ON: a message that disagrees with the record clears
  /// it, on the tick it arrives, from the peer's own words.
  ///
  /// Dropping only on heard-nothing, which is what the node did until
  /// 2026-09-17, is a FALSE-COMMIT bug and not a missed-commit one: a peer that
  /// upgrades P -> R leaves its old P record standing, the holder of P counts it
  /// toward fleet-1, announces an agreement the peer does not share, and drives
  /// to a cell nobody else is coming to. Committing something false is strictly
  /// worse than committing nothing.
  ///
  /// @param heard   the peer's newest published triple (may be invalid)
  /// @param held    what THIS robot holds; a record only counts if it matches
  /// @param latched in/out, the record for that peer
  template <class Triple>
  static void updatePeerLatch(const Triple& heard, const Triple& held,
                              Triple& latched) {
    // Order matters. Clear on contradiction FIRST, then re-record, so that a
    // peer moving straight from one triple to another in a single message both
    // loses the old record and gains the new one without a tick in between —
    // and so that a peer moving to something that is NOT ours ends with no
    // record at all rather than keeping the convenient one.
    if (!(heard == latched)) latched = Triple{};
    if (heard.valid() && heard == held) latched = heard;
  }
};

}  // namespace explo_planner
