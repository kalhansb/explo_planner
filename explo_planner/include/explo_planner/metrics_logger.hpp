#pragma once
/// @file metrics_logger.hpp
/// @brief Per-step CSV metric logging for exploration experiments.

#include <string>
#include <fstream>

namespace explo_planner {

struct StepMetrics {
  // Exploitation diagnostics. Exploration steps leave these at their defaults
  // ("explore"/-1/0); exploitation steps fill them from the active target and
  // the dwelled vantage so a CSV row attributes itself to a phase + target.
  std::string phase            = "explore";  // "explore" or "exploit"
  int    target_id             = -1;   // active target id, or -1 in explore
  int    vantage_index         = -1;   // index of the dwelled vantage, or -1
  int    n_vantages_valid      = 0;    // valid vantages this exploit plan tick
  int    vantage_los_clear     = 0;    // 1 if the dwelled vantage had clear LoS
  float  dwell_sec             = 0.0f; // dwell time held at the vantage

  int    step                  = 0;
  double sim_time_sec          = 0.0;
  int    total_observed_voxels = 0;
  int    frontier_voxels       = 0;
  float  distance_traveled     = 0.0f;
  float  selected_score        = 0.0f;
  float  plan_time_ms          = 0.0f;
  float  mean_eig              = 0.0f;
  float  mean_entropy          = 0.0f;
  float  mean_variance         = 0.0f;

  // Utility / cost-grid diagnostics. Populated by doPlan() once per step
  // from the candidate set evaluated this tick. mean_* are over all
  // candidates; selected_* are the components for the chosen candidate
  // (so post-hoc analysis can attribute the pick to either term).
  float  mean_info_gain        = 0.0f;
  // Population std of info_gain ACROSS the candidate set on this step — how
  // much the viewpoints actually disagree about information, which the mean
  // cannot show. U = info/(eps+cost) degenerates to argmin-cost as this goes
  // to zero: with every candidate scoring alike, the numerator stops
  // discriminating and only distance decides. It is therefore the quantity
  // that says whether a change to the FOV model helped or flattened the
  // field. NOTE: this is NOT adjacent to mean_info_gain in the CSV — see the
  // append-only note in metrics_logger.cpp.
  float  info_gain_std         = 0.0f;
  float  mean_path_cost        = 0.0f;
  float  selected_info_gain    = 0.0f;
  float  selected_path_cost    = 0.0f;
  /// EXACT ALIAS of selected_score — both are assigned current_goal_.score, and
  /// all 3432 rows of the g6pilot campaign were identical. The planner has one
  /// scalar objective; there is no second "utility" quantity for this to hold.
  /// It is NOT removed because the CSV is append-only and read positionally by
  /// archive scripts (see metrics_logger.cpp), so deleting a mid-file column
  /// would silently shift every column after it. Treat as redundant: never
  /// decompose score against utility, and never fit one on the other — that is
  /// a regression of a column on itself.
  ///
  /// The identity is checked at CAMPAIGN-SCORING time, not build time: the
  /// per-generation gate script (gate_g8.py check 15) re-derives it over every
  /// CSV row before any result is read. Those scripts are deliberately not in
  /// this tree — they pin one generation's identity and are written fresh per
  /// campaign — so nothing here enforces the alias, and a divergence would
  /// surface as a gate failure on the data rather than as a build failure.
  float  selected_utility      = 0.0f;

  // Multi-robot coordination diagnostics. Zero in single-robot mode.
  int    coord_active_peers       = 0;
  int    rejected_by_minpos       = 0;
  int    rejected_by_unreachable  = 0;

  // Coordinated proximity stop, CUMULATIVE across the run (like
  // distance_traveled): holds entered so far and total seconds spent held.
  // Per-step deltas come from differencing consecutive rows; without these
  // columns the only record of the yields was planner stdout.
  int    prox_hold_count          = 0;
  float  prox_hold_total_sec      = 0.0f;

  // Reconnection diagnostics. Rows are emitted on a periodic timer in EVERY
  // state, not just at LOG_STEP, so a minutes-long reconnect manoeuvre samples
  // the coverage curve instead of leaving a gap in it.
  //
  // ALL DURATIONS HERE ARE SIM SECONDS under use_sim_time (the node reads
  // this->now()/get_clock()). At RTF != 1 they are not wall seconds, and the
  // sampling period is likewise a sim-time period.
  //
  // `state` is the planner state at emit time, from stateName(). It is also the
  // row discriminator: a row with state == "LOG_STEP" is an end-of-step row
  // (plan-attribution columns populated); every other value is a timer row
  // (plan-attribution columns are zero by construction — there was no plan on
  // that tick to attribute them to). The timer deliberately skips LOG_STEP ticks
  // so the two kinds never collide.
  //
  // The reconnect_* pair is -1 whenever no manoeuvre is in flight, which keeps 0
  // meaning "arrived / just started" rather than "not applicable".
  // reconnect_elapsed_sec is raw elapsed sim time since the manoeuvre was armed
  // and is NOT bounded by pursuit_budget_max_sec: proximity-hold time is refunded to
  // the pursuit budget clock but not to this one, and in HYBRID the meeting-point
  // leg runs on the same clock after the chase is spent.
  //
  // reconnect_range_to_goal_m is REMAINING straight-line XY range to the
  // manoeuvre goal — a decreasing sample, not a travelled distance. Named for
  // what it is: as `reconnect_distance_m` it read like the manoeuvre's path
  // cost, which inverts the comparison it would be used for (a robot that
  // drives far and arrives ends near 0; one that gives up early stays large).
  // Travelled distance is `distance_traveled` differenced across the rows where
  // reconnect_elapsed_sec >= 0.
  std::string state                    = "UNKNOWN";
  float  reconnect_range_to_goal_m     = -1.0f;
  float  reconnect_elapsed_sec         = -1.0f;

  // Coverage-termination measure, as evaluated this tick, plus which source
  // produced it ("scovox" 2.5D column coverage of the ROI footprint, or
  // "planning_map" 2D cell coverage). This IS the primary endpoint quantity and
  // the one the DONE criterion is compared against, and until now it existed
  // nowhere in the CSV — it reached stdout only inside the branch where it had
  // already dropped below threshold, so a run could not be scored on the
  // criterion it terminates on. total_observed_voxels is not a substitute: it
  // is a 3D active-cell count whose relationship to column coverage varies with
  // occlusion and the z-band.
  //
  // -1 when it cannot be measured (source "planning_map" with no planning_map
  // received, or a degenerate ROI), matching coverageUnknownFraction().
  float  unknown_fraction         = -1.0f;
  std::string coverage_source     = "none";

  // Team-separation diagnostics (separation.hpp). Exploration rows only; an
  // exploitation row carries the "not applicable" defaults below, because the
  // vantage chooser does not consult the term. So does a row emitted by the
  // periodic metrics timer rather than by a planning step — that path builds a
  // fresh StepMetrics and never sees these fields at all. Averaging any of
  // these columns over a whole CSV therefore mixes in rows the term never
  // touched and dilutes the treatment toward "no effect": filter to
  // state == "LOG_STEP" and phase == "explore" first.
  //
  // The first two are measured in EVERY arm, including arms with the term
  // switched off, and that is the point of them. The mechanism this term
  // replaces, MinPos, has exactly one observable (rejected_by_minpos) and it
  // reads 0 for a whole campaign — which cannot distinguish a veto that never
  // mattered from a veto that never ran. These columns are built so the same
  // ambiguity cannot happen twice: an off-arm row still records how close to
  // its teammate the planner sent this robot, so the treated arm has a
  // measured counterfactual rather than an assumed one.
  //
  //   sep_peer_dist_m     metres from the SELECTED goal to the nearest
  //                       eligible teammate position; -1 for "no eligible
  //                       teammate" (none known, none fresh, or all finished).
  //                       Not the robot-to-robot separation — that is derivable
  //                       from the pose stream; this is what the planner chose.
  //   sep_discount        the multiplier actually applied to the selected
  //                       candidate's utility. 1.0 means unchanged, which is
  //                       what an off arm always writes. NOTE that this is
  //                       also the factor by which selected_score and
  //                       selected_utility above have ALREADY been reduced on
  //                       this row: the term scales the candidate's score in
  //                       place, so in a treated arm those two columns carry
  //                       the discounted number and in a control arm they
  //                       carry the raw one. To compare them across arms,
  //                       divide by this column first.
  //   sep_reordered       did the term change where the robot was sent?
  //                       1 yes, 0 no, -1 the question was not asked. It is a
  //                       comparison of PICKS, not of ordering: the planner is
  //                       re-run over the same candidates with the discount
  //                       removed and the two selections compared. -1 covers
  //                       the term being off, no candidate being selected, and
  //                       a pick that came from the blacklist-amnesty fallback,
  //                       which orders by failure age and which the term cannot
  //                       reach. -1 must not be pooled with 0: one means the
  //                       term did not move the goal, the other means nobody
  //                       asked.
  //   sep_eligible_peers  how many teammates were eligible to repel this tick.
  //                       Zero here makes every other column vacuous, so it is
  //                       the first thing to read. Structurally zero for a whole
  //                       run when TEAM_WORLD=0, since the term's anchors come
  //                       from the team model — such a cell yields no
  //                       counterfactual, only the confirmation that it cannot.
  float  sep_peer_dist_m          = -1.0f;
  float  sep_discount             = 1.0f;
  int    sep_reordered            = -1;
  int    sep_eligible_peers       = 0;
};

class MetricsLogger {
public:
  explicit MetricsLogger(const std::string& csv_path);
  ~MetricsLogger();

  void logStep(const StepMetrics& m);

private:
  std::ofstream file_;
  bool header_written_ = false;

  void writeHeader();
};

} // namespace explo_planner
