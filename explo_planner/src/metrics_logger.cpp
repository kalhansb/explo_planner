#include "explo_planner/metrics_logger.hpp"

#include <cerrno>
#include <cstring>
#include <iomanip>
#include <stdexcept>

namespace explo_planner {

MetricsLogger::MetricsLogger(const std::string& csv_path)
    : path_(csv_path), file_(csv_path, std::ios::out | std::ios::trunc) {
  // Fail loudly. std::ofstream does not throw by default, so an unwritable
  // output_csv (missing parent directory, read-only mount, bad permissions)
  // used to leave every subsequent `file_ << ...` a silent no-op: the run
  // completed, the node logged "Step N logged" for every step, and the
  // experiment produced no data at all. There is nothing to salvage from a
  // metrics run whose metrics cannot be written, so refuse to start.
  //
  // The open test alone was NOT enough, which is why noteStreamState() exists
  // below: a successful open says nothing about the writes that follow it, and
  // the two cases that matter both open cleanly. "/dev/full" accepts the open
  // and rejects every write; a disk that fills at step 2000 of 3000 accepts the
  // first 2000. Both reproduce the exact silence this constructor was written
  // to end, one flush later. (2026-09-18)
  if (!file_.is_open()) {
    throw std::runtime_error(
        "MetricsLogger: cannot open output CSV '" + csv_path + "': " +
        std::strerror(errno));
  }
}

void MetricsLogger::noteStreamState(const char* where) {
  // Sticky and first-wins. Once failbit or badbit is set every subsequent `<<`
  // is a no-op and the stream stays failed for the rest of the run, so checking
  // per row would otherwise restate the same failure on every one of the
  // thousands of ticks that follow it. fail() is the right predicate rather
  // than bad(): a formatting failure loses the row just as completely as an I/O
  // error does, and both leave a CSV that no longer matches its own header.
  if (!error_.empty()) return;
  if (!file_.fail()) return;
  error_ = std::string("MetricsLogger: ") + where + " write to '" + path_ +
           "' failed (" + std::strerror(errno) +
           "); CSV is truncated from this point on";
}

MetricsLogger::~MetricsLogger() {
  if (file_.is_open()) file_.close();
}

namespace {
/// Replace anything that would change the column count. An empty string stays
/// EMPTY -- there is no "-" sentinel, and the comment here used to promise one
/// the code has never written. That promise is worse than no comment: a parser
/// written against it tests `field == "-"` for "not pursuing", never matches,
/// and classifies every non-pursuing row as a chase with an unnamed quarry.
/// The dead `if (s.empty()) return "";` that sat here -- a branch returning
/// exactly what the loop below would have returned -- was the tell.
std::string sanitizeField(const std::string& s) {
  std::string out = s;
  for (char& ch : out)
    if (ch == ',' || ch == '\n' || ch == '\r' || ch == '"') ch = '_';
  return out;
}
}  // namespace

void MetricsLogger::writeHeader() {
  file_ << "step,sim_time_sec,total_observed_voxels,frontier_voxels,"
        << "distance_traveled,selected_score,plan_time_ms,"
        << "mean_eig,mean_entropy,mean_variance,"
        << "mean_info_gain,mean_path_cost,"
        << "selected_info_gain,selected_path_cost,selected_utility,"
        << "coord_active_peers,rejected_by_minpos,rejected_by_unreachable,"
        << "phase,target_id,vantage_index,n_vantages_valid,"
        << "vantage_los_clear,dwell_sec,"
        << "prox_hold_count,prox_hold_total_sec,"
        // Appended, never inserted. Every reader in this tree resolves columns
        // by header name — csv.DictReader in the python, and a header-scanning
        // awk in run_explo_sim_rviz.sh — so inserting would not in fact break
        // any of them; a comment here used to claim otherwise and was wrong.
        // The rule is kept anyway, for the readers that are NOT in this tree.
        // A campaign archive outlives the binary that wrote it and gets opened
        // by whatever is to hand months later, and a positional read of an
        // old file against a new schema does not fail, it silently returns the
        // wrong column. Appending costs one out-of-place block of names; that
        // is the whole price, and it is paid below three times over.
        << "state,reconnect_range_to_goal_m,reconnect_elapsed_sec,"
        << "unknown_fraction,coverage_source,"
        // Same rule: belongs beside mean_info_gain by meaning, but goes here
        // because the schema only ever grows at the right-hand end.
        << "info_gain_std,"
        // Same rule again (separation.hpp). These belong beside
        // rejected_by_minpos — they are the manipulation checks for the
        // mechanism that replaces it — and they are here instead.
        << "sep_peer_dist_m,sep_discount,sep_reordered,sep_eligible_peers,"
        // Same append-only rule. These belong next to rejected_by_unreachable
        // by meaning and are here instead. Unlike every other block in this
        // header they are populated on timer rows too — see the struct.
        << "plan_cand_total,plan_rej_close,plan_rej_map,plan_rej_unreach,"
        << "plan_rej_blacklist,plan_rej_minpos,plan_stall_ticks,"
        // R5, and the same append-only rule one more time: these belong beside
        // the reconnect_* block by meaning and are here instead.
        << "pursue_peer,pursue_quarry_live,team_complete,"
        // v8. Belongs immediately after plan_rej_blacklist by meaning — it is
        // that column's visited half — and is here instead, for the same
        // reason as every block above: inserting it where it reads would shift
        // FIVE columns under every positional reader of every banked run —
        // plan_rej_minpos, plan_stall_ticks, pursue_peer, pursue_quarry_live,
        // team_complete, which is everything between the insertion point and
        // the end of the header. (This said "eight" until 2026-09-18; count it
        // from the header above, which is the only place the order is real.)
        // Five is not a smaller argument than eight. One shifted column is
        // enough to make every banked run's positional reader wrong.
        << "plan_rej_visited\n";
  header_written_ = true;
  // A header that did not reach the file is the one failure a by-name reader
  // cannot recover from at all -- the rows below are then a nameless matrix of
  // numbers -- so it is checked separately from the rows rather than waiting for
  // the first logStep().
  noteStreamState("header");
}

void MetricsLogger::logStep(const StepMetrics& m) {
  if (!header_written_) writeHeader();
  file_ << m.step << ","
        // Fixed-point, not the default 6 SIGNIFICANT digits: on hardware
        // sim_time_sec is a Unix epoch stamp (~1.7e9), where 6 sig figs is
        // ±1000 s — every row of an hour-long run collapses to the same
        // value and dt-based post-processing divides by zero.
        << std::fixed << std::setprecision(6) << m.sim_time_sec
        << std::defaultfloat << ","
        << m.total_observed_voxels << ","
        << m.frontier_voxels << ","
        << m.distance_traveled << ","
        << m.selected_score << ","
        << m.plan_time_ms << ","
        << m.mean_eig << ","
        << m.mean_entropy << ","
        << m.mean_variance << ","
        << m.mean_info_gain << ","
        << m.mean_path_cost << ","
        << m.selected_info_gain << ","
        << m.selected_path_cost << ","
        << m.selected_utility << ","
        << m.coord_active_peers << ","
        << m.rejected_by_minpos << ","
        << m.rejected_by_unreachable << ","
        << m.phase << ","
        << m.target_id << ","
        << m.vantage_index << ","
        << m.n_vantages_valid << ","
        << m.vantage_los_clear << ","
        << m.dwell_sec << ","
        << m.prox_hold_count << ","
        << m.prox_hold_total_sec << ","
        << m.state << ","
        << m.reconnect_range_to_goal_m << ","
        << m.reconnect_elapsed_sec << ","
        << m.unknown_fraction << ","
        << m.coverage_source << ","
        << m.info_gain_std << ","
        << m.sep_peer_dist_m << ","
        << m.sep_discount << ","
        << m.sep_reordered << ","
        << m.sep_eligible_peers << ","
        << m.plan_cand_total << ","
        << m.plan_rej_close << ","
        << m.plan_rej_map << ","
        << m.plan_rej_unreach << ","
        << m.plan_rej_blacklist << ","
        << m.plan_rej_minpos << ","
        << m.plan_stall_ticks << ","
        // The only free-text field in the row that comes from configuration
        // rather than from this file, so it is the only one that can carry a
        // separator. A robot id with a comma in it would shift every column to
        // its right by one on that row and nowhere else — the worst kind of
        // corruption, because the file still parses. Substituted, not quoted:
        // a quoted field would be correct CSV but would break the awk
        // header-scanner in run_explo_sim_rviz.sh, which does not implement
        // quoting.
        << sanitizeField(m.pursue_peer) << ","
        << m.pursue_quarry_live << ","
        << m.team_complete << ","
        << m.plan_rej_visited << "\n";
  file_.flush();
  // Checked AFTER the flush, not after the insertions: the row can sit whole in
  // the stream buffer with every bit clear and only fail on the way to the
  // device, which is precisely how an ENOSPC presents. Flushing per row is
  // already this class's contract (a killed run must keep the steps it wrote),
  // so this adds a branch, not a syscall.
  noteStreamState("row");
}

} // namespace explo_planner
