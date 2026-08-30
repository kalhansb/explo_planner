#pragma once
/// @file fleet_identity.hpp
/// @brief Stable numeric robot ids agreed by the whole fleet.
///
/// Everything the M-TARE evolution adds — per-cell `known_by` bitmasks, the
/// allocator's cross-robot tie-break, comms `in_range_mask`, the gossip arrays
/// indexed by robot, lowest-id rendezvous adoption — needs a numeric id that
/// means the same thing on every robot. The planner did not have one.
///
/// What it had instead:
///   - `RobotIntent.robot_id`, a free-form string. Strings cannot address a
///     bit, and the only ordering available (lexicographic) is not stable
///     under a rename and is not what any of the above want.
///   - the comms emulator's robot-index topic, which is COMMS=1-only, N=2-only,
///     and documented as allowed to be unusable at runtime. Deriving fleet
///     identity from a facility the fleet is allowed to run without would make
///     every new mechanism silently inert in exactly the arms that do not use
///     the emulator.
///
/// So identity is a config fact, declared out of band: the ordered string array
/// param `team_robot_names`, whose position IS the id. Its hash rides on
/// TeamWorld so a fleet running two different definitions is caught rather
/// than quietly reinterpreting each other's masks.
///
/// UNCONFIGURED IS A LEGAL STATE, and that is deliberate. The param defaults to
/// empty, which yields `configured() == false` and no error — the identical
/// behaviour of every launch that predates this file. Making it hard-required
/// would break every existing config and violate the per-phase equivalence
/// gate ("at defaults the binary is behaviourally identical to its parent").
/// The requirement bites where it matters instead: any caller that needs ids
/// asks `requireFor()` and gets a fatal, named config error if identity is
/// missing. Absent identity therefore cannot silently degrade a mechanism into
/// doing nothing — it refuses to start.

#include <cstdint>
#include <string>
#include <vector>

namespace explo_planner {

/// Policy cap on team size. A POLICY cap, not a representation one: the masks
/// are uint32 and would hold 32. Eight is the largest fleet any of the
/// data structures here have been reasoned about at, and a cap that is checked
/// beats a cap that is assumed — an unnoticed 33rd robot would shift bits off
/// the end of every mask and corrupt the knowledge gate silently.
inline constexpr int kMaxTeamSize = 8;

/// Bit for robot `id`. Returns 0 for out-of-range ids, so a mask built from
/// unvalidated input is empty rather than wrong.
inline uint32_t robotBit(int id) {
  if (id < 0 || id >= kMaxTeamSize) return 0u;
  return 1u << static_cast<unsigned>(id);
}

/// Is robot `id` present in `mask`? False for out-of-range ids.
inline bool maskHas(uint32_t mask, int id) {
  const uint32_t b = robotBit(id);
  return b != 0u && (mask & b) != 0u;
}

/// Number of robots named in `mask`, counting only bits inside the policy cap.
int maskCount(uint32_t mask);

/// FNV-1a 32-bit over the names joined by NUL, in order.
///
/// Hand-rolled rather than std::hash because this value goes ON THE WIRE and
/// is compared between processes: std::hash is implementation-defined and is
/// permitted to differ between two libstdc++ versions, let alone two
/// standard libraries. A config check whose answer depends on which machine
/// compiled the binary is worse than no check — it would report a config
/// mismatch on a correctly configured fleet, and the fix people would reach
/// for is deleting the check.
///
/// Order matters (it is what defines the ids), so this is deliberately NOT
/// invariant to permutation: two robots that agree on the membership but not
/// the order do not agree on identity.
uint32_t teamNamesHash(const std::vector<std::string>& names);

/// The fleet definition this robot is running, resolved from the param.
struct FleetIdentity {
  /// A usable identity was resolved. False both for "not configured" (empty
  /// param — legal, legacy) and for "configured wrongly" (see `error`).
  bool configured = false;
  /// This robot's index in `names`, or -1 when not configured.
  int self_id = -1;
  /// teamNamesHash(names), or 0 when not configured.
  uint32_t team_hash = 0;
  /// The validated, ordered fleet. Empty when not configured.
  std::vector<std::string> names;
  /// Human-readable reason the param was rejected; empty when the param was
  /// accepted OR when it was absent. Distinguishing those two is the caller's
  /// job and is what `configured` plus emptiness of `names` is for: an absent
  /// param is legacy operation, a malformed one is a config error the node
  /// must refuse to run past.
  std::string error;

  /// Mask naming every robot in the fleet. 0 when not configured.
  uint32_t allMask() const;
  /// Number of robots. 0 when not configured.
  int size() const { return static_cast<int>(names.size()); }
  /// Index of `name`, or -1 if it is not in the fleet.
  int idOf(const std::string& name) const;
  /// Name of `id`, or "" if out of range. For log lines: every message about a
  /// peer should be able to say who, not just which bit.
  const std::string& nameOf(int id) const;
};

/// Resolve `team_robot_names` for the robot called `self_name`.
///
/// Empty `names` -> unconfigured, no error (the default, legacy path).
/// Otherwise the array must be a well-formed fleet definition or the result
/// carries `error` and `configured == false`:
///   - at most kMaxTeamSize entries;
///   - no empty entry (an empty name cannot be matched against `robot_name`,
///     and would give a bit nobody can claim);
///   - no duplicates (two robots would share one bit, and the knowledge gate
///     would credit one with the other's observations);
///   - `self_name` must appear (a robot that is not in its own fleet cannot
///     set its own bit, so every mask it publishes would be a lie).
FleetIdentity makeFleetIdentity(const std::vector<std::string>& names,
                                const std::string& self_name);

/// Startup guard for a feature that cannot work without ids. Returns "" when
/// `id` is usable, otherwise the message the node should die with — naming the
/// FEATURE, because "team_robot_names is empty" on its own does not tell an
/// operator which knob they turned to make it matter.
std::string requireFleetIdentity(const FleetIdentity& id,
                                 const char* feature_name);

}  // namespace explo_planner
