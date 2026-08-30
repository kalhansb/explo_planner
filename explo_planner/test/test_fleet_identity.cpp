#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "explo_planner/fleet_identity.hpp"

using namespace explo_planner;

// --- Masks ---

TEST(FleetIdentity, BitAndMembership) {
  EXPECT_EQ(robotBit(0), 1u);
  EXPECT_EQ(robotBit(1), 2u);
  EXPECT_EQ(robotBit(7), 128u);
  // Out-of-range ids give the empty bit rather than shifting off the end: a
  // mask built from unvalidated input must be empty, never aliased onto
  // somebody else's bit.
  EXPECT_EQ(robotBit(-1), 0u);
  EXPECT_EQ(robotBit(kMaxTeamSize), 0u);
  EXPECT_EQ(robotBit(1000), 0u);

  const uint32_t m = robotBit(0) | robotBit(3);
  EXPECT_TRUE(maskHas(m, 0));
  EXPECT_TRUE(maskHas(m, 3));
  EXPECT_FALSE(maskHas(m, 1));
  EXPECT_FALSE(maskHas(m, -1));
  EXPECT_FALSE(maskHas(m, kMaxTeamSize));
  EXPECT_EQ(maskCount(m), 2);
  EXPECT_EQ(maskCount(0u), 0);
  // Bits past the policy cap are not counted, so a garbage word cannot inflate
  // "how many robots know this cell".
  EXPECT_EQ(maskCount(0xFFFFFFFFu), kMaxTeamSize);
}

// --- Hash ---

// The hash is a wire value compared across processes. These are the properties
// consumers rely on; the exact digest is not one of them, only its stability
// within a build, which the golden below pins.
TEST(FleetIdentity, HashSeparatesFleets) {
  const std::vector<std::string> ab{"atlas", "bestla"};
  EXPECT_EQ(teamNamesHash(ab), teamNamesHash({"atlas", "bestla"}));
  // Order defines the ids, so it must change the hash.
  EXPECT_NE(teamNamesHash(ab), teamNamesHash({"bestla", "atlas"}));
  // Membership changes it.
  EXPECT_NE(teamNamesHash(ab), teamNamesHash({"atlas", "bestla", "curt"}));
  EXPECT_NE(teamNamesHash(ab), teamNamesHash({"atlas"}));
  // The NUL separator: without it these two would collide, and two fleets
  // partitioned differently would believe they agreed.
  EXPECT_NE(teamNamesHash({"ab", "c"}), teamNamesHash({"a", "bc"}));
  EXPECT_EQ(teamNamesHash({}), 2166136261u);  // FNV-1a offset basis
}

// Golden value. FNV-1a is fully specified, so this is computable by hand and by
// any other implementation; pinning it means a "harmless" refactor of the hash
// cannot silently make two builds of this binary disagree on the wire while
// every property test above still passes.
TEST(FleetIdentity, HashGolden) {
  // FNV-1a over "atlas\0bestla\0".
  uint32_t h = 2166136261u;
  for (const char c : std::string("atlas\0bestla\0", 13)) {
    h ^= static_cast<unsigned char>(c);
    h *= 16777619u;
  }
  EXPECT_EQ(teamNamesHash({"atlas", "bestla"}), h);
}

// --- Resolution ---

// The default. Empty param is legal and is NOT an error: it is every launch
// that predates this file, and the per-phase equivalence gate requires those
// to keep working untouched.
TEST(FleetIdentity, EmptyIsUnconfiguredNotAnError) {
  const FleetIdentity id = makeFleetIdentity({}, "atlas");
  EXPECT_FALSE(id.configured);
  EXPECT_TRUE(id.error.empty());
  EXPECT_EQ(id.self_id, -1);
  EXPECT_EQ(id.team_hash, 0u);
  EXPECT_EQ(id.size(), 0);
  EXPECT_EQ(id.allMask(), 0u);
}

TEST(FleetIdentity, ResolvesPositionAsId) {
  const FleetIdentity a = makeFleetIdentity({"atlas", "bestla", "curt"}, "atlas");
  ASSERT_TRUE(a.configured) << a.error;
  EXPECT_EQ(a.self_id, 0);
  EXPECT_EQ(a.size(), 3);
  EXPECT_EQ(a.allMask(), 0b111u);
  EXPECT_EQ(a.idOf("curt"), 2);
  EXPECT_EQ(a.idOf("nobody"), -1);
  EXPECT_EQ(a.nameOf(1), "bestla");
  EXPECT_EQ(a.nameOf(9), "");
  EXPECT_EQ(a.nameOf(-1), "");

  // The same array on a different robot yields the same hash and a different
  // id — the whole point of declaring identity out of band.
  const FleetIdentity c = makeFleetIdentity({"atlas", "bestla", "curt"}, "curt");
  ASSERT_TRUE(c.configured) << c.error;
  EXPECT_EQ(c.self_id, 2);
  EXPECT_EQ(c.team_hash, a.team_hash);
}

TEST(FleetIdentity, RejectsSelfNotInFleet) {
  const FleetIdentity id = makeFleetIdentity({"atlas", "bestla"}, "curt");
  EXPECT_FALSE(id.configured);
  EXPECT_NE(id.error.find("curt"), std::string::npos);
  // The message must name the fleet it checked against, or an operator cannot
  // tell a typo in robot_name from a typo in the array.
  EXPECT_NE(id.error.find("atlas"), std::string::npos);
}

TEST(FleetIdentity, RejectsDuplicates) {
  const FleetIdentity id =
      makeFleetIdentity({"atlas", "bestla", "atlas"}, "atlas");
  EXPECT_FALSE(id.configured);
  EXPECT_NE(id.error.find("twice"), std::string::npos);
}

TEST(FleetIdentity, RejectsEmptyEntry) {
  const FleetIdentity id = makeFleetIdentity({"atlas", ""}, "atlas");
  EXPECT_FALSE(id.configured);
  EXPECT_NE(id.error.find("empty"), std::string::npos);
}

TEST(FleetIdentity, RejectsOversizeFleet) {
  std::vector<std::string> names;
  for (int i = 0; i <= kMaxTeamSize; ++i) names.push_back("r" + std::to_string(i));
  const FleetIdentity id = makeFleetIdentity(names, "r0");
  EXPECT_FALSE(id.configured);
  EXPECT_NE(id.error.find("cap"), std::string::npos);

  // Exactly at the cap is fine, and its mask fills the cap.
  names.pop_back();
  const FleetIdentity ok = makeFleetIdentity(names, "r0");
  ASSERT_TRUE(ok.configured) << ok.error;
  EXPECT_EQ(maskCount(ok.allMask()), kMaxTeamSize);
}

// --- The startup guard ---

TEST(FleetIdentity, RequireNamesTheFeature) {
  const FleetIdentity none = makeFleetIdentity({}, "atlas");
  const std::string msg = requireFleetIdentity(none, "global_alloc_enabled");
  ASSERT_FALSE(msg.empty());
  // Naming the feature is the point: "team_robot_names is empty" alone does not
  // tell an operator which knob they turned to make it matter.
  EXPECT_NE(msg.find("global_alloc_enabled"), std::string::npos);
  EXPECT_NE(msg.find("team_robot_names"), std::string::npos);

  // A malformed array reports the malformation, not "empty" — the two need
  // different fixes.
  const FleetIdentity bad = makeFleetIdentity({"atlas", "atlas"}, "atlas");
  const std::string bad_msg = requireFleetIdentity(bad, "team_world_hz");
  ASSERT_FALSE(bad_msg.empty());
  EXPECT_NE(bad_msg.find("twice"), std::string::npos);
  EXPECT_EQ(bad_msg.find("is empty"), std::string::npos);

  EXPECT_TRUE(
      requireFleetIdentity(makeFleetIdentity({"atlas"}, "atlas"), "x").empty());
}
