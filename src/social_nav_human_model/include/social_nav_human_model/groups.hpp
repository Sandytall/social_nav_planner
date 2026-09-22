// Human group detection (MASTER_PROMPT §13).
//
// Humans that are close together AND moving similarly form a group. The planner should
// prefer to go around a group rather than drive through its middle, so we expose the
// group centroid, mean velocity, members and a bounding radius (used by GroupIntrusionCost).
//
// Grouping is transitive (A-B linked, B-C linked => {A,B,C}) via connected components.
#ifndef SOCIAL_NAV_HUMAN_MODEL__GROUPS_HPP_
#define SOCIAL_NAV_HUMAN_MODEL__GROUPS_HPP_

#include <cstdint>
#include <vector>

#include <Eigen/Core>

namespace social_nav_human_model
{

/// Minimal per-human input for grouping (subset of a Track).
struct HumanForGrouping
{
  std::uint64_t id{0};
  Eigen::Vector2d position{0.0, 0.0};
  Eigen::Vector2d velocity{0.0, 0.0};
};

struct GroupingParams
{
  double group_radius{1.5};        ///< max member spacing to link, metres (§15 group_radius)
  double velocity_similarity{0.5}; ///< max velocity difference to link, m/s
};

struct Group
{
  int id{-1};
  std::vector<std::uint64_t> member_ids;
  Eigen::Vector2d centroid{0.0, 0.0};
  Eigen::Vector2d velocity{0.0, 0.0};  ///< mean member velocity
  double radius{0.0};                  ///< centroid to farthest member
};

/// Detect groups among the given humans. Only clusters of >= 2 members are returned;
/// lone humans are not groups (§13).
std::vector<Group> detectGroups(
  const std::vector<HumanForGrouping> & humans, const GroupingParams & params);

}  // namespace social_nav_human_model

#endif  // SOCIAL_NAV_HUMAN_MODEL__GROUPS_HPP_
