#include "social_nav_human_model/groups.hpp"

#include <algorithm>
#include <numeric>

namespace social_nav_human_model
{

namespace
{
// Simple union-find for connected-components grouping.
struct UnionFind
{
  std::vector<int> parent;
  explicit UnionFind(std::size_t n) : parent(n)
  {
    std::iota(parent.begin(), parent.end(), 0);
  }
  int find(int x)
  {
    while (parent[x] != x) {
      parent[x] = parent[parent[x]];  // path halving
      x = parent[x];
    }
    return x;
  }
  void unite(int a, int b) { parent[find(a)] = find(b); }
};
}  // namespace

std::vector<Group> detectGroups(
  const std::vector<HumanForGrouping> & humans, const GroupingParams & params)
{
  std::vector<Group> groups;
  const std::size_t n = humans.size();
  if (n < 2) {
    return groups;
  }

  UnionFind uf(n);
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = i + 1; j < n; ++j) {
      const double gap = (humans[i].position - humans[j].position).norm();
      const double vel_diff = (humans[i].velocity - humans[j].velocity).norm();
      if (gap <= params.group_radius && vel_diff <= params.velocity_similarity) {
        uf.unite(static_cast<int>(i), static_cast<int>(j));
      }
    }
  }

  // Bucket members by component root.
  std::vector<std::vector<std::size_t>> components(n);
  for (std::size_t i = 0; i < n; ++i) {
    components[uf.find(static_cast<int>(i))].push_back(i);
  }

  int group_id = 0;
  for (const auto & members : components) {
    if (members.size() < 2) {
      continue;  // lone humans are not groups
    }
    Group g;
    g.id = group_id++;
    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    Eigen::Vector2d mean_vel = Eigen::Vector2d::Zero();
    for (std::size_t idx : members) {
      g.member_ids.push_back(humans[idx].id);
      centroid += humans[idx].position;
      mean_vel += humans[idx].velocity;
    }
    const double inv = 1.0 / static_cast<double>(members.size());
    g.centroid = centroid * inv;
    g.velocity = mean_vel * inv;
    double max_r = 0.0;
    for (std::size_t idx : members) {
      max_r = std::max(max_r, (humans[idx].position - g.centroid).norm());
    }
    g.radius = max_r;
    groups.push_back(g);
  }
  return groups;
}

}  // namespace social_nav_human_model
