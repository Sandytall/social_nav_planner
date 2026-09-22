#include "social_nav_human_model/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>

namespace social_nav_human_model
{

HumanTracker::HumanTracker(const TrackerParams & params)
: params_(params)
{
}

const std::vector<Track> & HumanTracker::update(
  const std::vector<Detection> & detections, double stamp)
{
  const std::size_t n_tracks = internal_.size();
  const std::size_t n_dets = detections.size();

  std::vector<bool> track_matched(n_tracks, false);
  std::vector<bool> det_matched(n_dets, false);

  // Greedy nearest-neighbour association within the gating distance. Collect all
  // admissible pairs, then assign shortest-first so each side is used at most once.
  struct Pair
  {
    double dist;
    std::size_t track_idx;
    std::size_t det_idx;
  };
  std::vector<Pair> pairs;
  pairs.reserve(n_tracks * n_dets);
  for (std::size_t ti = 0; ti < n_tracks; ++ti) {
    for (std::size_t di = 0; di < n_dets; ++di) {
      const double d = (internal_[ti].track.position - detections[di].position).norm();
      if (d <= params_.max_association_distance) {
        pairs.push_back({d, ti, di});
      }
    }
  }
  std::sort(pairs.begin(), pairs.end(), [](const Pair & a, const Pair & b) {
    return a.dist < b.dist;
  });

  const double alpha = std::clamp(params_.velocity_filter_alpha, 1e-3, 1.0);

  for (const auto & p : pairs) {
    if (track_matched[p.track_idx] || det_matched[p.det_idx]) {
      continue;
    }
    track_matched[p.track_idx] = true;
    det_matched[p.det_idx] = true;

    Internal & it = internal_[p.track_idx];
    const Detection & det = detections[p.det_idx];
    const double dt = stamp - it.last_update_stamp;

    if (dt > 1e-6) {
      const Eigen::Vector2d measured_vel = (det.position - it.track.position) / dt;
      if (it.track.has_velocity) {
        const Eigen::Vector2d new_vel =
          alpha * measured_vel + (1.0 - alpha) * it.track.velocity;
        const Eigen::Vector2d measured_acc = (new_vel - it.track.velocity) / dt;
        it.track.acceleration =
          alpha * measured_acc + (1.0 - alpha) * it.track.acceleration;
        it.track.velocity = new_vel;
      } else {
        it.track.velocity = measured_vel;
        it.track.has_velocity = true;
      }
    }

    it.track.position = det.position;
    it.track.confidence = det.confidence;
    it.last_update_stamp = stamp;
    it.track.stationary =
      it.track.has_velocity && it.track.velocity.norm() < params_.stationary_speed_threshold;
  }

  // Unmatched detections spawn new tracks.
  for (std::size_t di = 0; di < n_dets; ++di) {
    if (det_matched[di]) {
      continue;
    }
    Internal it;
    it.track.id = next_id_++;
    it.track.position = detections[di].position;
    it.track.confidence = detections[di].confidence;
    it.creation_stamp = stamp;
    it.last_update_stamp = stamp;
    internal_.push_back(it);
  }

  // Age out tracks not seen for longer than the timeout. A single miss keeps the track
  // alive: we coast on the last known state until the timeout elapses.
  internal_.erase(
    std::remove_if(
      internal_.begin(), internal_.end(),
      [&](const Internal & it) {
        return (stamp - it.last_update_stamp) > params_.track_timeout;
      }),
    internal_.end());

  initialised_ = true;
  refreshSnapshot(stamp);
  return tracks_;
}

void HumanTracker::refreshSnapshot(double stamp)
{
  tracks_.clear();
  tracks_.reserve(internal_.size());
  for (const auto & it : internal_) {
    Track t = it.track;
    t.tracking_age = stamp - it.creation_stamp;
    t.time_since_update = stamp - it.last_update_stamp;
    tracks_.push_back(t);
  }
}

}  // namespace social_nav_human_model
