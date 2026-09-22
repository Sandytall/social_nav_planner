// Basic multi-human tracker.
//
// Nearest-neighbour association, track creation/timeout, EMA velocity & acceleration
// estimation, stationary classification. Deliberately simple and deterministic so
// it is suitable for simulation and development; a Kalman variant is an optional upgrade.
//
// Hard rule: a single missed detection must NEVER delete a track. Tracks
// are only removed after `track_timeout` seconds without a matching detection.
#ifndef SOCIAL_NAV_HUMAN_MODEL__TRACKER_HPP_
#define SOCIAL_NAV_HUMAN_MODEL__TRACKER_HPP_

#include <cstdint>
#include <vector>

#include <Eigen/Core>

namespace social_nav_human_model
{

/// A raw detection fed to the tracker (detector-agnostic).
struct Detection
{
  Eigen::Vector2d position{0.0, 0.0};
  double confidence{1.0};
};

/// A maintained track.
struct Track
{
  std::uint64_t id{0};
  Eigen::Vector2d position{0.0, 0.0};
  Eigen::Vector2d velocity{0.0, 0.0};
  Eigen::Vector2d acceleration{0.0, 0.0};
  double confidence{1.0};
  double tracking_age{0.0};        ///< seconds since the track was created
  double time_since_update{0.0};   ///< seconds since the last matching detection
  bool stationary{false};
  bool has_velocity{false};        ///< false until a second observation exists
};

/// Configurable tracker knobs. None are hard-coded behavioural thresholds.
struct TrackerParams
{
  double track_timeout{1.0};             ///< remove a track after this many seconds unseen
  double velocity_filter_alpha{0.5};     ///< EMA weight on the newest measurement, (0,1]
  double max_association_distance{1.0};  ///< max NN gating distance, metres
  double stationary_speed_threshold{0.1};///< below this speed a track is "stationary"
};

class HumanTracker
{
public:
  explicit HumanTracker(const TrackerParams & params = {});

  /// Ingest a set of detections observed at absolute time `stamp` (seconds).
  /// Returns the currently active tracks (post-update, post-timeout).
  const std::vector<Track> & update(const std::vector<Detection> & detections, double stamp);

  const std::vector<Track> & tracks() const { return tracks_; }
  const TrackerParams & params() const { return params_; }

private:
  struct Internal
  {
    Track track;
    double creation_stamp{0.0};
    double last_update_stamp{0.0};
  };

  TrackerParams params_;
  std::vector<Internal> internal_;
  std::vector<Track> tracks_;  ///< public snapshot returned by update()
  std::uint64_t next_id_{0};
  bool initialised_{false};

  void refreshSnapshot(double stamp);
};

}  // namespace social_nav_human_model

#endif  // SOCIAL_NAV_HUMAN_MODEL__TRACKER_HPP_
