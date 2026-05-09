/**
 * @file manager_term_cfg.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief
 * @version 0.1
 * @date 2026-01-16
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <algorithm>
#include <deque>
#include <functional>
#include <numeric>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace isaaclab {

class ManagerBasedRLEnv;

using ObsFunc = std::function<std::vector<float>(ManagerBasedRLEnv *, YAML::Node)>;

/**
 * @brief Configuration and buffer for an observation term.
 * 
 */
struct ObservationTermCfg {
  YAML::Node params;
  ObsFunc func;
  std::vector<float> clip;  // [min, max]
  std::vector<float> scale; // per-element scaling factors
  int history_length = 1;
  bool scale_first = false; // inherit from unitree_rl_lab, only support false here

  /**
   * @brief Reset the observation buffer with the given observation.
   * 
   * @param obs The observation to reset the buffer with.
   */
  void reset(const std::vector<float> &obs)
  {
    for (int i = 0; i < history_length; ++i) {
      add(obs);
    }
  }

  /**
   * @brief Add a new observation to the buffer, applying scaling and clipping as configured.
   * 
   * @param obs_in The new observation to add.
   */
  void add(const std::vector<float> &obs_in)
  {
    std::vector<float> obs = obs_in;
    for (size_t j = 0; j < obs.size(); ++j) {
      // Apply scaling and clipping based on the configuration.
      if (scale_first) {
        if (!scale.empty()) {
          obs[j] *= scale.size() == 1U ? scale[0] : scale[j];
        }
        if (!clip.empty()) {
          obs[j] = std::clamp(obs[j], clip[0], clip[1]);
        }
      } else {
        if (!clip.empty()) {
          obs[j] = std::clamp(obs[j], clip[0], clip[1]);
        }
        if (!scale.empty()) {
          obs[j] *= scale.size() == 1U ? scale[0] : scale[j];
        }
      }
    }
    buff_.push_back(obs);

    if (buff_.size() > static_cast<size_t>(history_length)) {
      buff_.pop_front();
    }
  }

  /**
   * @brief Get the observation at the specified index in the buffer.
   * 
   * @param n The index of the observation to retrieve.
   * @return const std::vector<float>& The observation at the specified index.
   */
  const std::vector<float> &get(int n) const { return buff_[n]; }

  /**
   * @brief Get the concatenated observations from the buffer.
   * 
   * @return std::vector<float> The concatenated observations.
   */
  std::vector<float> get() const
  {
    std::vector<float> concatenated;
    for (const auto &entry : buff_) {
      concatenated.insert(concatenated.end(), entry.begin(), entry.end());
    }
    return concatenated;
  }

  /**
   * @brief Get the total size of all observations in the buffer.
   * 
   * @return size_t The total size of all observations.
   */
  size_t size() const
  {
    return std::accumulate(
      buff_.begin(), buff_.end(), static_cast<size_t>(0),
      [](size_t sum, const auto &v) { return sum + v.size(); });
  }

private:
  // Complete circular buffer with most recent entry at the end and oldest entry at the beginning.
  std::deque<std::vector<float>> buff_;
};

}  // namespace isaaclab
