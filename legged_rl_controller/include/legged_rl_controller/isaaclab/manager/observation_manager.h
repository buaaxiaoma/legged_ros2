/**
 * @file observation_manager.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief Manager for observation terms in a reinforcement learning environment.
 * @ref This file is adapted from unitree_rl_lab.
 * @version 0.1
 * @date 2026-01-17
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <map>
#include <string>
#include <unordered_map>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "legged_rl_controller/isaaclab/manager/manager_term_cfg.h"

namespace isaaclab {

using ObsMap = std::map<std::string, ObsFunc>;

inline ObsMap &observations_map()
{
  static ObsMap instance;
  return instance;
}

#define REGISTER_OBSERVATION(name) \
  inline std::vector<float> name(ManagerBasedRLEnv *env, YAML::Node params); \
  inline struct name##_registrar { \
    name##_registrar() { observations_map()[#name] = name; } \
  } name##_registrar_instance; \
  inline std::vector<float> name(ManagerBasedRLEnv *env, YAML::Node params)

class ObservationManager {
public:
  ObservationManager(YAML::Node cfg, ManagerBasedRLEnv *env)
  : cfg_(cfg), env_(env)
  {
    _prepare_terms();
  }

  void reset()
  {
    for (auto &group : group_obs_term_cfgs_) {
      for (auto &term : group.second) {
        term.reset(term.func(env_, term.params));
      }
    }
  }

  std::unordered_map<std::string, std::vector<float>> compute()
  {
    std::unordered_map<std::string, std::vector<float>> obs_map;
    for (const auto &group : group_obs_term_cfgs_) {
      obs_map[group.first] = compute_group(group.first);
    }
    return obs_map;
  }

  std::vector<float> compute_group(const std::string &group_name)
  {
    std::vector<float> obs;
    auto &group_terms = group_obs_term_cfgs_.at(group_name);

    for (auto &term : group_terms) {
      term.add(term.func(env_, term.params));
    }

    for (const auto &term : group_terms) {
      auto obs_ = term.get();
      obs.insert(obs.end(), obs_.begin(), obs_.end());
    }
    return obs;
  }

protected:
  void _prepare_terms()
  {
    YAML::Node observations_node = cfg_["observations"];
    if (!observations_node.IsDefined()) {
      throw std::runtime_error("Observation config missing 'observations' root node.");
    }
    if (observations_node.IsMap() && observations_node.size() == 1U) {
      // Map with a single group is treated as the default "obs" input to match
      // IsaacLab's ONNX export convention and avoid requiring a custom group name.
      auto group = observations_node.begin();
      group_obs_term_cfgs_["obs"] = _prepare_group_terms(group->second);
      return;
    }
    for (auto group = observations_node.begin(); group != observations_node.end(); ++group) {
      const auto group_name = group->first.as<std::string>();
      group_obs_term_cfgs_[group_name] = _prepare_group_terms(group->second);
    }
  }

  std::vector<ObservationTermCfg> _prepare_group_terms(const YAML::Node &group_cfg)
  {
    std::vector<ObservationTermCfg> terms;
    if (!group_cfg.IsSequence()) {
      throw std::runtime_error("Observation group must be a sequence of terms.");
    }
    for (auto it = group_cfg.begin(); it != group_cfg.end(); ++it) {
      const auto term_yaml_cfg = *it;
      const std::string term_name = term_yaml_cfg["name"].as<std::string>();
      terms.push_back(_build_term_cfg(term_name, term_yaml_cfg));
    }
    return terms;
  }

  ObservationTermCfg _build_term_cfg(const std::string &term_name, const YAML::Node &term_yaml_cfg)
  {
    if (observations_map()[term_name] == nullptr) {
      throw std::runtime_error("Observation term '" + term_name + "' is not registered.");
    }

    ObservationTermCfg term_cfg;
    term_cfg.params = term_yaml_cfg;

    const auto overloads = term_yaml_cfg["overloads"];
    if (!overloads.IsDefined()) {
      throw std::runtime_error("Observation term '" + term_name + "' missing 'overloads'.");
    }
    if (!overloads["flatten_history_dim"].IsDefined() ||
      !overloads["flatten_history_dim"].as<bool>()) {
      throw std::runtime_error(
        "Observation term '" + term_name + "' requires 'flatten_history_dim: true'.");
    }

    int history_length = overloads["history_length"].as<int>(1);
    // In IsaacLab, if history_length is not explicitly set, it defaults to 0, 
    // it is equivalent to 1 if flatten_history_dim is true.
    // So we set it to 1 here.
    if (history_length <= 0) {
      history_length = 1;
    }
    term_cfg.history_length = history_length;
    if (!overloads["scale"].IsNull()) {
      if (overloads["scale"].IsSequence()) {
        term_cfg.scale = overloads["scale"].as<std::vector<float>>();
      } else {
        term_cfg.scale = {overloads["scale"].as<float>()};
      }
    }
    if (!overloads["clip"].IsNull()) {
      term_cfg.clip = overloads["clip"].as<std::vector<float>>();
    }

    term_cfg.func = observations_map()[term_name];

    auto obs = term_cfg.func(env_, term_cfg.params);
    term_cfg.reset(obs);

    return term_cfg;
  }

  const YAML::Node cfg_;
  ManagerBasedRLEnv *env_;

private:
  std::unordered_map<std::string, std::vector<ObservationTermCfg>>
    group_obs_term_cfgs_;
};

}  // namespace isaaclab
