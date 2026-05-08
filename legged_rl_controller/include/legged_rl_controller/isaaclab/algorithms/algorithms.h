/**
 * @file algorithms.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief Runner for ONNX models using ONNX Runtime C++ API
 * @ref This file is adapted from unitree_rl_lab.
 * @version 0.1
 * @date 2026-01-16
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include "onnxruntime_cxx_api.h"
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace isaaclab {

class Algorithms {
public:
  virtual std::vector<float> act(
    const std::unordered_map<std::string, std::vector<float>> &obs) = 0;

  std::vector<float> get_action()
  {
    std::lock_guard<std::mutex> lock(act_mtx_);
    return action;
  }

  std::vector<float> action;

protected:
  std::mutex act_mtx_;
};

class OrtRunner : public Algorithms {
public:
  explicit OrtRunner(const std::string &model_path)
  {
    // Init Model
    env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "onnx_model");
    session_options_.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);

    session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), session_options_);

    for (size_t i = 0; i < session_->GetInputCount(); ++i) {
      Ort::TypeInfo input_type = session_->GetInputTypeInfo(i);
      input_shapes_.push_back(input_type.GetTensorTypeAndShapeInfo().GetShape());
      auto input_name = session_->GetInputNameAllocated(i, allocator_);
      input_names_.push_back(input_name.release());
    }

    for (const auto &shape : input_shapes_) {
      size_t size = 1;
      for (const auto &dim : shape) {
        if (dim <= 0) {
          throw std::runtime_error("Only static positive ONNX input shapes are supported.");
        }
        size *= dim;
      }
      input_sizes_.push_back(size);
    }

    // Get output shape
    Ort::TypeInfo output_type = session_->GetOutputTypeInfo(0);
    output_shape_ = output_type.GetTensorTypeAndShapeInfo().GetShape();
    auto output_name = session_->GetOutputNameAllocated(0, allocator_);
    output_names_.push_back(output_name.release());

    if (output_shape_.size() < 2 || output_shape_[1] <= 0) {
      throw std::runtime_error("Only static 2D ONNX output shapes are supported.");
    }
    action.resize(output_shape_[1]);
  }

  std::vector<float> act(
    const std::unordered_map<std::string, std::vector<float>> &obs) override
  {
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

    // make sure all input names are in obs
    for (const auto &name : input_names_) {
      if (obs.find(name) == obs.end()) {
        throw std::runtime_error(
          "Input name " + std::string(name) + " not found in observations.");
      }
    }

    // Create input tensors
    std::vector<Ort::Value> input_tensors;
    for (size_t i = 0; i < input_names_.size(); ++i) {
      const std::string name_str(input_names_[i]);
      auto &input_data = obs.at(name_str);
      if (input_data.size() != input_sizes_[i]) {
        throw std::runtime_error(
          "Input '" + name_str + "' size mismatch: got " +
          std::to_string(input_data.size()) + ", expected " +
          std::to_string(input_sizes_[i]) + ".");
      }
      for (size_t j = 0; j < input_data.size(); ++j) {
        if (!std::isfinite(input_data[j])) {
          throw std::runtime_error(
            "Input '" + name_str + "' contains non-finite value at index " +
            std::to_string(j) + ".");
        }
      }
      auto *input_ptr = const_cast<float *>(input_data.data());
      auto input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, input_ptr, input_sizes_[i], input_shapes_[i].data(),
        input_shapes_[i].size());
      input_tensors.push_back(std::move(input_tensor));
    }

    // Run the model
    auto output_tensor = session_->Run(
      Ort::RunOptions{nullptr}, input_names_.data(), input_tensors.data(),
      input_tensors.size(), output_names_.data(), 1);

    // Copy output data
    auto floatarr = output_tensor.front().GetTensorMutableData<float>();
    std::lock_guard<std::mutex> lock(act_mtx_);
    std::memcpy(action.data(), floatarr, output_shape_[1] * sizeof(float));
    for (size_t i = 0; i < action.size(); ++i) {
      if (!std::isfinite(action[i])) {
        throw std::runtime_error(
          "ONNX output contains non-finite action at index " + std::to_string(i) + ".");
      }
    }
    return action;
  }

private:
  Ort::Env env_;
  Ort::SessionOptions session_options_;
  std::unique_ptr<Ort::Session> session_;
  Ort::AllocatorWithDefaultOptions allocator_;

  std::vector<const char *> input_names_;
  std::vector<const char *> output_names_;

  std::vector<std::vector<int64_t>> input_shapes_;
  std::vector<int64_t> input_sizes_;
  std::vector<int64_t> output_shape_;
};
}  // namespace isaaclab
