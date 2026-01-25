// Copyright 2025 selfpatch
// SPDX-License-Identifier: Apache-2.0

/// @file simulated_sensor_base.hpp
/// @brief Base class for simulated sensors with fault injection
///
/// NOTE: This header provides a reusable base class for simulated sensors.
/// Currently, sensor nodes implement fault injection directly for simplicity.
/// This base class is retained for future refactoring when sensors need to
/// share common fault injection logic.

#pragma once

#include <random>

namespace sensor_diagnostics
{

/// Configuration for fault injection in simulated sensors
struct FaultConfig
{
  double failure_probability{0.0};  ///< Probability of complete failure (0.0-1.0)
  bool inject_nan{false};           ///< Inject NaN values
  double noise_multiplier{1.0};     ///< Multiplier for noise (>1 = degraded)
  double drift_rate{0.0};           ///< Drift rate per second
};

/// Base class for simulated sensors with fault injection
class SimulatedSensorBase
{
public:
  SimulatedSensorBase()
  : rng_(std::random_device{}()), uniform_dist_(0.0, 1.0), normal_dist_(0.0, 1.0)
  {
  }

  virtual ~SimulatedSensorBase() = default;

  /// Check if sensor should fail this cycle
  bool should_fail() const
  {
    return uniform_dist_(rng_) < config_.failure_probability;
  }

  /// Check if NaN values should be injected
  bool should_inject_nan() const { return config_.inject_nan; }

  /// Get noise value scaled by noise_multiplier
  double get_noise(double base_stddev)
  {
    return normal_dist_(rng_) * base_stddev * config_.noise_multiplier;
  }

  /// Get accumulated drift value
  double get_drift(double elapsed_seconds) const
  {
    return config_.drift_rate * elapsed_seconds;
  }

  /// Generate uniform random value in range [min, max]
  double get_uniform(double min, double max)
  {
    return min + uniform_dist_(rng_) * (max - min);
  }

  /// Update fault configuration
  void set_fault_config(const FaultConfig & config) { config_ = config; }

  /// Get current fault configuration
  const FaultConfig & get_fault_config() const { return config_; }

protected:
  FaultConfig config_;
  mutable std::mt19937 rng_;
  mutable std::uniform_real_distribution<double> uniform_dist_;
  mutable std::normal_distribution<double> normal_dist_;
};

}  // namespace sensor_diagnostics
