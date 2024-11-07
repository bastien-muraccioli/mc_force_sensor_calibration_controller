#pragma once
#include <mc_rbdyn/Robot.h>
#include <ostream>

#include "Measurement.h"

struct InitialGuess
{
  double mass = 0;
  std::array<double, 3> rpy = {0};
  std::array<double, 3> com = {0};
  std::array<double, 6> offset = {0};
 // Conversion to string using fmt::format
  std::string to_string() const
  {
    return fmt::format(
        R"(mass        : {}
rpy         : {}, {}, {}
com         : {}, {}, {}
force offset: {}, {}, {}, {}, {}, {})",
        mass,
        rpy[0], rpy[1], rpy[2],
        com[0], com[1], com[2],
        offset[0], offset[1], offset[2], offset[3], offset[4], offset[5]);
  }
};

// Overload operator<< to use to_string() for ostream support
inline std::ostream &operator<<(std::ostream &os, const InitialGuess &r)
{
  return os << r.to_string();
}

namespace mc_rtc
{
template<>
struct ConfigurationLoader<InitialGuess>
{
  static InitialGuess load(const mc_rtc::Configuration & config)
  {
    InitialGuess r;
    config("mass", r.mass);
    config("com", r.com);
    config("rpy", r.rpy);
    config("offset", r.offset);
    return r;
  }

  static mc_rtc::Configuration save(const InitialGuess & r)
  {
    mc_rtc::Configuration c;
    c.add("mass", r.mass);
    c.add("com", r.com);
    c.add("rpy", r.rpy);
    c.add("offset", r.offset);
    return c;
  }
};
} // namespace mc_rtc

struct CalibrationResult
{
  CalibrationResult(const InitialGuess & guess)
  {
    mass = guess.mass;
    rpy = guess.rpy;
    com = guess.com;
    offset = guess.offset;
  }

  bool success = false;
  double mass = 0;
  std::array<double, 3> rpy = {0};
  std::array<double, 3> com = {0};
  std::array<double, 6> offset = {0};
};

InitialGuess computeInitialGuessFromModel(const mc_rbdyn::Robot & robot,
                                          const std::string & sensor,
                                          bool includeParent = false,
                                          bool verbose = false);

CalibrationResult calibrate(const mc_rbdyn::Robot & robot,
                            const std::string & sensor,
                            const Measurements & measurements,
                            const InitialGuess & initialGuess,
                            bool verbose = false);
