// Authors: jiananz1@uci.edu

#pragma once

#include "common_include.h"

namespace cl {
namespace coop {
    
enum MeasurementType {
    kRange,
    kPose,
};

template<int E>
class RelativeMeasurement {
public:
    typedef Eigen::Matrix<double, E, 1, E == 1 ? Eigen::RowMajor : Eigen::ColMajor> Measurement;
    typedef std::shared_ptr<RelativeMeasurement<E>> Ptr;

    RelativeMeasurement(const Measurement& m, const MeasurementType& t) :
        measurement_(m), type_(t) {}

    RelativeMeasurement() = default;
    ~RelativeMeasurement() = default;

    // Mutator
    inline void SetMeasurement(const Measurement & meas) { measurement = meas; }

    // Accessor
    inline Measurement measurement() const { return measurement_; }

private:
    Measurement measurement_;
    MeasurementType type_;
};

} // namespace coop
} // namespace cl