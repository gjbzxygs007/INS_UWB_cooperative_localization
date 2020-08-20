

#ifndef COOPERATIVE_MEASUREMENT_H
#define COOPERATIVE_MEASUREMENT_H

#include <iostream>
#include <common_include.h>


namespace cl {
    namespace coop {
        enum MeasType {
            range,
            pose,
        };

        template<int E>
        class RelMeasurement {
        public:
            static const int dim_mea = E;
            typedef Eigen::Matrix<double, E, 1, E == 1 ? Eigen::RowMajor : Eigen::ColMajor> Measurement;
            typedef Eigen::Matrix<double, E, E, E == 1 ? Eigen::RowMajor : Eigen::ColMajor> MeasCov;
            typedef std::shared_ptr<RelMeasurement<E>> Ptr;

            RelMeasurement(Measurement & m, MeasCov & cov, MeasType & t) : _measurement(m), _meas_cov(cov), _type(t) {}
            RelMeasurement() = default;
            ~RelMeasurement() = default;

            // Mutator
            inline void setMeasurement(const Measurement & mea) { _measurement = mea; }
            inline void setMeasCov(const MeasCov & m) {_meas_cov = m; }

            // Accessor
            inline Measurement getMeasurement() const { return _measurement; }
            inline MeasCov getMeasCov() const {return _meas_cov; }

        private:
            Measurement _measurement;
            MeasCov _meas_cov;
            MeasType _type;
        };
    }
}


#endif //COOPERATIVE_MEASUREMENT_H
