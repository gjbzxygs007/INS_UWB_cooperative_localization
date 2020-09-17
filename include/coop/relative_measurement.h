

#ifndef COOPERATIVE_MEASUREMENT_H
#define COOPERATIVE_MEASUREMENT_H

#include <iostream>
#include <common_include.h>


namespace cl {
    namespace coop {
        enum MeasType {
            RANGE,
            POSE,
        };

        template<int E>
        class RelMeasurement {
        public:
            static const int dim_mea = E;
            typedef Eigen::Matrix<double, E, 1, E == 1 ? Eigen::RowMajor : Eigen::ColMajor> Measurement;
            typedef std::shared_ptr<RelMeasurement<E>> Ptr;

            RelMeasurement(Measurement & m, MeasType & t) : _measurement(m), _type(t) {}
            RelMeasurement() = default;
            ~RelMeasurement() = default;

            // Mutator
            inline void setMeasurement(const Measurement & mea) { _measurement = mea; }

            // Accessor
            inline Measurement getMeasurement() const { return _measurement; }

        private:
            Measurement _measurement;
            MeasType _type;
        };
    }
}


#endif //COOPERATIVE_MEASUREMENT_H
