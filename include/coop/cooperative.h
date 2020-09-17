#ifndef COOPERATIVE_COOPERATIVE_H
#define COOPERATIVE_COOPERATIVE_H

/*
 * Define the class for cooperative localization
 * Cooperative is define as an ABC with interface implementation,
 * an implementation of DMV method has realized
 * TODO: realize PECMV method
 */
#include "measurement_model.h"

namespace cl {
namespace coop {
    template <int D, int E>
    class Cooperative {
    private:
        ImuPlusRange::Ptr _state_update;
        StateVector<D> _state;
        RelMeasurement<



    public:
        static int DIMSTATE = D;
        s
        virtual Set
        virtual Update



    };
}
}
#endif //COOPERATIVE_COOPERATIVE_H
