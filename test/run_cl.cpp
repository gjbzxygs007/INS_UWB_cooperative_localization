//
// Created by ubuntu-jianan
//

#include "coop/measurement_model.h"
#include "config.h"

int main (int argc, char ** argv) {
    if ( argc != 2 ) {
        cout<<"Usage: run_cl parameter_file"<<endl;
        return 1;
    }
    cl::Config::SetParameterFile(argv[1]);
    cl::coop::ImuPlusRange tt;
    std::cout << tt.getMeasurementCov() << std::endl;

    std::cout << "test" << std::endl;
    return 0;
}

