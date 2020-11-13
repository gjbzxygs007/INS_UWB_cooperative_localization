add_library(looselycl SHARED
            measurement_model.cpp
            inertial_nav.cpp
            config.cpp)

target_link_libraries(looselycl ${THIRD_PARTY_LIBS})