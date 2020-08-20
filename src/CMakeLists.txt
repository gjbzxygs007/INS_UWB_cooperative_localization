add_library(looselycl SHARED
            measurement_model.cpp)

target_link_libraries(looselycl ${THIRD_PARTY_LIBS})