add_executable(run_cl run_cl.cpp)
target_link_libraries(run_cl looselycl ${PROJECT_SOURCE_DIR}/lib/liblibvncxx.a)