cmake_minimum_required(VERSION 2.8.3)
project(back_end)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
  uneven_map
  front_end
)

find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES back_end
 CATKIN_DEPENDS roscpp std_msgs uneven_map visualization_msgs front_end
)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
        SYSTEM
)

set(G2O_core_FILE include/g2o/core)
set(G2O_stuff_FILE include/g2o/stuff)
message(STATUS "g2o source core directory: ${G2O_core_FILE}")
message(STATUS "g2o source stuff directory: ${G2O_stuff_FILE}")

add_library(teb_stuff SHARED
        ${G2O_stuff_FILE}/filesys_tools.h    ${G2O_stuff_FILE}/string_tools.h
        ${G2O_stuff_FILE}/color_macros.h     ${G2O_stuff_FILE}/macros.h         ${G2O_stuff_FILE}/timeutil.cpp
        ${G2O_stuff_FILE}/command_args.cpp   ${G2O_stuff_FILE}/misc.h           ${G2O_stuff_FILE}/sparse_helper.cpp  ${G2O_stuff_FILE}/timeutil.h
        ${G2O_stuff_FILE}/command_args.h     ${G2O_stuff_FILE}/os_specific.c    ${G2O_stuff_FILE}/sparse_helper.h
        ${G2O_stuff_FILE}/filesys_tools.cpp  ${G2O_stuff_FILE}/os_specific.h    ${G2O_stuff_FILE}/string_tools.cpp
        ${G2O_stuff_FILE}/property.cpp       ${G2O_stuff_FILE}/property.h
        ${G2O_stuff_FILE}/sampler.cpp        ${G2O_stuff_FILE}/sampler.h        ${G2O_stuff_FILE}/unscented.h
        ${G2O_stuff_FILE}/tictoc.cpp ${G2O_stuff_FILE}/tictoc.h
        ${G2O_stuff_FILE}/g2o_stuff_api.h
)
target_link_libraries(teb_stuff PUBLIC ${catkin_LIBRARIES})

add_library(teb_core SHARED
        ${G2O_core_FILE}/dynamic_aligned_buffer.hpp
        ${G2O_core_FILE}/ownership.h
        ${G2O_core_FILE}/base_edge.h
        ${G2O_core_FILE}/base_binary_edge.h          ${G2O_core_FILE}/hyper_graph_action.cpp
        ${G2O_core_FILE}/base_binary_edge.hpp        ${G2O_core_FILE}/hyper_graph_action.h
        ${G2O_core_FILE}/base_multi_edge.h           ${G2O_core_FILE}/hyper_graph.cpp
        ${G2O_core_FILE}/base_multi_edge.hpp         ${G2O_core_FILE}/hyper_graph.h
        ${G2O_core_FILE}/base_unary_edge.h           ${G2O_core_FILE}/linear_solver.h
        ${G2O_core_FILE}/base_unary_edge.hpp         ${G2O_core_FILE}/marginal_covariance_cholesky.cpp
        ${G2O_core_FILE}/base_vertex.h               ${G2O_core_FILE}/marginal_covariance_cholesky.h
        ${G2O_core_FILE}/base_vertex.hpp             ${G2O_core_FILE}/matrix_structure.cpp
        ${G2O_core_FILE}/batch_stats.cpp             ${G2O_core_FILE}/matrix_structure.h
        ${G2O_core_FILE}/batch_stats.h               ${G2O_core_FILE}/openmp_mutex.h
        ${G2O_core_FILE}/block_solver.h              ${G2O_core_FILE}/block_solver.hpp
        ${G2O_core_FILE}/parameter.cpp               ${G2O_core_FILE}/parameter.h
        ${G2O_core_FILE}/cache.cpp                   ${G2O_core_FILE}/cache.h
        ${G2O_core_FILE}/optimizable_graph.cpp       ${G2O_core_FILE}/optimizable_graph.h
        ${G2O_core_FILE}/solver.cpp                  ${G2O_core_FILE}/solver.h
        ${G2O_core_FILE}/creators.h                  ${G2O_core_FILE}/optimization_algorithm_factory.cpp
        ${G2O_core_FILE}/estimate_propagator.cpp     ${G2O_core_FILE}/optimization_algorithm_factory.h
        ${G2O_core_FILE}/estimate_propagator.h
        ${G2O_core_FILE}/factory.cpp                 ${G2O_core_FILE}/optimization_algorithm_property.h
        ${G2O_core_FILE}/factory.h                   ${G2O_core_FILE}/sparse_block_matrix.h
        ${G2O_core_FILE}/sparse_optimizer.cpp  ${G2O_core_FILE}/sparse_block_matrix.hpp
        ${G2O_core_FILE}/sparse_optimizer.h
        ${G2O_core_FILE}/hyper_dijkstra.cpp ${G2O_core_FILE}/hyper_dijkstra.h
        ${G2O_core_FILE}/parameter_container.cpp     ${G2O_core_FILE}/parameter_container.h
        ${G2O_core_FILE}/optimization_algorithm.cpp ${G2O_core_FILE}/optimization_algorithm.h
        ${G2O_core_FILE}/optimization_algorithm_with_hessian.cpp ${G2O_core_FILE}/optimization_algorithm_with_hessian.h
        ${G2O_core_FILE}/optimization_algorithm_gauss_newton.cpp ${G2O_core_FILE}/optimization_algorithm_gauss_newton.h
        ${G2O_core_FILE}/optimization_algorithm_levenberg.cpp ${G2O_core_FILE}/optimization_algorithm_levenberg.h
        ${G2O_core_FILE}/optimization_algorithm_dogleg.cpp ${G2O_core_FILE}/optimization_algorithm_dogleg.h
        ${G2O_core_FILE}/sparse_optimizer_terminate_action.cpp ${G2O_core_FILE}/sparse_optimizer_terminate_action.h
        ${G2O_core_FILE}/jacobian_workspace.cpp ${G2O_core_FILE}/jacobian_workspace.h
        ${G2O_core_FILE}/robust_kernel.cpp ${G2O_core_FILE}/robust_kernel.h
        ${G2O_core_FILE}/robust_kernel_impl.cpp ${G2O_core_FILE}/robust_kernel_impl.h
        ${G2O_core_FILE}/robust_kernel_factory.cpp ${G2O_core_FILE}/robust_kernel_factory.h
        ${G2O_core_FILE}/g2o_core_api.h
)
target_link_libraries(teb_core PUBLIC teb_stuff ${catkin_LIBRARIES})

add_library(teb_optimizer
        SHARED
            src/path_optimizer/path_optimizer.cpp
            src/path_optimizer/timed_elastic_band.cpp)
target_link_libraries(teb_optimizer PUBLIC
        ${catkin_LIBRARIES} teb_core)

add_library(back_end 
    src/alm_traj_opt.cpp
    src/pso_smoother.cpp
        src/stomp_smoother.cpp
        src/stomp.cpp
        src/stomp_utils.cpp
)

target_link_libraries(back_end
    ${catkin_LIBRARIES}
        teb_optimizer
)

add_executable(back_end_node src/back_end_node.cpp )

target_link_libraries(back_end_node
    back_end
    ${catkin_LIBRARIES} 
)
