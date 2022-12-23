cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(ndd)

set(CMAKE_CXX_STANDARD 17)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(ndd cpp/src/NDD.cpp)
target_include_directories(ndd PRIVATE ${PCL_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} cpp/include)
target_link_libraries(ndd ${PCL_LIBRARY_DIRS} Eigen3::Eigen)
set_source_files_properties(cpp/src/NDD.cpp PROPERTIES COMPILE_FLAGS -Wall -Weffc++ -pedantic)

add_executable(ndd_demo demo/NDD_demo.cpp)
target_include_directories(ndd_demo PRIVATE ${PCL_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} cpp/include cpp)
target_link_libraries(ndd_demo ndd ${PCL_LIBRARIES} Eigen3::Eigen)
set_source_files_properties(demo/NDD_demo.cpp PROPERTIES COMPILE_FLAGS -Wall -Weffc++ -pedantic)

add_executable(SAC-IA demo/SAC-IA.cpp)
target_link_libraries(SAC-IA ${PCL_LIBRARIES})
set_source_files_properties(SAC-IA PROPERTIES COMPILE_FLAGS -Wall -Weffc++ -pedantic)

add_executable(SAC-IA+ICP demo/SAC-IA+ICP.cpp)
target_link_libraries(SAC-IA+ICP ${PCL_LIBRARIES})
set_source_files_properties(SAC-IA+ICP PROPERTIES COMPILE_FLAGS -Wall -Weffc++ -pedantic)

add_executable(SAC-IA+NDT demo/SAC-IA+NDT.cpp)
target_link_libraries(SAC-IA+NDT ${PCL_LIBRARIES})
set_source_files_properties(SAC-IA+NDT PROPERTIES COMPILE_FLAGS -Wall -Weffc++ -pedantic)