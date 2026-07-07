add_executable(occupancy_grid_tests
  test_main.cpp
)
target_link_libraries(occupancy_grid_tests
  PRIVATE
    occupancy_grid_lib
)

add_test(NAME occupancy_grid_tests COMMAND occupancy_grid_tests)
