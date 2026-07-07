add_library(occupancy_grid_lib STATIC
  grid.cpp
  rle.cpp
  metrics.cpp
)
target_include_directories(occupancy_grid_lib
  PUBLIC
    ../include
)

add_executable(occupancy_grid
  main.cpp
)
target_link_libraries(occupancy_grid
  PRIVATE
    occupancy_grid_lib
)
