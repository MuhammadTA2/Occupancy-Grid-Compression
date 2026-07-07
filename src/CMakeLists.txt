
add_executable(occupancy_grid
  main.cpp
  rle.cpp
)
target_include_directories(occupancy_grid
  PRIVATE
    ../include
)

