
add_executable(occupancy_grid
  main.cpp
  rle.cpp
)
target_include_directories(occupancy_grid
       	Private
		../include
)
target_link_libraries(occupancy_grid
  INTERFACE 
    cisc187::cisc187_options
    cisc187::cisc187_warnings
)


