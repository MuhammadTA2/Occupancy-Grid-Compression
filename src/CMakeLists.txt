add_library(occupancy_grid_lib STATIC
  grid.cpp
  rle.cpp
  metrics.cpp
  bit_writer.cpp
  varint.cpp
  fixed_width_coder.cpp
  varint_coder.cpp
  rle_codec.cpp
  rice_coder.cpp
  counts_coder.cpp
  packetizer.cpp
  loopback_transport.cpp
  packet_reassembler.cpp
  split_codec.cpp
  lossy_transport.cpp
  arq.cpp
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

add_executable(occupancy_grid_benchmark
  benchmark.cpp
)
target_link_libraries(occupancy_grid_benchmark
  PRIVATE
    occupancy_grid_lib
)
