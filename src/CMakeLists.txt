
add_executable(konane
  main.cpp
)
    
target_link_libraries(konane
  INTERFACE 
    cisc187::cisc187_options
    cisc187::cisc187_warnings
)


