set(CMAKE_C_STANDARD   17)
set(CMAKE_CXX_STANDARD 20)

project(rfm69hcw)
add_library(rfm69hcw ${CMAKE_CURRENT_LIST_DIR}/rfm69hcw.cpp)
target_link_libraries(rfm69hcw hardware_spi hardware_gpio hardware_timer)
target_include_directories(rfm69hcw PUBLIC ${CMAKE_CURRENT_LIST_DIR})
