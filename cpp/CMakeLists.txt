cmake_minimum_required(VERSION 3.26)
project(PackageTest C)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(stm32mp1 CONFIG REQUIRED)

add_executable(cpp-stm32mp1 main.c stm32mp1xx_it.c)
target_link_libraries(cpp-stm32mp1
  stm32mp1::disco_bsp
  stm32mp1::c_polyfill)
