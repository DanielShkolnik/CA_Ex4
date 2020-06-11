cmake_minimum_required(VERSION 3.13.2)
project(CA_Ex4)

set(CMAKE_CXX_STANDARD 11)

add_executable(CA_Ex4 core_api.cpp core_api.h main.c sim_api.c sim_api.h)