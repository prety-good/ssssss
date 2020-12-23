cmake_minimum_required(VERSION 3.10)
project(lex)
#find_package( OpenCV REQUIRED )
#include_directories( ${OpenCV_INCLUDE_DIRS} )
set(CMAKE_CXX_STANDARD 14)

add_executable(lex lex.yy.c)
target_link_libraries( lex ${OpenCV_LIBS} )
