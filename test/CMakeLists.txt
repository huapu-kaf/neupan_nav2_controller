cmake_minimum_required(VERSION 3.8)

# Find Python
find_package(PkgConfig REQUIRED)
pkg_check_modules(PYTHON REQUIRED python3-embed)

# Create test executable
add_executable(test_python_init test_python_init.cpp)

# Link Python libraries
target_include_directories(test_python_init PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(test_python_init ${PYTHON_LIBRARIES} ${CMAKE_DL_LIBS})
target_compile_options(test_python_init PRIVATE ${PYTHON_CFLAGS_OTHER})