cmake_minimum_required(VERSION 3.5)

add_compile_options(-std=c++17)

# Setting default visibility to hidden
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)
# Helps in reducing the possibility of symbol clashes and also minimizes
# the chance of accidentally exporting symbols that aren't intended to be
# part of your library's public API

include_directories(${CMAKE_CURRENT_SOURCE_DIR} include/ src/)

if(PYTHON_EXECUTABLE AND NOT Python3_EXECUTABLE)
    set(Python3_EXECUTABLE "${PYTHON_EXECUTABLE}" CACHE FILEPATH "Python3 executable" FORCE)
endif()

find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 REQUIRED)

# find_package(xbot2_mujoco REQUIRED)

message(STATUS "Compiling python bindings for xbot2_mujoco")

# to get PYTHON_SITE
execute_process(
      COMMAND "${Python3_EXECUTABLE}" -c "if True:
        import sysconfig
        print(sysconfig.get_path('platlib', vars={'base': '', 'platbase': ''}).lstrip('/'))"
      OUTPUT_VARIABLE PYTHON_SITE
      OUTPUT_STRIP_TRAILING_WHITESPACE)

pybind11_add_module("PyXbotMjSim" PyXbotMjSim.cpp)
target_link_libraries("PyXbotMjSim" PUBLIC xbot_mujoco_env pybind11::module)
# install python bindings
install(TARGETS "PyXbotMjSim"
    DESTINATION ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE}/${PROJECT_NAME})
install(FILES __init__.py DESTINATION ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE}/${PROJECT_NAME})
