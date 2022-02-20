#
# CMake module to C++ static analysis against
# Google C++ Style Guide (https://google.github.io/styleguide/cppguide.html)
#
# For more detials please follow links:
#
# - https://github.com/google/styleguide
# - https://pypi.python.org/pypi/cpplint
# - https://github.com/theandrewdavis/cpplint
#
# Copyright (c) 2016 Piotr L. Figlarek
# Copyright on modifications © 2018 Till Junge <till.junge@epfl.ch>
#
# Usage
# -----
# Include this module via CMake include(...) command and then add each source directory
# via introduced by this module cpplint_add_subdirectory(...) function. Added directory
# will be recursivelly scanned and all available files will be checked.
#
# Example
# -------
# # include CMake module
# include(cmake/cpplint.cmake)
#
# # add all source code directories
# cpplint_add_subdirectory(core)
# cpplint_add_subdirectory(modules/c-bind)
#
# License (MIT)
# -------------
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


# select files extensions to check

# target to run cpplint.py for all configured sources
set(CPPLINT_TARGET lint CACHE STRING "Name of C++ style checker target")

# project root directory
set(CPPLINT_PROJECT_ROOT ${PROJECT_SOURCE_DIR} CACHE STRING "Project ROOT directory")

# find cpplint.py script
find_file(CPPLINT name "cpplint.py" PATHS ${PROJECT_SOURCE_DIR}/external)
if(CPPLINT)
    message(STATUS "cpplint parser: ${CPPLINT}")
else()
    message(FATAL_ERROR "cpplint script: NOT FOUND! "
                        "Please install cpplint as described on https://pypi.python.org/pypi/cpplint. "
			"In most cases command 'sudo pip install cpplint' should be sufficent.")
endif()


# common target to concatenate all cpplint.py targets
add_custom_target(${CPPLINT_TARGET})


# use cpplint.py to check source code files inside DIR directory
function(cpplint_add_subdirectory DIR)

    if (NOT PYTHONINTERP_FOUND)
        return()
    endif ()

    # create relative path to the directory
    set(ABSOLUTE_DIR ${DIR})

    # perform cpplint check
    execute_process(
      COMMAND ${PYTHON_EXECUTABLE} ${CPPLINT}
      "--recursive"
      ${ABSOLUTE_DIR}
      COMMENT "cpplint: Checking source code style"
    )

    message(STATUS "Running cpplint done.")
endfunction()
