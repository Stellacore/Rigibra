#
# MIT License
#
# Copyright (c) 2024 Stellacore Corporation
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

##
## -- CMake build system description
##

cmake_minimum_required(VERSION 3.13)

set(aProjName "Rigibra")
set(aProjVersion "0.1.0") # Set this based on features, updates and patches
set(aProjLib "${aProjName}Lib")
include(./cmake/func_setReleaseTag.cmake)
setReleaseTag(aProjSourceIdentity)

project(
	${aProjName}
	VERSION ${aProjVersion}
	DESCRIPTION "Resolve conventions for sensor-to-vehicle mount configuration"
	LANGUAGES CXX
	)

include(CTest)
enable_testing()


message("### CMAKE_MAJOR_VERSION: " ${CMAKE_MAJOR_VERSION})
message("### CMAKE_MINOR_VERSION: " ${CMAKE_MINOR_VERSION})
message("### CMAKE_PATCH_VERSION: " ${CMAKE_PATCH_VERSION})
message("### CMAKE_TWEAK_VERSION: " ${CMAKE_TWEAK_VERSION})
message("### PROJECT: " ${PROJECT})
message("### PROJECT_VERSION: " ${PROJECT_VERSION})


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ===
# === Environment specific flags
# ===

set(BUILD_FLAGS_FOR_CLANG
	""
	)
set(BUILD_FLAGS_FOR_GCC
	"-DRigibra_Project_Version=\"${aProjVersion}\""
	"-DRigibra_Source_Identity=\"${aProjSourceIdentity}\""
	#
	-pthread
	-fomit-frame-pointer
	-fno-operator-names
	-fstrict-enums
	#
	-Wc++11-compat
	-Wall
	-Wextra
	-Wuninitialized
	-Winit-self
	-fno-nonansi-builtins
	-pedantic-errors
	#
	-fmax-errors=1  # keep template error message generation under control
	)
set(BUILD_FLAGS_FOR_VISUAL
	""
	)


find_package(engabra REQUIRED NO_MODULE)
message(engabra Found: ${engabra_FOUND})
message(engabra Version: ${engabra_VERSION})

# ===
# === Documentation
# ===

add_subdirectory(doc) # project reference documentation

# ===
# === Library
# ===

message("adding src library aProjLib: ${aProjLib}")

# set(EXPORT_ALIAS_NAME ${aProjLib}::${aProjLib})
# set(EXPORT_TARGETS_NAME ${aProjLib}Targets)
# set(EXPORT_FILENAME ${EXPORT_TARGETS_NAME}.cmake)
# message("### EXPORT_ALIAS_NAME: " ${EXPORT_ALIAS_NAME})
# message("### EXPORT_TARGETS_NAME: " ${EXPORT_TARGETS_NAME})
# message("### EXPORT_FILENAME: " ${EXPORT_FILENAME})

add_subdirectory(src)

# ===
# === Demonstrations
# ===

## add_subdirectory(demo)

# ===
# === Test programs
# ===

add_subdirectory(test)

