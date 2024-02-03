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
## -- CMake source code tree
##


# list of sources on which library depends
set(${aProjName}LibSources

	Rigibra.cpp
	
	)

# create CMake library target
add_library(
	${${aProjName}LibName}
	STATIC
		${${aProjName}LibSources}
	)

target_compile_options(
	${${aProjName}LibName}
	PRIVATE
		$<$<CXX_COMPILER_ID:Clang>:${BUILD_FLAGS_FOR_CLANG}>
		$<$<CXX_COMPILER_ID:GNU>:${BUILD_FLAGS_FOR_GCC}>
		$<$<CXX_COMPILER_ID:MSVC>:${BUILD_FLAGS_FOR_VISUAL}>
	)

target_include_directories(
	${${aProjName}LibName}
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
		$<INSTALL_INTERFACE:include/${aProjName}>
	PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
	)

# install the library
install(
	TARGETS ${${aProjName}LibName}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	)

# Alias for use in downstream CMakeLists
add_library(${aProjName}::${aProjName} ALIAS ${${aProjName}LibName})

# Export info
add_library(
	${EXPORT_ALIAS_NAME}
	ALIAS
	${${aProjName}LibName}
	)

# export linkable library target
install(
	TARGETS ${${aProjName}LibName}
	EXPORT ${EXPORT_TARGETS_NAME}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	#[[
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	INCLUDES DESTINATION
		$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/${${aProjName}_LibDIR}>
		$<INSTALL_INTERFACE:${${aProjName}_LibDIR}>
	NAMESPACE ${aProjName}:: # e.g. ${aProjName}::${aProjName}
	#]]
	)

