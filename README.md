# Rigibra - RIGId body algeBRA package


## Build Process

Use the CMake system to create a build generation environment. E.g. in a
\*nix environment, build and test commands are similar to,

	$ # -- Compile everything (including doxygen documentation)
	$ mkdir <someBuildDir> && cd <someBuildDir>
	$ cmake  \
		-DCMAKE_BUILD_TYPE=Release \
		-DCMAKE_INSTALL_PREFIX=/tmpLocal/ \
		-DCMAKE_PREFIX_PATH=/tmpLocal/ \
		/repos/Rigibra
	$ cmake --build . --target all -j `nproc`
	$ # -- Run library unit tests
	$ ctest -j `nproc`
	$ # -- Create distribution packages
	$ cpack
	$ # -- Installation
	$ dpkg --contents ./Engabra*
	$ sudo apt-get install ./Engabra-0.1.0-Linux.deb   # Install
	$ sudo apt-get remove engabra   # Remove


