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

NOTE: The build process uses "git describe" command to get source code
identifier information. If the build process (user) id is different from
the source code repository ownership id, you may recieve an error
message like this:

	### Can't get 'git describe' result
	### ErrorString: fatal: detected dubious ownership in repository at ...

If this happens, the unit test (e.g. run via CTest), "test_Version" will
fail. The git error can be avoided by adding an git exception assigned to
the source code source repository, e.g.,

	# To add an exception for this directory, call:
	$ git config --global --add safe.directory /repos/Rigibra


