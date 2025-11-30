
# Install Find* cmake files to the DATADIR
# This is usually /usr/local/share/opencog/cmake
INSTALL(FILES
	AddCxxtest.cmake
	FindAsio.cmake
	FindBFD.cmake
	Findcpprest.cmake
	FindCxxtest.cmake
	FindCython.cmake
	FindFolly.cmake
	FindGHC.cmake
	FindGTK3.cmake
	FindGuile.cmake
	FindHyperTable.cmake
	FindIberty.cmake
	FindLibGearman.cmake
	FindLinkGrammar.cmake
	FindMOSES.cmake
	FindMsgPack.cmake
	FindOCaml.cmake
	FindOctomap.cmake
	FindPGSQL.cmake
	FindPthreads.cmake
	FindRocksDB.cmake
	FindSIGAR.cmake
	FindSparseHash.cmake
	FindStack.cmake
	FindSTLPort.cmake
	FindTBB.cmake
	FindUUID.cmake
	FindVALGRIND.cmake
	FindZMQ.cmake
	OCamlDep.cmake
	OpenCogFindPython.cmake
	OpenCogFindGuile.cmake
	OpenCogGccOptions.cmake
	OpenCogInstallOptions.cmake
	OpenCogLibOptions.cmake
	Summary.cmake
	UseOCaml.cmake
	COPYING-CMAKE-SCRIPTS
	DESTINATION
	${DATADIR}/cmake/)

# Use GNUInstallDirs for proper multiarch support
include(GNUInstallDirs)

# Set installation directories, respecting CMAKE_INSTALL_LIBDIR if provided
set(INCLUDE_INSTALL_DIR include/ )
# Use CMAKE_INSTALL_LIBDIR if provided (for Debian multiarch), otherwise default to lib/
if(CMAKE_INSTALL_LIBDIR)
	set(LIB_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/ )
else()
	set(LIB_INSTALL_DIR lib/ )
endif()

# CMake config files should be in lib/cmake/CogUtil or lib/<multiarch>/cmake/CogUtil
if(CMAKE_INSTALL_LIBDIR)
	set(COGUTIL_CMAKE_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/CogUtil)
else()
	set(COGUTIL_CMAKE_DIR lib/cmake/CogUtil)
endif()

include(CMakePackageConfigHelpers)
configure_package_config_file(CogUtilConfig.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/CogUtilConfig.cmake
	INSTALL_DESTINATION COGUTIL_CMAKE_DIR
	PATH_VARS INCLUDE_INSTALL_DIR LIB_INSTALL_DIR)

write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/CogUtilConfigVersion.cmake"
	VERSION ${SEMANTIC_VERSION}
	COMPATIBILITY SameMajorVersion
)

INSTALL(FILES
	${CMAKE_CURRENT_BINARY_DIR}/CogUtilConfigVersion.cmake
	${CMAKE_CURRENT_BINARY_DIR}/CogUtilConfig.cmake
	DESTINATION
	${COGUTIL_CMAKE_DIR})
