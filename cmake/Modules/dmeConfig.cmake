INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_DME dme)

FIND_PATH(
    DME_INCLUDE_DIRS
    NAMES dme/api.h
    HINTS $ENV{DME_DIR}/include
        ${PC_DME_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    DME_LIBRARIES
    NAMES gnuradio-dme
    HINTS $ENV{DME_DIR}/lib
        ${PC_DME_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(DME DEFAULT_MSG DME_LIBRARIES DME_INCLUDE_DIRS)
MARK_AS_ADVANCED(DME_LIBRARIES DME_INCLUDE_DIRS)

