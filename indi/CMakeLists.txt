cmake_minimum_required(VERSION 2.4.7)
PROJECT(indi_tinyfocus CXX)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules/")
set(BIN_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/bin")

include (CheckCSourceCompiles)
include (MacroOptionalFindPackage)

find_package(INDI REQUIRED)
find_package(USB10 REQUIRED)

include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_SOURCE_DIR})
include_directories( ${INDI_INCLUDE_DIR})

########### QHY ###########
set(indi_tinyfocus_SRCS
  ${CMAKE_SOURCE_DIR}/tiny_focuser.cpp
  )

add_executable(indi_tinyfocus ${indi_tinyfocus_SRCS})

target_link_libraries(indi_tinyfocus
  ${INDI_DRIVER_LIBRARIES} ${INDI_MAIN_LIBRARIES} ${INDI_LIBRARIES}
  ${LIBUSB10_LIBRARIES} )

install(TARGETS indi_tinyfocus RUNTIME DESTINATION bin )
