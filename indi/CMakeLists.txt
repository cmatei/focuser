cmake_minimum_required(VERSION 2.4.7)
PROJECT(indi_focuser CXX)

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
set(indifocuser_SRCS
  ${CMAKE_SOURCE_DIR}/tiny_focuser.cpp
  )

add_executable(indi_focuser ${indifocuser_SRCS})

target_link_libraries(indi_focuser
  ${INDI_DRIVER_LIBRARIES} ${INDI_MAIN_LIBRARIES} ${INDI_LIBRARIES}
  ${LIBUSB10_LIBRARIES} )

install(TARGETS indi_focuser RUNTIME DESTINATION bin )
install(FILES indi_tinyfocuser_sk.xml DESTINATION ${INDI_DATA_DIR})
