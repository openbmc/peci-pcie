cmake_minimum_required (VERSION 3.6)
project (peci-pcie CXX)
include (ExternalProject)
set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable (peci-pcie src/peci_pcie.cpp src/pci_ids.cpp)

option (YOCTO "Enable Building in Yocto" OFF)

if (NOT YOCTO)
    add_dependencies (peci-pcie libpeci)
    externalproject_add (libpeci PREFIX ${CMAKE_BINARY_DIR}/libpeci
                         GIT_REPOSITORY https://github.com/openbmc/libpeci.git
                         GIT_TAG bc641112abc99b4a972665aa984023a6713a21ac
                         INSTALL_COMMAND "")

    externalproject_get_property (libpeci SOURCE_DIR)
    include_directories (${SOURCE_DIR})

    externalproject_get_property (libpeci BINARY_DIR)
    add_library (peci SHARED IMPORTED)
    set_target_properties (peci
                           PROPERTIES IMPORTED_LOCATION
                                      ${BINARY_DIR}/libpeci.so)
endif ()

target_include_directories (peci-pcie PRIVATE ${CMAKE_SOURCE_DIR})

target_link_libraries (peci-pcie peci sdbusplus -lsystemd)

include_directories (${CMAKE_CURRENT_SOURCE_DIR}/include)

install (TARGETS peci-pcie
         RUNTIME DESTINATION bin
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib/static)

find_package (Boost 1.66 REQUIRED)
include_directories (${BOOST_SRC_DIR})

add_definitions (-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions (-DBOOST_SYSTEM_NO_DEPRECATED)
add_definitions (-DBOOST_ALL_NO_LIB)
add_definitions (-DBOOST_NO_RTTI)
add_definitions (-DBOOST_NO_TYPEID)
add_definitions (-DBOOST_ASIO_DISABLE_THREADS)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-rtti")

set (SERVICE_FILES
     ${PROJECT_SOURCE_DIR}/service_files/xyz.openbmc_project.PCIe.service)
install (FILES ${SERVICE_FILES} DESTINATION /lib/systemd/system/)
