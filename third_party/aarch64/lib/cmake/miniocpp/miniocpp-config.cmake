
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was miniocpp-config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

####################################################################################

# The exported miniocpp::miniocpp_inih target must be visible to the
# dependency resolver, so it can be reused instead of re-fetched.
include("${CMAKE_CURRENT_LIST_DIR}/miniocpp-targets.cmake")

include("${CMAKE_CURRENT_LIST_DIR}/miniocpp-deps.cmake")

# The link interface is not exported; re-apply it.  set_property is used
# instead of target_link_libraries so the imported target stays compatible
# with the CMake 3.13.4 minimum.
set_property(TARGET miniocpp::miniocpp APPEND PROPERTY
             INTERFACE_LINK_LIBRARIES ${MINIO_CPP_DEPS_LINK_LIBS})
