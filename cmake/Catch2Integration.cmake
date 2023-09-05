Include(FetchContent)
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.4.0
)
FetchContent_MakeAvailable(Catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(CTest)
include(Catch)

add_custom_target(all_tests)

function(add_catch2_test_executable name)
    set(multiValueArgs SOURCES DEPENDS)
    cmake_parse_arguments(ARG "${options}" "${oneValueArgs}"
                          "${multiValueArgs}" ${ARGN} )
    if (DEFINED ARG_UNPARSED_ARGUMENTS)
      message(SEND_ERROR "UNPARSED_ARGUMENTS: ${ARG_UNPARSED_ARGUMENTS}")
    endif()

    add_executable(${name} ${ARG_SOURCES})
    target_link_libraries(${name}
      PRIVATE Catch2::Catch2WithMain ${ARG_DEPENDS}
    )
    add_dependencies(all_tests ${name})
    catch_discover_tests(${name})
endfunction()
