# Third-party and host dependencies. Fetched sources land in the build tree.
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(Threads REQUIRED)

if(PERSIM_BUILD_VIEWER)
    find_package(raylib 5.5 QUIET)
    if(NOT raylib_FOUND)
        include(FetchContent)
        set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
        set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
        FetchContent_Declare(raylib
            GIT_REPOSITORY https://github.com/raysan5/raylib.git
            GIT_TAG c1ab645ca298a2801097931d1079b10ff7eb9df8
            GIT_SHALLOW TRUE)
        FetchContent_MakeAvailable(raylib)
    endif()
endif()

find_package(Doxygen QUIET)
