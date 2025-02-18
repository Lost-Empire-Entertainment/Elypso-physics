cmake_minimum_required(VERSION 3.29.5)

# Ensure install path is set correctly
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/out/install")
endif()


if (WIN32)
    # Enable hot reload for MSVC
    if (POLICY CMP0141)
        cmake_policy(SET CMP0141 NEW)
        set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
    endif()
    # Explicitly set the MSVC toolset for Visual Studio
    if (MSVC)
        set(CMAKE_GENERATOR_TOOLSET "v143")
    endif()
endif()

# Specify the compiler for Linux builds
if (UNIX)
    set(CMAKE_C_COMPILER "/usr/bin/gcc")
    set(CMAKE_CXX_COMPILER "/usr/bin/g++")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set runtime library type for MSVC
if (MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

# Paths to frequently used directories
get_filename_component(PARENT_DIR "${CMAKE_SOURCE_DIR}" DIRECTORY)
set(PARENT_DIR_PATH "${PARENT_DIR}")

# Physics files
set(PHYSICS_SRC_DIR "${CMAKE_SOURCE_DIR}/src")
set(PHYSICS_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/include")

# External files
set(EXT_GLM_DIR "${CMAKE_SOURCE_DIR}/../_external_shared/GLM")

# Add GLM
add_library(glm INTERFACE)
target_include_directories(glm INTERFACE ${EXT_GLM_DIR})

# Add source files
file(GLOB_RECURSE PHYSICS_SOURCE_FILES CONFIGURE_DEPENDS
    ${PHYSICS_SRC_DIR}/*.cpp
    ${PHYSICS_SRC_DIR}/*/*.cpp
)

# Store include directories
set(PHYSICS_INCLUDE_DIRS
    ${PHYSICS_INCLUDE_DIR}
    ${EXT_GLM_DIR}
)

# Build as a shared library (DLL)
add_library(ElypsoPhysics SHARED ${PHYSICS_SOURCE_FILES})
set_target_properties(ElypsoPhysics PROPERTIES OUTPUT_NAME "ElypsoPhysics")
target_compile_features(ElypsoPhysics PRIVATE cxx_std_20)
target_include_directories(ElypsoPhysics PRIVATE ${PHYSICS_INCLUDE_DIRS})

# Append 'D' suffix to debug builds for both Windows and Linux
set_target_properties(ElypsoPhysics PROPERTIES
    OUTPUT_NAME "ElypsoPhysics$<$<CONFIG:Debug>:D>"
)

# Also build a static library
add_library(ElypsoPhysics_Static STATIC ${PHYSICS_SOURCE_FILES})
set_target_properties(ElypsoPhysics_Static PROPERTIES OUTPUT_NAME "ElypsoPhysics_Static")
target_compile_features(ElypsoPhysics_Static PRIVATE cxx_std_20)
target_include_directories(ElypsoPhysics_Static PRIVATE ${PHYSICS_INCLUDE_DIRS})

# Append 'D' suffix to debug builds for both Windows and Linux
set_target_properties(ElypsoPhysics_Static PROPERTIES
    OUTPUT_NAME "ElypsoPhysics_Static$<$<CONFIG:Debug>:D>"
)

# Apply all directories to target
target_include_directories(ElypsoPhysics PRIVATE ${PHYSICS_INCLUDE_DIRS})

if (WIN32)
    target_link_libraries(ElypsoPhysics PRIVATE glm)
else()
    find_package(X11 QUIET)
    if (X11_FOUND)
        target_link_libraries(ElypsoPhysics PRIVATE glm ${X11_LIBRARIES})
    endif()
endif()

# Define preprocessor directives
target_compile_definitions(ElypsoPhysics PRIVATE PHYSICS_DLL_EXPORT)
target_compile_definitions(ElypsoPhysics_Static PRIVATE PHYSICS_STATIC_LIB)

# Install the DLL and its import library (ElypsoPhysics.dll + ElypsoPhysics.lib)
install(TARGETS ElypsoPhysics
    RUNTIME DESTINATION bin  # .dll (Windows) and .so (Linux)
    LIBRARY DESTINATION lib  # .lib (Windows import lib) or .so (Linux)
    ARCHIVE DESTINATION lib  # .lib (Linux static lib)
)

# Install the static library (ElypsoPhysics_Static.lib)
install(TARGETS ElypsoPhysics_Static
    ARCHIVE DESTINATION lib  # Static library goes to lib/
)

# Install header files
install(DIRECTORY ${PHYSICS_INCLUDE_DIR}/
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp"
)