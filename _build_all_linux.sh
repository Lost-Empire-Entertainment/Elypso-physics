#!/bin/bash
set -e

# Set the origin folder as the location of this script
origin="$(dirname "$(readlink -f "$0")")"

# Set the engine folder (relative to origin)
engine="$origin/../Elypso-engine/Engine"

# Build and install directories (origin folders)
origin_build_release="$origin/build-release"
origin_build_debug="$origin/build-debug"
origin_release_install="$origin/install-release"
origin_debug_install="$origin/install-debug"

echo -e "\n===================================================="
echo "Preinstall Cleanup"
printf "====================================================\n"

if rm -rf "$origin_build_release"; then
    echo "Deleted build release folder."
fi
if rm -rf "$origin_build_debug"; then
    echo "Deleted build debug folder."
fi
if rm -rf "$origin_release_install"; then
    echo "Deleted install release folder."
fi
if rm -rf "$origin_debug_install"; then
    echo "Deleted install debug folder."
fi

echo -e "\n===================================================="
echo "Building Release and Debug Versions"
printf "====================================================\n"

# Run the release build script
if [ -f "$origin/build_linux_release.sh" ]; then
    bash "$origin/build_linux_release.sh"
else
    echo "Error: build_linux_release.sh not found."
    exit 1
fi

# Run the debug build script
if [ -f "$origin/build_linux_debug.sh" ]; then
    bash "$origin/build_linux_debug.sh"
else
    echo "Error: build_linux_debug.sh not found."
    exit 1
fi

###############################################################################
# Setup Target Directories for Copying Libraries and Headers
###############################################################################

# Physics library target folder (e.g., used by external projects)
physics_folder="$engine/../_external_shared/elypsophysics"
physics_release="$physics_folder/release"
physics_debug="$physics_folder/debug"

# Origin library paths (shared libraries only)
origin_release_lib="$origin_release_install/lib/libElypsoPhysics.so"
origin_debug_lib="$origin_debug_install/lib/libElypsoPhysicsD.so"

# Engine library target folders
engine_libs_folder="$engine/files/external dlls"
engine_release_lib="$engine_libs_folder/release/libElypsoPhysics.so"
engine_debug_lib="$engine_libs_folder/debug/libElypsoPhysicsD.so"

# Game library target folders
game_libs_folder="$engine/../Game/files/external dlls"
game_release_lib="$game_libs_folder/release/libElypsoPhysics.so"
game_debug_lib="$game_libs_folder/debug/libElypsoPhysicsD.so"

# Header files to be copied
header_files=("collider.hpp" "collisiondetection.hpp" "gameobjecthandle.hpp" "physicsworld.hpp" "rigidbody.hpp")
origin_headers="$origin/include"
header_target_folder="$physics_folder"

echo -e "\n===================================================="
echo "Installing Libraries and Headers"
printf "====================================================\n"

# Create target directories if they do not exist
mkdir -p "$physics_release" "$physics_debug" \
         "$engine_libs_folder/release" "$engine_libs_folder/debug" \
         "$game_libs_folder/release" "$game_libs_folder/debug" \
         "$header_target_folder"

# Copy libraries to physics folder
cp "$origin_release_lib" "$physics_release/" && echo "Copied release library to physics release folder."
cp "$origin_debug_lib" "$physics_debug/" && echo "Copied debug library to physics debug folder."

# Copy libraries to engine folders
cp "$origin_release_lib" "$engine_release_lib" && echo "Copied release library to engine release folder."
cp "$origin_debug_lib" "$engine_debug_lib" && echo "Copied debug library to engine debug folder."

# Copy libraries to game folders
cp "$origin_release_lib" "$game_release_lib" && echo "Copied release library to game release folder."
cp "$origin_debug_lib" "$game_debug_lib" && echo "Copied debug library to game debug folder."

# Copy header files to the physics folder
for header in "${header_files[@]}"; do
    origin_header="$origin_headers/$header"
    if [ -f "$origin_header" ]; then
        cp "$origin_header" "$header_target_folder/" && echo "Copied $header to physics folder."
    else
        echo "Error: Header file $origin_header not found."
        exit 1
    fi
done

echo -e "\n===================================================="
echo "Finished copying files!"
printf "====================================================\n"

exit 0
