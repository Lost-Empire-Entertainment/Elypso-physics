#!/bin/bash
# This script builds ElypsoPhysics from source using g++ and CMake with Unix Makefiles in Release mode on Linux.

# Set the root folder as the location of this script
PHYSICS_ROOT="$(dirname "$(readlink -f "$0")")"
BUILD_DIR="$PHYSICS_ROOT/build-release"
INSTALL_DIR="$PHYSICS_ROOT/install-release"

# Record start time
TIME_START=$(date +%T)

# Create the build directory if it doesn't exist
mkdir -p "$BUILD_DIR" || { echo "[ERROR] Failed to create build directory: $BUILD_DIR"; exit 1; }
cd "$BUILD_DIR" || { echo "[ERROR] Failed to access build directory: $BUILD_DIR"; exit 1; }

# Configure ElypsoPhysics with CMake using Unix Makefiles
echo "[INFO] Configuring ElypsoPhysics with CMake..."
cmake -G "Unix Makefiles" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER=gcc \
  -DCMAKE_CXX_COMPILER=g++ \
  -DBUILD_SHARED_LIBS=ON \
  -DCMAKE_C_FLAGS="-O2 -DNDEBUG" \
  -DCMAKE_CXX_FLAGS="-O2 -DNDEBUG" \
  -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" \
  -Wno-dev \
  "$PHYSICS_ROOT" || { echo "[ERROR] CMake configuration failed."; exit 1; }

# Build ElypsoPhysics with make
echo "[INFO] Building ElypsoPhysics..."
make -j"$(nproc)" || { echo "[ERROR] Build process failed."; exit 1; }

# Install ElypsoPhysics
echo "[INFO] Installing ElypsoPhysics..."
make install || { echo "[ERROR] Install process failed."; exit 1; }

# Record end time
TIME_END=$(date +%T)

# Success message
echo "[SUCCESS] ElypsoPhysics built and installed successfully."
echo "---------------------------------------------"
echo "Shared library: $INSTALL_DIR/lib/libElypsoPhysics.so"
echo "Static library: $INSTALL_DIR/lib/libElypsoPhysics.a"
echo "Include headers: $INSTALL_DIR/include"
echo "Build duration: $TIME_START - $TIME_END"
echo "---------------------------------------------"

# Optional: Pause to allow the user to review the output
read -r -p "Press enter to exit..."
exit 0
