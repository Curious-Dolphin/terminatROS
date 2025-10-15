#!/usr/bin/env bash

# Exit immediately if a command exits with a non-zero status.
set -e

# --- Configuration ---
ALIAS_NAME="terminatros"
SRC_PATH="src/terminatros.py"

# --- Helper Functions for Colored Output ---
info() {
    echo -e "\033[1;34m[INFO]\033[0m $1"
}
success() {
    echo -e "\033[1;32m[SUCCESS]\033[0m $1"
}
error() {
    echo -e "\033[1;31m[ERROR]\033[0m $1"
}
warn() {
    echo -e "\033[1;33m[WARN]\033[0m $1"
}


# --- 1. Find and Source ROS 2 ---
info "Searching for ROS 2 environment..."
if [ -z "$ROS_DISTRO" ]; then
    ROS2_DISTROS=(humble foxy galactic rolling)
    FOUND_ROS2=false
    for distro in "${ROS2_DISTROS[@]}"; do
        if [ -f "/opt/ros/$distro/setup.bash" ]; then
            info "Found ROS 2 '$distro'. Sourcing environment..."
            source "/opt/ros/$distro/setup.bash"
            FOUND_ROS2=true
            break
        fi
    done
    if [ "$FOUND_ROS2" = false ]; then
        error "Could not find a ROS 2 installation in /opt/ros/. Please install ROS 2 first."
        exit 1
    fi
else
    info "ROS 2 environment is already sourced (Distro: $ROS_DISTRO)."
fi


# --- 2. Dependency Checks ---
info "Checking dependencies..."

if ! command -v ros2 &> /dev/null; then
    error "ROS 2 command not found, even after sourcing. Your installation may be broken."
    exit 1
fi
info "✓ ROS 2 command is available."

# Check for pip3, and install if it's missing (NEW & IMPROVED SECTION)
if ! command -v pip3 &> /dev/null; then
    warn "pip3 (Python's package manager) is not installed, but is required."
    read -p "Do you want to install it now using 'apt'? (y/n) " -n 1 -r
    echo # move to a new line
    if [[ "$RE