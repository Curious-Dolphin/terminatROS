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
    if [[ "$REPLY" =~ ^[Yy]$ ]]; then
        info "Attempting to install python3-pip. This may ask for your password."
        # Update package list and install pip3
        sudo apt-get update
        sudo apt-get install -y python3-pip
        # Verify that the installation was successful
        if ! command -v pip3 &> /dev/null; then
            error "pip3 installation failed. Please try installing 'python3-pip' manually."
            exit 1
        fi
        success "pip3 has been successfully installed."
    else
        error "Installation cancelled. pip3 is required to proceed."
        exit 1
    fi
fi
info "✓ pip3 is installed."

# Check for 'textual' Python package
info "Checking for 'textual' library..."
if ! pip3 show textual &> /dev/null; then
    warn "'textual' Python package not found."
    read -p "Do you want to install it now? (y/n) " -n 1 -r
    echo
    if [[ "$REPLY" =~ ^[Yy]$ ]]; then
        info "Installing 'textual' via pip3..."
        pip3 install textual
    else
        error "Installation cancelled. 'textual' is required to run the app."
        exit 1
    fi
fi
info "✓ 'textual' is installed."


# --- 3. Alias Configuration (Unchanged) ---
info "Configuring alias..."
SHELL_CONFIG_FILE=""
if [[ "$SHELL" == *"bash"* ]]; then
    SHELL_CONFIG_FILE="$HOME/.bashrc"
elif [[ "$SHELL" == *"zsh"* ]]; then
    SHELL_CONFIG_FILE="$HOME/.zshrc"
else
    error "Unsupported shell: $SHELL. Please configure the alias manually."
    exit 1
fi
info "Detected shell config file: $SHELL_CONFIG_FILE"
FULL_SRC_PATH=$(realpath "$SRC_PATH")
ALIAS_CMD="alias $ALIAS_NAME='python3 \"$FULL_SRC_PATH\"'"
if ! grep -q "# Alias for terminatROS" "$SHELL_CONFIG_FILE"; then
    info "Adding alias to $SHELL_CONFIG_FILE..."
    echo -e "\n# Alias for terminatROS" >> "$SHELL_CONFIG_FILE"
    echo "$ALIAS_CMD" >> "$SHELL_CONFIG_FILE"
else
    info "Alias already exists. Skipping."
fi


# --- 4. Final Instructions (Unchanged) ---
success "Configuration complete!"
warn "To use the '$ALIAS_NAME' command, you must first restart your terminal"
warn "or run: source $SHELL_CONFIG_FILE"