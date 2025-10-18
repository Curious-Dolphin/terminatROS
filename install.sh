#!/usr/bin/env bash

# Exit immediately if a command fails, and print each command.
set -ex

# --- Configuration ---
COMMAND_NAME="terminatros"
SRC_PATH="main.py"
VENV_PATH="./venv"
PYTHON_DEPENDENCIES=("textual==0.58.0" "psutil" "pyyaml")

# --- Helper Functions for Colored Output ---
info() { echo -e "\033[1;34m[INFO]\033[0m $1"; }
success() { echo -e "\033[1;32m[SUCCESS]\033[0m $1"; }
error() { echo -e "\033[1;31m[ERROR]\033[0m $1"; }
warn() { echo -e "\033[1;33m[WARN]\033[0m $1"; }

# --- 1. Dependency Checks & Virtual Environment Setup ---
info "Checking system dependencies..."
if ! command -v python3 &> /dev/null; then error "python3 could not be found." && exit 1; fi
# More robust check for 'ensurepip', which is required to create a venv successfully.
if ! python3 -c "import ensurepip" &> /dev/null; then
    warn "Python 'venv' module or 'ensurepip' is not available. Attempting to install the correct version."
    # Detect the major.minor version of the python3 command
    PYTHON_VERSION=$(python3 --version | cut -d' ' -f2 | cut -d'.' -f1,2)
    VENV_PACKAGE="python${PYTHON_VERSION}-venv"
    info "Attempting to install '$VENV_PACKAGE' via apt..."
    sudo apt-get update && sudo apt-get install -y "$VENV_PACKAGE"
fi
info "✓ Python's venv module is available."

# --- VENV VALIDATION AND CREATION (IMPROVED LOGIC) ---
info "Validating or creating Python virtual environment at '$VENV_PATH'..."
RECREATE_VENV=false
if [ -f "$VENV_PATH/bin/pip" ]; then
    # A venv exists. Check if its Python interpreter path is valid.
    VENV_PYTHON_INTERPRETER=$(head -n 1 "$VENV_PATH/bin/pip" | sed 's/^#!//')
    if [ ! -f "$VENV_PYTHON_INTERPRETER" ]; then
        # The interpreter points to an old, non-existent location.
        warn "Stale virtual environment detected (project likely moved or copied)."
        RECREATE_VENV=true
    else
        info "Virtual environment appears to be valid."
    fi
else
    # No venv exists at all.
    info "No virtual environment found."
    RECREATE_VENV=true
fi

if [ "$RECREATE_VENV" = true ]; then
    info "Recreating virtual environment..."
    rm -rf "$VENV_PATH"
    python3 -m venv "$VENV_PATH"
fi
info "✓ Virtual environment is ready."
# --- END IMPROVED LOGIC ---

info "Installing Python packages..."
"$VENV_PATH/bin/pip" install --upgrade "${PYTHON_DEPENDENCIES[@]}"
info "✓ All required Python packages are installed in the venv."


# --- 2. Shell Function Configuration (FINAL FIX) ---
info "Configuring shell function..."
SHELL_CONFIG_FILE=""
if [[ "$SHELL" == *"bash"* ]]; then
    SHELL_CONFIG_FILE="$HOME/.bashrc"
elif [[ "$SHELL" == *"zsh"* ]]; then
    SHELL_CONFIG_FILE="$HOME/.zshrc"
else
    error "Unsupported shell: $SHELL." && exit 1
fi
info "Detected shell config file: $SHELL_CONFIG_FILE"

info "Constructing absolute paths..."
# Get the directory where the install.sh script is currently running. This makes it generic.
CURRENT_DIR=$(pwd)

# Resolve path to be clean, removing any './'
CLEAN_VENV_PATH=$(realpath -m "$CURRENT_DIR/$VENV_PATH")
FULL_SRC_PATH="$CURRENT_DIR/$SRC_PATH"
FULL_VENV_ACTIVATE_PATH="$CLEAN_VENV_PATH/bin/activate"
info "Source path: $FULL_SRC_PATH"
info "Venv activate path: $FULL_VENV_ACTIVATE_PATH"

# Use printf -v to safely create the multi-line function string
printf -v FUNCTION_DEFINITION 'function %s() {\n    source "%s" && python3 "%s" "$@"\n}' \
    "$COMMAND_NAME" \
    "$FULL_VENV_ACTIVATE_PATH" \
    "$FULL_SRC_PATH"

START_MARKER="# START terminatROS function"
END_MARKER="# END terminatROS function"

info "Cleaning up old definitions from $SHELL_CONFIG_FILE..."
awk -v start="$START_MARKER" -v end="$END_MARKER" '
    !($0 ~ start), c;
    $0 ~ start {c=1}
    $0 ~ end {c=0}
' "$SHELL_CONFIG_FILE" > "${SHELL_CONFIG_FILE}.tmp" && mv "${SHELL_CONFIG_FILE}.tmp" "$SHELL_CONFIG_FILE"
sed -i "/alias $COMMAND_NAME=/d" "$SHELL_CONFIG_FILE"

info "Adding new function to $SHELL_CONFIG_FILE..."
{
    echo ""
    echo "$START_MARKER"
    echo "$FUNCTION_DEFINITION"
    echo "$END_MARKER"
} >> "$SHELL_CONFIG_FILE"
info "✓ Shell function configured."


# --- 3. Final Instructions ---
success "Configuration complete!"
warn "To use the '$COMMAND_NAME' command, you must reload your shell:"
echo -e "  \033[1;32msource $SHELL_CONFIG_FILE\033[0m"
echo

