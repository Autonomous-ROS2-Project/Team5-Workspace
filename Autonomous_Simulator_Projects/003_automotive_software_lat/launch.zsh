#!/usr/bin/env zsh

# Ensure this script is executable with: chmod +x launch.zsh

# Command to run
CMD="source ~/.zshrc && source install/setup.zsh && ros2 launch simulator simulation.launch.xml"

# Check if we're already inside a tmux session
if [ -n "$TMUX" ]; then
    # We're inside tmux, run the command directly in the current pane
    eval "$CMD"
else
    # We're not in tmux. Print an error message
    echo "Error: This script should be run from within a tmux session."
    exit 1
fi
