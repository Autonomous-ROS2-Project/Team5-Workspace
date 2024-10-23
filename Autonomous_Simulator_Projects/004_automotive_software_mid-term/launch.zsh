#!/usr/bin/env zsh
# Ensure this script is executable with: chmod +x launch.zsh

# Base command that needs to be sourced for each pane
BASE_CMD="source ~/.zshrc && source install/setup.zsh"

# Commands for each pane
SIMULATOR_CMD="$BASE_CMD && ros2 launch simulator simulation.launch.xml"
SCENARIO_CMD="$BASE_CMD && ros2 launch scenario_runner scenario_runner.launch.xml"
EVALUATION_CMD="$BASE_CMD && ros2 launch evaluation evaluation.launch.xml"
AUTONOMOUS_CMD="$BASE_CMD && ros2 launch autonomous_driving autonomous_driving.launch.xml"

# Create a new window named "autonomous_system"
tmux new-window -n 'autonomous_system'

# First split vertically
tmux split-window -h

# Split left pane vertically
tmux select-pane -t 0
tmux split-window -v

# Split bottom-left pane horizontally
tmux select-pane -t 1
tmux split-window -v

# Split right pane vertically
tmux select-pane -t 3
tmux split-window -v

# Layout is now:
# +----+----+
# | 0  | 3  |
# | 1  | 4  |
# | 2  | 5  |
# +----+----+

sleep 1

# Send commands to each pane
# Top left pane (0) - Simulator
tmux select-pane -t 0
tmux send-keys "$SIMULATOR_CMD" C-m

sleep 3
# Middle left pane (1) - Scenario Runner
tmux select-pane -t 1
tmux send-keys "$SCENARIO_CMD" C-m

sleep 3
# Bottom left pane (2) - Evaluation
tmux select-pane -t 2
tmux send-keys "$EVALUATION_CMD" C-m

sleep 3
# Top right pane (3) - Autonomous Driving
tmux select-pane -t 3
tmux send-keys "$AUTONOMOUS_CMD" C-m

# Optional: Select the first pane (Simulator)
tmux select-pane -t 0
