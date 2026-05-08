#!/bin/bash
SESSION="bxi_nav"

if ! command -v tmux &> /dev/null; then
    sudo apt update && sudo apt install tmux -y
fi
tmux kill-session -t $SESSION 2>/dev/null

tmux new-session -d -s $SESSION
tmux set-option -g mouse on
tmux set-option -g pane-border-status top
tmux set-option -g pane-border-format " #[fg=black,bg=green] #T #[default] "

# ---------------------------------------------------------------
# 核心布局划分
# ---------------------------------------------------------------

tmux split-window -h -p 45 -t $SESSION

tmux select-pane -t 0
tmux split-window -v -p 50 -t $SESSION  # 左侧上下平分
tmux select-pane -t 0 -T "里程计"
tmux select-pane -t 1 -T "导航"
tmux select-pane -t 2 -T "雷达驱动"


tmux send-keys -t $SESSION:0.0 "source install/setup.bash;ros2 launch point_lio point_lio.launch.py" C-m
tmux send-keys -t $SESSION:0.1 "source install/setup.bash;ros2 launch vehicle_simulator system_real_robot.launch.py" C-m
tmux send-keys -t $SESSION:0.2 "source install/setup.bash;ros2 launch livox_ros_driver2 msg_MID360s_launch.py" C-m

tmux select-pane -t 0
tmux attach-session -t $SESSION
