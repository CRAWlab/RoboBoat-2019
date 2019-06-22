#!/bin/sh
tmux new-session -d -n 'main'
tmux new-window -n 'estop'
tmux new-window -n 'imu'
tmux new-window -n 'thrust'
tmux new-window -n 'TD'
tmux new-window -n 'pub'
tmux new-window -n 'stat' 'htop'
tmux split-window -v 
tmux resize-pane -D 10
tmux -2 attach-session -d