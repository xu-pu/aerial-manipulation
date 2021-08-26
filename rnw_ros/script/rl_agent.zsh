#!/usr/bin/env zsh

echo "Enter rl_agent.zsh"

source /home/sheep/.zshrc

conda_on
rl_on

cd /home/sheep/Code/LearningRockwalk

./rl_agent_node.py