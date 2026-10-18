#!/bin/bash
# aman/infra/setup.sh
# Ticket: M-01 — Provision cloud simulation server
# Run this script on a fresh Ubuntu 22.04 instance (AWS c5.2xlarge or equivalent)
# Usage: bash setup.sh

set -euo pipefail

echo "=== Drone Traffic Control — Server Setup ==="
echo "Target: Python 3.11, Docker, Git, CUDA (optional)"

# 1. System update
sudo apt-get update -y && sudo apt-get upgrade -y

# 2. Core dependencies
sudo apt-get install -y \
    git \
    curl \
    wget \
    build-essential \
    python3.11 \
    python3.11-venv \
    python3.11-dev \
    python3-pip \
    docker.io \
    docker-compose \
    htop \
    tmux \
    unzip

# 3. Python venv for the project
cd ~
python3.11 -m venv drone-venv
source drone-venv/bin/activate

# 4. Clone the repo (replace with actual remote URL once pushed)
# git clone https://github.com/YOUR_ORG/drone-traffic-control.git
# cd drone-traffic-control

# 5. Install Python dependencies
pip install --upgrade pip
pip install \
    numpy \
    scipy \
    cvxpy \
    pettingzoo \
    gymnasium \
    torch \
    torchrl \
    fastapi \
    uvicorn \
    httpx \
    matplotlib \
    seaborn \
    pandas \
    wandb \
    onnx \
    onnxruntime \
    jupyter \
    ipykernel

# 6. Docker — add ubuntu user to docker group
sudo usermod -aG docker ubuntu
echo "Log out and back in for Docker group to take effect."

# 7. Verify
python3 --version
git --version
docker --version

echo ""
echo "=== Setup complete ==="
echo "NEXT: Run install_sim.sh to install BlueSky and Flightmare"
echo "NEXT: Document the server IP in aman/infra/server.md (DO NOT commit IP+key together)"
