# Server Access Guide
> Fill this in after provisioning. Commit the IP but NOT the key path or password.
> Share the actual key via Signal/WhatsApp only.

## Instance Details

| Field | Value |
|-------|-------|
| Provider | AWS EC2 / GCP Compute Engine |
| Instance type | c5.2xlarge (8 vCPU, 16 GB RAM) |
| OS | Ubuntu 22.04 LTS |
| Region | us-east-1 |
| Public IP | `TODO: fill in after provisioning` |
| SSH user | ubuntu |

## SSH Command

```bash
ssh -i ~/.ssh/drone-tc-key.pem ubuntu@<PUBLIC_IP>
```

> Key file: share via secure channel. DO NOT commit `.pem` files.
> The key file is in `.gitignore`.

## Port Mapping

| Service | Port |
|---------|------|
| Sovereign Core API | 8000 |
| Jupyter Lab | 8888 |
| W&B local (if used) | 8080 |

## Environment Setup (after SSH)

```bash
source ~/drone-venv/bin/activate
cd ~/drone-traffic-control
git pull origin dev
```

## Tmux Session Layout

```
Window 0: simulation runner
Window 1: API server (uvicorn)
Window 2: training (python train_shielded.py)
Window 3: monitoring (htop)
```

Start a named session:
```bash
tmux new-session -s dtc
```

## Who Has Access

| Person | Key received | Date |
|--------|-------------|------|
| Salman | [ ] | |
| Anas | [ ] | |
| Aman | [ ] | |
