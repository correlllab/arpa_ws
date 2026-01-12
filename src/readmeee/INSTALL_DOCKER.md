# Installing Docker on Ubuntu

Follow these steps to install Docker on your Ubuntu system.

## Option 1: Install Docker using apt (Recommended)

### Step 1: Update package index
```bash
sudo apt update
```

### Step 2: Install prerequisites
```bash
sudo apt install -y ca-certificates curl gnupg lsb-release
```

### Step 3: Add Docker's official GPG key
```bash
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
```

### Step 4: Set up Docker repository
```bash
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
```

### Step 5: Install Docker Engine
```bash
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

### Step 6: Add your user to docker group (so you don't need sudo)
```bash
sudo usermod -aG docker $USER
```

**Important:** You need to log out and log back in (or restart) for this to take effect!

### Step 7: Verify installation
```bash
docker --version
docker compose version
```

## Option 2: Quick Install using apt (Simpler, but older version)

If the above doesn't work, you can use the Ubuntu repository version:

```bash
sudo apt update
sudo apt install -y docker.io docker-compose
sudo usermod -aG docker $USER
```

**Then log out and log back in!**

## Option 3: Install using Snap (Alternative)

```bash
sudo snap install docker
```

## After Installation

1. **Log out and log back in** (or restart your computer) so the docker group membership takes effect.

2. **Test Docker without sudo:**
```bash
docker run hello-world
```

3. **If you still need sudo**, check your group membership:
```bash
groups
# Should show 'docker' in the list
```

## Troubleshooting

### "Permission denied" errors
- Make sure you logged out and back in after adding yourself to the docker group
- Or run commands with `sudo` (not recommended for regular use)

### "Cannot connect to Docker daemon"
- Start the Docker service:
```bash
sudo systemctl start docker
sudo systemctl enable docker  # Start on boot
```

### Check Docker status
```bash
sudo systemctl status docker
```

## Next Steps

Once Docker is installed, return to the ARPA simulation setup:

```bash
cd /home/the2xman/arpa_ws
./run_simulation.sh
```

Or follow the instructions in `QUICKSTART.md`

