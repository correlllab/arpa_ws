# Fix Docker Permission Issues

## Problem
You're getting "permission denied" errors when trying to use Docker because your user isn't in the docker group yet.

## Solution Options

### Option 1: Activate Group Without Logging Out (Quick Fix)

Run this command to activate the docker group in your current session:

```bash
newgrp docker
```

Then test:
```bash
docker run hello-world
```

**Note:** After running `newgrp docker`, you'll be in a new shell. Type `exit` to return to your original shell, but the docker group will still be active.

### Option 2: Log Out and Log Back In (Recommended)

1. Log out of your current session
2. Log back in
3. Test Docker:
```bash
docker run hello-world
```

### Option 3: Use Sudo (Temporary Workaround)

You can use `sudo` for now, but it's not recommended for regular use:

```bash
sudo docker build -t arpa_system:latest -f Dockerfile .
sudo docker-compose up -d
sudo docker exec -it arpa_system bash
```

## Verify It's Working

After fixing permissions, verify:

```bash
# Check you're in docker group
groups | grep docker

# Test Docker
docker run hello-world

# Check Docker version
docker --version
```

## Then Proceed with ARPA Simulation

Once Docker permissions are fixed:

```bash
cd /home/the2xman/arpa_ws
./run_simulation.sh
```

