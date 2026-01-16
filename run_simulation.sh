#!/bin/bash
# Quick start script for ARPA simulation in Docker

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================="
echo "ARPA Simulation Docker Setup"
echo "=========================================="

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed!"
    echo "Please install Docker first: https://docs.docker.com/get-docker/"
    exit 1
fi

# Check if Docker Compose is installed
if ! command -v docker-compose &> /dev/null && ! docker compose version &> /dev/null; then
    echo "Error: Docker Compose is not installed!"
    echo "Please install Docker Compose first: https://docs.docker.com/compose/install/"
    exit 1
fi

# Allow X11 forwarding
echo "Setting up X11 forwarding..."
xhost +local:docker 2>/dev/null || true

# Check if image exists
echo "Checking for Docker image..."
if ! docker images | grep -q "arpa_system.*latest"; then
    echo "Docker image not found. Building..."
    echo "This will take 15-30 minutes..."
    docker build -t arpa_system:latest -f Dockerfile .
    echo "Build complete!"
else
    echo "Docker image found."
fi

# Check if container is already running
if docker ps | grep -q "arpa_system"; then
    echo "Container is already running!"
    echo "Attaching to container..."
    docker exec -it arpa_system bash -c "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && bash"
else
    echo "Starting Docker container..."
    echo ""
    echo "To launch simulation inside the container, run:"
    echo "  ros2 launch arpa_bringup arpa_sim.launch.py"
    echo ""
    
    # Use docker compose if available, otherwise docker run
    if docker compose version &> /dev/null; then
        docker compose up -d
        docker exec -it arpa_system bash
    elif command -v docker-compose &> /dev/null; then
        docker-compose up -d
        docker exec -it arpa_system bash
    else
        echo "Starting with docker run..."
        docker run -it --rm \
            --name arpa_system \
            --network host \
            --privileged \
            -e DISPLAY=$DISPLAY \
            -e QT_X11_NO_MITSHM=1 \
            -e ROS_DOMAIN_ID=21 \
            -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -v "$SCRIPT_DIR":/root/ros2_ws:rw \
            -v /dev/shm:/dev/shm \
            arpa_system:latest \
            bash -c "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && bash"
    fi
fi


