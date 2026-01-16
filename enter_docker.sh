#!/bin/bash

# Script to start and enter the Docker container

cd "$(dirname "$0")"

# Check if container is already running
if sudo docker ps | grep -q arpa_system; then
    echo "Container is already running. Entering..."
    sudo docker exec -it arpa_system /bin/bash
elif sudo docker ps -a | grep -q arpa_system; then
    echo "Container exists but is stopped. Starting and entering..."
    sudo docker start arpa_system
    sudo docker exec -it arpa_system /bin/bash
else
    echo "Container doesn't exist. Starting with docker-compose..."
    sudo docker-compose up -d
    echo "Waiting for container to be ready..."
    sleep 2
    sudo docker exec -it arpa_system /bin/bash
fi

