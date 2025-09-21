#!/bin/bash
set -e

# Bring down the stack to save system resources while building
docker compose down

# Rebuild and start the stack
docker compose up -d --build
