# Docker Setup Guide
This project includes Docker configuration for Ubuntu 24.04 containerization.

## Prerequisites
- Docker Engine 20.10+
- Docker Compose 1.29+

## Building the Docker Image

```bash
docker build -t base-inertial-parameters:latest -f docker/Dockerfile .
```

## Running the Container

```bash
docker run -it --rm -v $(pwd):/base_inertial_parameters base-inertial-parameters:latest python src/examples/franka_testing/main_test.py
```

## Environment Details

The Docker image includes:
- **Base OS**: Ubuntu 24.04
- **Python**: 3.10
- **Key Libraries**:
  - NumPy 1.26.4
  - Matplotlib 3.5+
  - Pandas 1.4+
  - ROS2 Humble (rosbag2)


## Notes

- The container runs as a non-root user (`developer`) for security
- Volume mounts allow live editing of code on the host system
- Results and data are preserved in the `./results` directory
