# Working Notes for Docker

The majority of the software for this project runs on applications installed in a monolithic docker container. The Dockerfile is located at `docker/Dockerfile`.

## Building the Docker Image

Docker produces an image from which containers can be instantiated. The image is built using the Dockerfile located at `docker/Dockerfile`. To do this, navigate to the [`docker`](../docker/) directory and run the following command:

```bash
docker build -t vine_system .
```

Of note is the tag (set with the `-t` flag) which allows you to specify a name for the image. In this case, it is recommended to call the image `vine_system` as other services such as docker compose and the `start_docker.sh` script expect this name. With that said, it is still possible to change the tag to something else if required.

The name of the expected docker image is set as an environment variable in the `.env` file (as `IMAGE_NAME`) and can be modified there or alternatively through `export IMAGE_NAME=<your_image_name>`. These changes will affect the related services listed above.

## Starting the Docker Container

There are two methods to start the Docker container: the first uses Docker Compose and is preferred, the other uses the `vine_system_start.sh` script.

To use Docker Compose: in the directory containing the `docker-compose.yml` file, run the following command:

```bash
docker compose up -d
```

To start the container in detached mode. It is then possible to connect to the container using a command such as `docker exec -it <container_name> bash` to connect to the terminal into an interactive bash terminal. Alternatively, commands can directly be run inside the container.

Alternatively, the script [`vine_system_start.sh`](../scripts/vine_system_start.sh) can be used which does mostly the same thing as the Docker Compose method but does not effectively clean up after itself.

### Troubleshooting

When using the Docker Compose method, it is possible that the container will not have access to the X11 display. To fix this, you can run the following command to allow the container to access the host's X11 socket prior to starting the docker container:

```bash
xhost +local:docker
```

### Stopping the Docker Container

If Docker Compose is being used, simply running:

```bash
docker compose down
```

From the location of the `docker-compose.yml` file, this command will stop and remove the containers defined in the Compose file.

Alternatively, the docker container will need to be stopped manually and the container removed.

## Connecting to the Container

This section is most relevant for a Docker Compose workflow. To connect to the running container, either the direct command can be used or a utility script is provided.

```bash
docker exec -it <container_name> bash
```

Replace `<container_name>` with the name of your running container. This command will open an interactive bash shell inside the container.

Alternatively, a utility script is provided to simplify this process. You can use the following command to connect to the container:

```bash
./scripts/start_docker.sh
```

This script will automatically find the running container and connect you to it provided the environment variable `IMAGE_NAME` is set correctly.
