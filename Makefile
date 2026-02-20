IMAGE_NAME=docking-humble
CONTAINER_NAME=docking_container

.PHONY: build start attach stop shell restart clean status

build:
	docker build -t $(IMAGE_NAME) .

start:
	xhost +local:docker
	docker run -it \
		--name $(CONTAINER_NAME) \
		--net=host \
		--privileged \
		--env="DISPLAY=$(DISPLAY)" \
		--env="TURTLEBOT3_MODEL=burger" \
		--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		--volume="$(PWD)/docking_ws:/root/docking_ws" \
		$(IMAGE_NAME)

attach:
	docker start -ai $(CONTAINER_NAME)

shell:
	docker exec -it $(CONTAINER_NAME) bash

stop:
	docker stop $(CONTAINER_NAME)

restart: stop attach

status:
	docker ps -a | grep $(CONTAINER_NAME) || true

clean:
	docker rm -f $(CONTAINER_NAME)
