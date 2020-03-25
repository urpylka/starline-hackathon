all: into

into:
	./scripts/docker/simulator/into_docker.sh
build:
	./scripts/docker/simulator/build_docker.sh
run:
	./scripts/docker/simulator/run_docker.sh
rviz:
	sudo docker exec -it kobuki-sim bash -c << "source /root/.bashrc && /opt/ros/kinetic/bin/rviz -d /catkin_ws/src/solution_1/rviz/tb_config.rviz"

build-n:
	./scripts/docker/simulator/build_docker.sh -n
run-n:
	./scripts/docker/simulator/run_docker.sh -n

build-r:
	./scripts/docker/kobuki/build_docker.sh
