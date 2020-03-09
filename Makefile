all: into

into:
	./docker/simulator/into_docker.sh
build:
	./docker/simulator/build_docker.sh
run:
	./docker/simulator/run_docker.sh
