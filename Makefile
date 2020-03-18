all: into

into:
	./scripts/ocker/simulator/into_docker.sh
build:
	./scripts/docker/simulator/build_docker.sh
run:
	./scripts/docker/simulator/run_docker.sh
