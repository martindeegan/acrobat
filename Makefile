TAG=devel

# arm64 architecture
build_image_arm64:
	docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
	docker build -t martindeegan/acrobat:${TAG}_arm64 --build-arg FROM_IMAGE=nvcr.io/nvidia/l4t-base:r32.3.1 .

push_image_arm64:
	docker push martindeegan/acrobat:${TAG}_arm64

run_container_arm64:
	docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
	docker run  --privileged -it martindeegan/acrobat:${TAG}_arm64

# x86_64 architecture
build_base_x86_64:
	docker build -t martindeegan/acrobat:${TAG}_base_x86_64 --target base --file dockerfiles/base/Dockerfile --cache-from martindeegan/acrobat:latest_base_x86_64 --build-arg FROM_IMAGE=nvidia/cuda:10.2-devel-ubuntu18.04 .

push_base_x86_64:
	docker push martindeegan/acrobat:${TAG}_base_x86_64

build_testing_x86_64:
	docker build -t martindeegan/acrobat:${TAG}_testing_x86_64 --target testing --cache-from martindeegan/acrobat:latest_testing_x86_64  --build-arg FROM_IMAGE=martindeegan/acrobat:${TAG}_base_x86_64 .

build_runtime_x86_64:
	docker build -t martindeegan/acrobat:${TAG}_runtime_x86_64 --target runtime --cache-from martindeegan/acrobat:latest_runtime_x86_64 --build-arg FROM_IMAGE=martindeegan/acrobat:${TAG}_base_x86_64 .
	docker push martindeegan/acrobat:${TAG}_runtime_x86_64

run_container_x86_64:
	docker run  --privileged -it martindeegan/acrobat:${TAG}_x86_64
