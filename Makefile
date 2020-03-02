# BUILD_TYPE can be any of [runtime, testing]
BUILD_TYPE=runtime
PUSH_TYPE=runtime
TAG=${USER}_devel
FROM_TAG=${TAG}
CACHE_TAG=${TAG}

# arm64 architecture
build_base_arm64:
	docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
	docker build -t martindeegan/acrobat:${TAG}_base_arm64 \
				 --file dockerfiles/base/Dockerfile \
				 --cache-from martindeegan/acrobat:latest_base_arm64 \
				 --build-arg FROM_IMAGE=nvcr.io/nvidia/l4t-base:r32.3.1 \
				 .

build_dependencies_arm64:
	docker build -t martindeegan/acrobat:${TAG}_dependencies_arm64b \
				 --file dockerfiles/dependencies/Dockerfile \
				 --cache-from martindeegan/acrobat:${CACHE_TAG}_dependencies_arm64 \
				 --build-arg FROM_IMAGE=martindeegan/acrobat:${FROM_TAG}_base_arm64 \
				 .

build_arm64:
	docker build -t martindeegan/acrobat:${TAG}_${BUILD_TYPE}_arm64 \
				 --file dockerfiles/${BUILD_TYPE}/Dockerfile \
				 --cache-from martindeegan/acrobat:${CACHE_TAG}_${BUILD_TYPE}_arm64 \
				 --build-arg FROM_IMAGE=martindeegan/acrobat:${FROM_TAG}_dependencies_arm64 \
				 .

push_arm64:
	docker push martindeegan/acrobat:${TAG}_${PUSH_TYPE}_arm64

# x86_64 architecture
build_base_x86_64:
	docker build -t martindeegan/acrobat:${TAG}_base_x86_64 \
				 --file dockerfiles/base/Dockerfile \
				 --cache-from martindeegan/acrobat:latest_base_x86_64 \
				 --build-arg FROM_IMAGE=nvidia/cuda:10.2-devel-ubuntu18.04 \
				 .

build_dependencies_x86_64:
	docker build -t martindeegan/acrobat:${TAG}_dependencies_x86_64 \
				 --file dockerfiles/dependencies/Dockerfile \
				 --cache-from martindeegan/acrobat:${CACHE_TAG}_dependencies_x86_64 \
				 --build-arg FROM_IMAGE=martindeegan/acrobat:${FROM_TAG}_base_x86_64 \
				 .

build_x86_64:
	docker build -t martindeegan/acrobat:${TAG}_${BUILD_TYPE}_x86_64 \
				 --file dockerfiles/${BUILD_TYPE}/Dockerfile \
				 --cache-from martindeegan/acrobat:${CACHE_TAG}_${BUILD_TYPE}_x86_64 \
				 --build-arg FROM_IMAGE=martindeegan/acrobat:${FROM_TAG}_dependencies_x86_64 \
				 .

push_x86_64:
	docker push martindeegan/acrobat:${TAG}_${PUSH_TYPE}_x86_64

