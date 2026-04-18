GODOT_DOCKER_TAG=godot_builder
# 1. Include the .env file
include .env
# 2. Export the variables so commands in the Makefile can use them
export

.PHONY: all
all: build html5

# Example targets showing how you can use the variables now:
build:
	@echo "Building image: $(IMAGE_NAME):$(IMAGE_TAG)"
	docker compose build

html5:
	@echo "Running HTML5 export using $(IMAGE_NAME)..."
	docker compose run --rm export-html5

linux:
	@echo "Running Linux export using $(IMAGE_NAME)..."
	docker compose run --rm export-linux
