# Variables
# Defaults to the image name specified in your .csproj if not overridden
CONTAINER_NAME ?= tankwebui-server-image
TAG ?= latest

.PHONY: all build run clean

all: build

## ----------------------------------------------------------------------
## Build the Docker container using .NET SDK built-in container support
## ----------------------------------------------------------------------
build:
	@echo "Building container image: $(CONTAINER_NAME):$(TAG)..."
	dotnet nuget locals all --clear
	DOTNET_SYSTEM_NET_DISABLEIPV6=1 dotnet restore 
	DOTNET_SYSTEM_NET_DISABLEIPV6=1 dotnet publish -c Release \
		--os linux \
		--arch arm64 \
		-p:CONTAINER_NAME=$(CONTAINER_NAME) \
		/t:PublishContainer --disable-parallel --ignore-failed-sources

## ----------------------------------------------------------------------
## Run the container locally, exposing ports and mounting wwwroot
## ----------------------------------------------------------------------
run:
	@echo "Running container $(CONTAINER_NAME):$(TAG) with wwwroot mounted..."
		docker run --rm -it \
		-p 5165:5165 \
		-p 7288:7288 \
		-v $(PWD)/wwwroot:/app/wwwroot \
		$(CONTAINER_NAME):$(TAG)

certs:
	# Create the directory for the certs
	mkdir -p ~/.aspnet/https

	# Generate the cert (Feel free to change the password)
	dotnet dev-certs https -ep ~/.aspnet/https/rentready.pfx -p MySecretPassword123!

	# Trust the cert locally
	dotnet dev-certs https --trust

## ----------------------------------------------------------------------
## Clean the .NET project build output
## ----------------------------------------------------------------------
clean:
	@echo "Cleaning project..."
	dotnet clean
