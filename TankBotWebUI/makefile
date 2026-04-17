-include .env
PASSWORD_SECRET=dumbpassword
CERT_NAME=tanksstatuscert
CERT_FOLDER =./TankUI/certs/
CONTAINER_NAME?=tankstatus-server-image


.PHONY: all build run set-up-certs clean open
all: clean set-up-certs build run open

build:
	dotnet publish --os linux /t:PublishContainer
	podman image ls

run:
	podman-compose up

set-up-certs:
	mkdir -p $(CERT_FOLDER)
	dotnet dev-certs https -ep $(CERT_FOLDER)$(CERT_NAME).pfx -p $(PASSWORD_SECRET)
	chmod 644 $(CERT_FOLDER)$(CERT_NAME).pfx
	echo "CERT_PASSWORD=$(PASSWORD_SECRET)" > .env
	echo "CERT_NAME=$(CERT_NAME)">>.env
	echo "CERT_FOLD=/app/certs/">>.env
	echo "CONTAINER_NAME=$(CONTAINER_NAME)">>.env

clean:
	podman-compose down
	dotnet dev-certs https --clean
	dotnet clean

open:
	xdg-open https://localhost:7288/test-tank
