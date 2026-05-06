include .env
export $(shell sed -ne 's/ *#.*$$//; /./ s/=.*$$// p' .env)   

build:
	docker build -t $(JUPYTER_LAB_CONTAINER) ./
