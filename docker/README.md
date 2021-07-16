# Docker for CommonRoad Search
In the following, we explain two ways of using Docker for CommonRoad Search. Special thanks to Tom Dörr, who helped us in creating this.

## Option 1 (recommended): Pull the prebuilt docker image
### Install docker
On Ubuntu/Debian/Linux-Mint etc.:
```
sudo apt-get install docker.io
```
For other platforms, visit https://docs.docker.com/get-docker/.


### Run the docker container
In the root folder of the CommonRoad Search repository:
```
sudo docker run -it -p 9000:8888 --mount src="$(pwd)",target=/commonroad/commonroad-search,type=bind gitlab.lrz.de:5005/tum-cps/commonroad-search:2020_AI
```
You can now access the Jupyter Notebook by opening `localhost:9000` in your browser.

## Option 2: Build the docker image locally
### Install docker
On Ubuntu/Debian/Linux-Mint etc.:
```
sudo apt-get install docker.io
```
For other platforms, visit https://docs.docker.com/get-docker/.

### Build docker image
In the folder where `commonroad_search_2020.dockerfile` is located:
```
sudo docker build -t commonroad-search:2020_AI  - < commonroad_search_2020.dockerfile
```

### Run the docker container
In the root folder of the CommonRoad Search repository:
```
sudo docker run -it -p 9000:8888 --mount src="$(pwd)",target=/commonroad/commonroad-search,type=bind commonroad-search:2020_AI
```
You can now access the Jupyter Notebook by opening `localhost:9000` in your browser.