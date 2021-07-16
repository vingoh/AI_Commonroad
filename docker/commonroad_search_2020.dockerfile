FROM continuumio/anaconda3

# we need to compile python modules with pybind11, 
# which only supports up to python 3.7 at the moment
RUN conda create -n commonroad-py37 python=3.7
RUN echo "source activate commonroad-py37" > ~/.bashrc
# install packages
RUN apt-get update && apt-get install \
imagemagick -y \
python3-dev -y \
make \
build-essential \
m4 \
libboost-dev \
libboost-thread-dev \
libboost-test-dev \
libboost-filesystem-dev \
cmake \
libeigen3-dev \ 
socat \
tk-dev -y
RUN bash -ic 'pip install jupyter tqdm imageio pyyaml ipywidgets networkx'

# link python3.7 to python3
RUN rm /usr/bin/python3 && ln -sf python3.7 /usr/bin/python3

# create and switch to commonroad directory
WORKDIR /commonroad

# install commonroad-io
RUN pip install commonroad-io==2020.3

# build and install commonroad-drivability-checker
RUN git clone https://gitlab.lrz.de/tum-cps/commonroad-drivability-checker.git && \
cd commonroad-drivability-checker && pip install -r requirements.txt && \
bash build.sh -e "/opt/conda/envs/commonroad-py37" -v 3.7 --cgal --serializer -i -j 4
ENV PYTHONPATH /commonroad-drivability-checker/

# create and switch commonroad-search directory
WORKDIR /commonroad/commonroad-search
# switch back to commonroad directory
WORKDIR /commonroad

# == this is the safer option
# ENTRYPOINT bash -c "\
# source activate commonroad-py37 &&\
# cd /commonroad-search/notebooks;\
# socat TCP-LISTEN:8888,fork TCP:127.0.0.1:9000 &\
# jupyter notebook --ip 0.0.0.0 --no-browser --allow-root --port 9000 "

# == this is the more convenient option
ENTRYPOINT bash -c "source activate commonroad-py37 &&\
 jupyter notebook --ip 0.0.0.0 --no-browser --allow-root --NotebookApp.token='' --NotebookApp.password=''"

