COMMON=-O2 -I../inc -L../_LINUX -std=c++11 -mavx

default:
	g++ $(COMMON) ../src/simulate.cpp -lmujoco150 -lGL -lglew libglfw.so.3 -o simulate
	g++ $(COMMON) ../src/simpipe.cpp ../src/ethraw.cpp -lmujoco150 -lGL -lglew libglfw.so.3 -o simpipe

all: default
