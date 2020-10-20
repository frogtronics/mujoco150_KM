COMMON=-O2 -I../inc -L../_OSX -std=c++11 -lstdc++ -mavx

all:
	clang $(COMMON) ../src/changeAngles.cpp   -lmujoco150 -lglfw.3 -o changeAngles
	clang $(COMMON) ../src/mapModel.cpp   -lmujoco150 -lglfw.3 -o mapModel
