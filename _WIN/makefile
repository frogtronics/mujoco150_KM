COMMON=/O2 /MT /EHsc /arch:AVX /I../inc /I../../libunistd/unistd /Fe../_WIN/

all:
	cl $(COMMON) ../src/changeAngles.cpp ../_WIN/glfw3.lib  ../_WIN/mujoco150.lib ../inc/getopt.c ../inc/getopt_long.c
	
	del *.obj
