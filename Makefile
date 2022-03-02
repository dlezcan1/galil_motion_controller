all: galilcontroller_debug.o

galilcontroller_debug.o: GalilMotionController/GalilController.h
	g++ -std=c++11 GalilMotionController/main.cpp GalilMotionController/GalilController.cpp -lgclib -lgclibo -o debug_galil_controller.out
