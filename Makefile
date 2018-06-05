CC              := g++
CFLAGS          :=  -std=c++11
OBJECTS         := 
LIBRARIES       := -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_nonfree -lopencv_objdetect -lopencv_ocl -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_ts -lopencv_video -lopencv_videostab

.PHONY: all clean

all: clean test

test: 
	$(CC) $(CFLAGS) -o test main.cpp $(LIBRARIES)
        
clean:
	rm -f test
