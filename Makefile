all:: makerun

REMOTE_HOST=pi@10.0.0.189
REMOTE_PATH=git/sprobot

CC=g++

SRC=module
OBJ=$(shell uname -m)

ifeq "$(OBJ)" "aarch64"
PIGPIO_LIB=-lpigpio
else
PIGPIO_LIB=
endif

#CAMERA_SDK=-lOrbbecSDK
CAMERA_SDK=-lArducamDepthCamera

CPPFLAGS=-g -std=c++17 -Wall  -I/usr/include -I/usr/local/include -fPIC -DARCH=$(OBJ)
LDFLAGS=-lstdc++ -fPIC

$(OBJ)/%.o: $(OBJ)/%.c
	$(CC) $(CPPFLAGS) -c -o $@ $<

$(OBJ)/%.o: $(SRC)/%.cpp $(SRC)/%.h
	$(CC) $(CPPFLAGS) -c -o $@ $<

test1: $(OBJ)/test1.o $(OBJ)/mechanics.o $(OBJ)/structures.o $(OBJ)/module.o $(OBJ)/camera.o $(OBJ)/image.o
	$(CC) -o $@ $^ $(LDFLAGS) -pthread $(PIGPIO_LIB) -lrt $(CAMERA_SDK) -ljpeg

imagetest: $(OBJ)/image.o $(OBJ)/imagetest.o
	$(CC) -o $@ $^ $(LDFLAGS) -lrt -lm
	./imagetest

cameratest: $(OBJ)/cameratest.o $(OBJ)/camera.o $(OBJ)/image.o
	$(CC) -o $@ $^ $(LDFLAGS) -lOrbbecSDK -ljpeg

arducamtest: $(OBJ)/arducamtest.o
	$(CC) -o $@ $^ $(LDFLAGS) -lArducamDepthCamera -ljpeg

module.so: $(OBJ)/module.o  $(OBJ)/mechanics.o $(OBJ)/structures.o $(OBJ)/camera.o $(OBJ)/image.o
	$(CC) -shared -o $@ $^ $(LDFLAG) $(PIGPIO_LIB) -lrt -lm -lpthread $(CAMERA_SDK) -ljpeg

makerun:
	rsync -rci * $(REMOTE_HOST):$(REMOTE_PATH)
	rsh $(REMOTE_HOST) "cd $(REMOTE_PATH) ; make module.so"
	#rsh $(REMOTE_HOST) "cd $(REMOTE_PATH) ; sudo LD_LIBRARY_PATH=/usr/local/lib python web.py"

run:
	rsync -rci * $(REMOTE_HOST):$(REMOTE_PATH)
	rsh $(REMOTE_HOST) "cd $(REMOTE_PATH) ; sudo LD_LIBRARY_PATH=/usr/local/lib python web.py"

runtest2:
	rsync -rci * $(REMOTE_HOST):$(REMOTE_PATH)
	rsh $(REMOTE_HOST) "cd $(REMOTE_PATH) ; make test2"
	rsh $(REMOTE_HOST) "cd $(REMOTE_PATH) ; sudo ./test2"

runtest1:
	rsync -rci * $(REMOTE_HOST):$(REMOTE_PATH)
	rsh $(REMOTE_HOST) "cd $(REMOTE_PATH) ; make test1"
	rsh $(REMOTE_HOST) "cd $(REMOTE_PATH) ; sudo LD_LIBRARY_PATH=/usr/local/lib ./test1"

runcamera:
	rsync -rci * $(REMOTE_HOST):$(REMOTE_PATH)
	rsh $(REMOTE_HOST) "cd $(REMOTE_PATH) ; make cameratest"
	rsh $(REMOTE_HOST) "cd $(REMOTE_PATH) ; export LD_LIBRARY_PATH=/usr/local/lib ; ./cameratest"

runarducam:
	rsync -rci * $(REMOTE_HOST):$(REMOTE_PATH)
	rsh $(REMOTE_HOST) "cd $(REMOTE_PATH) ; make arducamtest"
	rsh $(REMOTE_HOST) "cd $(REMOTE_PATH) ; ./arducamtest"


loadjpeg:
	rsync $(REMOTE_HOST):$(REMOTE_PATH)/depth.jpg .
	rsync $(REMOTE_HOST):$(REMOTE_PATH)/depth.jpg.dump .
	rsync $(REMOTE_HOST):$(REMOTE_PATH)/color.jpg .
	rsync $(REMOTE_HOST):$(REMOTE_PATH)/color.jpg.dump .

loadajpeg:
	rsync $(REMOTE_HOST):$(REMOTE_PATH)/adepth.jpg .
	#rsync $(REMOTE_HOST):$(REMOTE_PATH)/adepth.jpg.dump .
	rsync $(REMOTE_HOST):$(REMOTE_PATH)/aconf.jpg .
	#rsync $(REMOTE_HOST):$(REMOTE_PATH)/aconf.jpg.dump .
	#rsync $(REMOTE_HOST):$(REMOTE_PATH)/araw.jpg .
	#rsync $(REMOTE_HOST):$(REMOTE_PATH)/araw.jpg.dump .

loadwebjpeg:
	rsync $(REMOTE_HOST):$(REMOTE_PATH)/static/depth.jpg analysis/depth$(SUFFIX).jpg
	rsync $(REMOTE_HOST):$(REMOTE_PATH)/static/depth.jpg.dump analysis/depth$(SUFFIX).dump
	rsync $(REMOTE_HOST):$(REMOTE_PATH)/static/color.jpg analysis/color$(SUFFIX).jpg
	rsync $(REMOTE_HOST):$(REMOTE_PATH)/static/color.jpg.dump analysis/color$(SUFFIX).dump

killtest1:
	#rsh $(REMOTE_HOST) "sudo kill `ps -C test1 -o pid=`"
	rsh $(REMOTE_HOST) "ps -C test1 -o pid= | xargs sudo kill"

pstest1:
	rsh $(REMOTE_HOST) "ps -e | grep test1"

piclean:
	rsync -rci * $(REMOTE_HOST):$(REMOTE_PATH)
	rsh $(REMOTE_HOST) "cd $(REMOTE_PATH) ; make clean"

shutdown:
	rsh $(REMOTE_HOST) "cd $(REMOTE_PATH) ; sudo shutdown -h now ; exit"

clean:
	rm -rf $(OBJ) test1 module.so imagetest cameratest arducamtest
	mkdir $(OBJ)
