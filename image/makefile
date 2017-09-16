objects=main.o  ImageController.o

LIBS=`pkg-config --libs --cflags opencv` -lzbar -lpng -lboost_system

INCLUDE=-I ./ -I /usr/local/include -I /usr/include

DEFINE=

LDFLAGS=-g 

BIN=qr

${BIN}:${objects}
	g++ ${LDFLAGS} -o ${BIN} ${objects} ${LIBS} ${INCLUDE} ${DEFINE}

main.o:main.cpp ImageController.h
	g++ ${LDFLAGS} -c main.cpp

ImageController.o:ImageController.cpp ImageController.h
	g++ ${LDFLAGS} -c ImageController.cpp


clean:
	rm -f ${BIN} ${objects} core
