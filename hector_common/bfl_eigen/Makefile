default: install
include $(shell rospack find mk)/cmake.mk

install: all
	cd build && ${MAKE} install

clean:
	cd build && ${MAKE} clean
	rm -Rf bin lib include
	rm -Rf build 
