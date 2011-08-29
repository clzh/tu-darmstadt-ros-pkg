default: install
include $(shell rospack find mk)/cmake.mk

install: all
	cd build && ${MAKE} install

clean:
	cd build && ${MAKE} clean
	rm -Rf install
	rm -Rf build 
