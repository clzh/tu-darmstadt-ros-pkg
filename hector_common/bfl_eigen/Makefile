default: install

clean:
	cd build && ${MAKE} clean
	rm -Rf bin lib include build 

include $(shell rospack find mk)/cmake.mk

install: all
	cd build && ${MAKE} install

