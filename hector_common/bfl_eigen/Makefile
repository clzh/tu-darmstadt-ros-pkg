default: install
include $(shell rospack find mk)/cmake.mk

install: all
	cd build && ${MAKE} install

clean: full_clean

full_clean:
	rm -rf install
