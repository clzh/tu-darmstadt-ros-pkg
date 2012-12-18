all: vrmagic_devkit_wrapper

TARBALL = build/vrmusbcam-3.13g.tar.bz2
TARBALL_URL = http://www.vrmagic.com/fileadmin/downloads/imaging/Software/linux/vrmusbcam-3.13g.tar.bz2
SOURCE_DIR = build/vrmusbcam-3.13g
#MD5SUM_FILE = vrmusbcam-3.13g.tar.bz2.md5sum
UNPACK_CMD = tar xfj
include $(shell rospack find mk)/download_unpack_build.mk

vrmagic_devkit_wrapper: $(SOURCE_DIR)/unpacked
	mkdir -p include/vrmagic_devkit_wrapper
	mkdir -p lib
	cp $(SOURCE_DIR)/include/* ./include/vrmagic_devkit_wrapper
	cp $(SOURCE_DIR)/lib/* ./lib
	touch vrmagic_devkit_wrapper
clean:
	-rm -rf src $(SOURCE_DIR) vrmagic_devkit_wrapper
	-rm -rf include
	-rm -rf lib
wipe: clean
	-rm -rf build
