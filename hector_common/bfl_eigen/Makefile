default: install

SVN_DIR = build/orocos-bfl-0.8.0
SVN_URL = http://svn.mech.kuleuven.be/repos/orocos/branches/bfl/branch-0.8/
SVN_REVISION = -r 33362
SVN_PATCH = orocos-bfl-0.8.0.patch boost-reference.patch total.patch
include $(shell rospack find mk)/svn_checkout.mk
include $(shell rospack find mk)/cmake.mk

SRCS := $(wildcard src/* src/*/* src/*/*/*)

all: $(SVN_DIR) $(SVN_PATCH) patched

bfl: all

install: bfl
	cd build && ${MAKE} install

clean: full_clean

full_clean:
	rm -rf install patched

