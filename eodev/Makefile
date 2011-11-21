all: eodev

TARBALL = build/EO-1.2.0.zip
TARBALL_URL = http://goo.gl/8WTPD
SOURCE_DIR = build/eo
MD5SUM_FILE =
UNPACK_CMD = unzip
include $(shell rospack find mk)/download_unpack_build.mk

eodev: ${SOURCE_DIR}/unpacked
	mkdir -p src
	cd $(SOURCE_DIR) && ./build_gcc_linux_release
	rm -rf src
clean:
	-rm -rf src $(SOURCE_DIR) eodev
wipe: clean
	-rm -rf build