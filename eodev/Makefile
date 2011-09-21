all: eodev

TARBALL = build/E0-1.2.0.zip
TARBALL_URL = http://goo.gl/8WTPD
SOURCE_DIR = build/E0-1.2.0
MD5SUM_FILE = EO-1.2.0.zip.md5sum
UNPACK_CMD = unzip
include $(shell rospack find mk)/download_unpack_build.mk

eodev: $(SOURCE_DIR)/unpacked
	mkdir -p src
	cd $(SOURCE_DIR)/eo && ./build_gcc_linux_release
	rm -rf src
clean:
	-rm -rf src $(SOURCE_DIR) eodev
wipe: clean
	-rm -rf build