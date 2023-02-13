nproc=$(shell python3 -c 'import multiprocessing; print( max(multiprocessing.cpu_count() - 1, 1))')
use_integ=$(ILLIXR_INTEGRATION)

CXX := clang++-10
CC := clang-10

.PHONY: dbg plugin.dbg.so
plugin.dbg.so: dbg
dbg: build/Debug/Makefile
	make -C ../build/Debug "-j$(nproc)" && \
	rm -f plugin.$@.so && \
	ln -s ../build/Debug/libslam2.so plugin.$@.so && \
	true

.PHONY: opt plugin.opt.so
plugin.opt.so: opt
opt: build/RelWithDebInfo/Makefile
	make -C ../build/RelWithDebInfo "-j$(nproc)" && \
	rm -f plugin.$@.so && \
	ln -s ../build/RelWithDebInfo/libslam2.so plugin.$@.so && \
	true

build/Debug/Makefile:
	mkdir -p ../build/Debug && \
	cd ../build/Debug && \
	cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER=$(CXX) -DCMAKE_C_COMPILER=$(CC) -DRUN_ILLXIR=ON ../../ov_msckf/ && \
	true

build/RelWithDebInfo/Makefile:
	mkdir -p ../build/RelWithDebInfo && \
	cd ../build/RelWithDebInfo && \
	cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_COMPILER=$(CXX) -DCMAKE_C_COMPILER=$(CC) -DRUN_ILLXIR=ON ../../ov_msckf/ && \
	true

tests/run:
tests/gdb:

.PHONY: clean
clean:
	touch build && rm -rf build *.so