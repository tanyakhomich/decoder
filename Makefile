CXX ?= c++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic
PYTHON ?= python3

.PHONY: all test clean

all: build/jpeg_decoder

build/jpeg_decoder: main.cpp
	mkdir -p build
	$(CXX) $(CXXFLAGS) main.cpp -o $@

test: all
	$(PYTHON) tests/test_decoder.py ./build/jpeg_decoder

clean:
	rm -rf build
