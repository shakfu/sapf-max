
.phony: all build clean test


all: build


build:
	@mkdir -p build && cd build && \
		cmake .. && \
		cmake --build . --config Release

test: build
	@cd build && ctest --output-on-failure

clean:
	@rm -rf build
