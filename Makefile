TARGET := Chassis_2026

CMAKE := D:/EdgeAI/yolo_env/Scripts/cmake.exe
OPENOCD := D:/Microsoft VS code/stm32tools/@xpack-dev-tools/openocd/0.12.0-7.1/.content/bin/openocd.exe

.PHONY: all configure build clean download flash

all: build

configure:
	"$(CMAKE)" --preset f103

build: configure
	"$(CMAKE)" --build --preset f103

download: build
	"$(OPENOCD)" -f openocd.cfg -c "program build-cmake/$(TARGET).elf verify reset exit"

flash: download

clean:
	rm -rf build-cmake
