# Build with MSYS2 UCRT64's g++ (no CMake / external deps required).
# Usage (from an MSYS2 UCRT64 shell, or see build.bat for plain PowerShell/cmd):
#   mingw32-make            # release build -> bin/physics_engine.exe
#   mingw32-make debug      # build with -g -O0
#   mingw32-make clean

CXX := g++
SRC := $(wildcard src/*.cpp)
OBJDIR := build
BIN := bin/physics_engine.exe

CXXFLAGS := -std=c++17 -Wall -Wextra -Wno-unused-parameter -Isrc
LDFLAGS  := -lopengl32 -lgdi32 -luser32 -lkernel32 -mwindows -static -static-libgcc -static-libstdc++

ifeq ($(MAKECMDGOALS),debug)
CXXFLAGS += -g -O0
else
CXXFLAGS += -O2 -DNDEBUG
endif

OBJS := $(patsubst src/%.cpp,$(OBJDIR)/%.o,$(SRC))

.PHONY: all debug clean run
all: $(BIN)
debug: $(BIN)

$(BIN): $(OBJS)
	@mkdir -p bin
	$(CXX) $(OBJS) -o $@ $(LDFLAGS)

$(OBJDIR)/%.o: src/%.cpp
	@mkdir -p $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

run: all
	./$(BIN)

clean:
	rm -rf $(OBJDIR) bin
