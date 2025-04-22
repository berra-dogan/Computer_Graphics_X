# Makefile with required SRC, default OUT, printed INCLUDES, handling spaces in paths, and a debug option

# Check if SRC is provided
ifeq ($(SRC),)
  $(error SRC is required. Please specify the source file, e.g., make SRC=trial.cpp)
endif

# Compiler and flags
CC = /opt/homebrew/opt/llvm/bin/clang
CFLAGS = -std=c++17

# Debugging flags, if DEBUG is set
ifeq ($(DEBUG),1)
  CFLAGS += -g  # Add the -g flag to enable debugging symbols
  $(info Debugging enabled)
else
  $(info Debugging disabled)
endif

# Set PROJECT_DIR to the current working directory, properly quoted to handle spaces
PROJECT_DIR = "$(CURDIR)"

# Set INCLUDES and print it, properly quoting to handle spaces
INCLUDES ?= -I$(PROJECT_DIR)/stb/ -I$(PROJECT_DIR)/headers/

# Set OUT to be the basename of SRC (remove the file extension)
OUT ?= $(basename $(SRC))

CFLAGS += -I/opt/homebrew/opt/libomp/include -Xpreprocessor -fopenmp -stdlib=libc++
LDFLAGS = -L/opt/homebrew/opt/libomp/lib -lomp -lc++ -Wall -Wextra -Wunused

all:
	$(CC) $(CFLAGS) $(INCLUDES) $(SRC) -o $(OUT) $(LDFLAGS)

# Clean the output
clean:
	rm -f $(OUT)
