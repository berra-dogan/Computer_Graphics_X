CXX = g++
CXXFLAGS = -O2 -std=c++17

# If you have stb_image and stb_image_write as separate .cpp files, add them here; 
# if they are header-only, no need to compile separately.
# Assuming header-only in this example.

TARGET = color_matching
SRC = color_matching.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC) -lm

clean:
	rm -f $(TARGET)
