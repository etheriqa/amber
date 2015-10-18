TARGET = ./amber

SRC_DIR = $(PWD)
SRCS = $(shell find $(SRC_DIR) -name '*.cc')
DEPS = $(SRCS:.cc=.d)
OBJS = $(SRCS:.cc=.o)

CPPFLAGS = -MMD -MP -I$(SRC_DIR)
CXXFLAGS = -Wall -std=c++1y -O3 -mavx
LDLIBS = -lpthread

.PHONY: pt bdpt clean

$(TARGET): $(OBJS)
	$(CXX) $(LDLIBS) $^ -o $@

pt: $(TARGET)
	$(TARGET) --algorithm pt
	convert output.ppm pt.png
	open pt.png

bdpt: $(TARGET)
	$(TARGET) --algorithm bdpt
	convert output.ppm bdpt.png
	open bdpt.png


clean:
	$(RM) $(TARGET) $(shell find $(SRC_DIR) -name '*.o') $(shell find $(SRC_DIR) -name '*.d')

-include $(DEPS)
