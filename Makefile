TARGET = amber

SRC_DIR = $(PWD)
SRCS = $(shell find $(SRC_DIR) -name '*.cc')
DEPS = $(SRCS:.cc=.d)
OBJS = $(SRCS:.cc=.o)

CPPFLAGS = -MMD -MP -I$(SRC_DIR)
CXXFLAGS = -Wall -std=c++1y -O3

.PHONY: run clean

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $^ -o $@

run: $(TARGET)
	./$(TARGET)

clean:
	$(RM) $(TARGET) $(shell find $(SRC_DIR) -name '*.o') $(shell find $(SRC_DIR) -name '*.d')

-include $(DEPS)
