TARGET = ./amber

SOURCES = $(shell find . -name '*.cc')
DEPENDS = $(SOURCES:.cc=.d)
OBJECTS = $(SOURCES:.cc=.o)

CPPFLAGS = -MMD -MP -I.
CXXFLAGS = -Wall -Wextra -std=c++1y -O3 -mavx
LDLIBS = -lpthread

.PHONY: pt bdpt clean

$(TARGET): $(OBJECTS)
	$(CXX) $(LDLIBS) $^ -o $@

pt: $(TARGET)
	$(TARGET) --algorithm pt
	convert output.ppm pt.png
	open pt.png

bdpt: $(TARGET)
	$(TARGET) --algorithm bdpt
	convert output.ppm bdpt.png
	open bdpt.png

pm: $(TARGET)
	$(TARGET) --algorithm pm
	convert output.ppm pm.png
	open pm.png

clean:
	$(RM) $(TARGET)
	$(RM) $(shell find . -maxdepth 1 -name '*.ppm' -o -name '*.hdr' -o -name '*.png')
	$(RM) $(shell find . -name '*.o' -o -name '*.d')

-include $(DEPENDS)
