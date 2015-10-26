TARGET = ./amber

SOURCES = $(shell find . -name '*.cc')
DEPENDS = $(SOURCES:.cc=.d)
OBJECTS = $(SOURCES:.cc=.o)

CPPFLAGS = -MMD -MP -I.
CXXFLAGS = -Wall -Wextra -std=c++1y -O3 -mavx
LDLIBS = -lpthread

.PHONY: pt bdpt pm pssmlt convert clean

$(TARGET): $(OBJECTS)
	$(CXX) $(LDLIBS) $^ -o $@

pt: $(TARGET)
	$(TARGET) --algorithm pt
	@$(MAKE) pt.png
	open pt.png

bdpt: $(TARGET)
	$(TARGET) --algorithm bdpt
	@$(MAKE) bdpt.png
	open bdpt.png

pm: $(TARGET)
	$(TARGET) --algorithm pm
	@$(MAKE) pm.png
	open pm.png

pssmlt: $(TARGET)
	$(TARGET) --algorithm pssmlt
	@$(MAKE) pssmlt.png
	open pssmlt.png

%.png: output.ppm
	convert output.ppm $@

clean:
	$(RM) $(TARGET)
	$(RM) $(shell find . -maxdepth 1 -name '*.ppm' -o -name '*.hdr' -o -name '*.png')
	$(RM) $(shell find . -name '*.o' -o -name '*.d')

-include $(DEPENDS)
