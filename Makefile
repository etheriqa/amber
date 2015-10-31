TARGET = ./amber

SOURCES = $(shell find . -name '*.cc')
DEPENDS = $(SOURCES:.cc=.d)
OBJECTS = $(SOURCES:.cc=.o)

CPPFLAGS = -MMD -MP -I.
CXXFLAGS = -Wall -Wextra -std=c++1y -O3 -mavx
LDLIBS = -lpthread

.SUFFIXES: .ppm .png
.PHONY: pt bdpt pm pssmlt clean

$(TARGET): $(OBJECTS)
	$(CXX) $(LDLIBS) $^ -o $@

pt: $(TARGET)
	$(TARGET) --algorithm pt --name pt
	@$(MAKE) pt.png

bdpt: $(TARGET)
	$(TARGET) --algorithm bdpt --name bdpt
	@$(MAKE) bdpt.png

pm: $(TARGET)
	$(TARGET) --algorithm pm --name pm
	@$(MAKE) pm.png

pssmlt: $(TARGET)
	$(TARGET) --algorithm pssmlt --name pssmlt
	@$(MAKE) pssmlt.png

ppm: $(TARGET)
	$(TARGET) --algorithm ppm --name ppm
	@$(MAKE) ppm.png

.ppm.png:
	convert $< $@
	open $@

clean:
	$(RM) $(TARGET)
	$(RM) $(shell find . -maxdepth 1 -name '*.ppm' -o -name '*.hdr' -o -name '*.png')
	$(RM) $(shell find . -name '*.o' -o -name '*.d')

-include $(DEPENDS)
