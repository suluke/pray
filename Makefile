# Sample Makefile for building a pgrep library.
########################################################
#
# All the source files needed to implement the required interface:
SOURCES+=$(wildcard src/*.cpp)

# Build options: Include directories, extra compile flags, lib directories and
# extra libs
INCLUDES+=include/ ext/
EXTRA_CFLAGS+=-O3 -std=c++14 -fPIC -Wall -Werror=vla -Werror -fopenmp
LIBDIRS+=
LIBS+=-lstdc++fs -fopenmp

########################################################
# You shouldn't need to modify anything beyond this point
NAME?=pray
OUT?=out/
BUILD_DIR?=$(OUT)/obj/
TARGETS?=$(addprefix $(OUT)/,$(NAME))

LDFLAGS=$(LIBDIRS) $(LIBS)
CFLAGS=$(EXTRA_CFLAGS) $(addprefix -I,$(INCLUDES))
OBJS=$(addprefix $(BUILD_DIR)/,$(SOURCES:.cpp=.o))
DEPS=$(OBJS:.o=.d)
OBJ_DIRS=$(dir $(OBJS))

.PRECIOUS: $(OBJS) $(DEPS)
.PHONY: all clean
all: $(TARGETS)
clean:
	rm $(TARGETS) $(OBJS) $(DEPS)

$(OUT) $(OBJ_DIRS):
	mkdir -p $@

-include $(DEPS)

$(BUILD_DIR)/%.o: %.cpp $(OBJ_DIRS) Makefile
	$(CXX) $(CFLAGS) -c -o $@ -MMD -MT $(@:.o=.d) $(filter %.cpp, $^) > $(@:.o=.d)

$(OUT)/$(NAME): $(OBJS)  $(OUT) Makefile
	$(CXX) -o $@ $(filter %.o,$^) $(LDFLAGS) $(LIBS)
