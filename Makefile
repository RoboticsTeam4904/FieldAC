CXX = g++
OPENCV = $(shell pkg-config --cflags --libs opencv)
CXXFLAGS = -std=c++11 -Wall
LDFLAGS = $(OPENCV)

SRC_DIRS = $(shell find . -name "*.cpp" -exec dirname \; | uniq)
SRCS = $(shell find . -name "*.cpp")
OBJ_DIR = build
OBJS = $(patsubst %.cpp,$(OBJ_DIR)/%.o,$(SRCS:./%=%))
BIN_DIR = bin

EXEC = field

.PHONY: all clean distclean

all: $(BIN_DIR)/$(EXEC)

$(BIN_DIR)/$(EXEC): buildrepo $(OBJS)
	@mkdir -p `dirname $@`
	@echo "Linking $@...`
	$(CXX) $(OBJS) -o $@

$(OBJ_DIR)/%.o: %.cpp
	@echo "Generating dependencies for $<..."
	$(call make-depend,$<,$@,$(subst .o,.d,$@))
	@echo "Compiling $<..."
	@$(CXX) $(CXXFLAGS) $(LDFLAGS) $< -o $@

clean:
	$(RM) -r $(OBJ_DIR)

distclean: clean
	$(RM) -r $(BIN_DIR)

buildrepo:
	@$(call make-repo)

define make-repo
   for dir in $(SRCDIRS); \
   do \
	mkdir -p $(OBJDIR)/$$dir; \
   done
endef

# usage: $(call make-depend,source-file,object-file,depend-file)
define make-depend
  $(CXX) -MM       \
        -MF $3    \
        -MP       \
        -MT $2    \
        $(CXXFLAGS) \
        $1
endef