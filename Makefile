CXX       ?= g++
CXXFLAGS  ?= -O3 -std=c++17 -Wall -Wextra -pthread
CPPFLAGS  += -Isrc -MMD -MP
LDFLAGS   ?= -lncursesw -lpthread -ldl

TARGET    := rk_tester
BUILD_DIR := build
SRCS      := $(wildcard src/*.cpp)
OBJS      := $(patsubst src/%.cpp,$(BUILD_DIR)/%.o,$(SRCS))
DEPS      := $(OBJS:.o=.d)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) -pthread $(OBJS) -o $@ $(LDFLAGS)

$(BUILD_DIR)/%.o: src/%.cpp | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR) $(TARGET)

install: $(TARGET)
	rm -f /usr/local/bin/$(TARGET)
	install -m 755 $(TARGET) /usr/local/bin/
	rm -f /usr/local/bin/rk_stress_tui

-include $(DEPS)

.PHONY: all clean install
