CXX ?= g++
CXXFLAGS ?= -O3 -std=c++17 -Wall -Wextra -pthread
LDFLAGS ?= -lncursesw -lpthread

TARGET = rk_tester

all: $(TARGET)

$(TARGET): rk_tester.cpp
	$(CXX) $(CXXFLAGS) rk_tester.cpp -o $(TARGET) $(LDFLAGS)

clean:
	rm -f $(TARGET)

install: $(TARGET)
	install -m 755 $(TARGET) /usr/local/bin/
	rm -f /usr/local/bin/rk_stress_tui

.PHONY: all clean install
