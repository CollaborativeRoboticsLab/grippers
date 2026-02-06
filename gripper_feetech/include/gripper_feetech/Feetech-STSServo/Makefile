CXX := g++
CXXFLAGS := -std=c++17 -O2
LDLIBS := -lpthread

# All example binaries in this repo
BINARIES := \
  simple_motion \
  continous_motion \
  change_servo_id \
  move_synchronously \
  velocity_mode \
  follow_servo \
  move_to_home \
  step_mode

.PHONY: all clean

# Build all examples
all: $(BINARIES)

simple_motion: simple_motion.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDLIBS)

continous_motion: continous_motion.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDLIBS)

change_servo_id: change_servo_id.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDLIBS)

move_synchronously: move_synchronously.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDLIBS)

velocity_mode: velocity_mode.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDLIBS)

follow_servo: follow_servo.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDLIBS)

move_to_home: move_to_home.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDLIBS)

step_mode: step_mode.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDLIBS)

# Remove all built binaries and common build artefacts
clean:
	rm -f $(BINARIES) *.o *.out *.log
