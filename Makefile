PROJECT_PATH=~/Code/line-follower
BOARD=--fqbn arduino:avr:nano
PORT=--port /dev/ttyACM1
MONITOR_FLAGS=--describe \
			  $(PORT) \
			  $(BOARD)
UPLOAD_FLAGS=--verify \
  			 $(PORT) \
  			 --discovery-timeout 3s \
  			 $(BOARD)
DEBUG_FLAGS=--build-property \
			build.extra_flags=-DDEBUG
 
all: build upload

build:
	arduino-cli compile $(BOARD) $(PROJECT_PATH)

debug:
	arduino-cli compile $(DEBUG_FLAGS) $(BOARD) $(PROJECT_PATH)

monitor:
	arduino-cli monitor $(MONITOR_FLAGS)

upload:
	arduino-cli upload $(UPLOAD_FLAGS) $(PROJECT_PATH)

