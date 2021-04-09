all:
	rm -f braitenberg_vehicles
	clear
	@echo "Let us build the project"
	g++ -Wall *.cpp \
	    -o braitenberg_vehicles \
		`pkg-config --cflags --libs opencv4`
	@echo CODE COMPILED SUCCESSFULLY!