all:
	rm -f novelty_detection
	clear
	@echo "Let us build the project novelty_detection"
	g++ -Wall *.cpp \
	    -o novelty_detection \
		`pkg-config --cflags --libs opencv4`
	@echo CODE COMPILED SUCCESSFULLY!