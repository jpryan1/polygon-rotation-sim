 
CPPFLAGS = -std=c++11 -O2 -MMD
LDFLAGS = `pkg-config --static --libs gl glu glew glfw3`
N=55

SRCS = src/forcevec.cpp src/Collision.cpp src/circle.cpp src/Poly.cpp src/animation.cpp src/PolygonSim.cpp src/polygon.cpp src/next-event.cpp src/process-event.cpp
OBJS = build/forcevec.o build/Collision.o build/circle.o build/Poly.o build/animation.o build/PolygonSim.o build/lib/glew.o build/polygon.o build/next-event.o build/process-event.o



all: PolySim

build/lib/glew.o: glew.c
	gcc -c $< -o $@ $(LDFLAGS)

build/%.o: src/%.cpp 
	g++ $(CPPFLAGS) -c -o $@ $<


PolySim: ${OBJS}
	g++ $(CPPFLAGS) $^ -o $@ $(LDFLAGS)



.PHONY: clean

clean:
	rm build/*.o build/*.d

animate:
	./PolySim -a
run:
	./PolySim 



-include $(OBJS:.o=.d)