# Linux (default)
EXE = p1
CPPFLAGS = -std=c++11
LDFLAGS = -lGL -lGLU -lglut

# Windows (cygwin)
ifeq "$(OS)" "Windows_NT"
	EXE = $(EXE).exe
	LDFLAGS = -lopengl32 -lglu32 -lglut32
endif

# OS X
ifeq "$(OSTYPE)" "darwin"
	LDFLAGS = -framework Carbon -framework OpenGL -framework GLUT
endif

$(EXE) : main.cpp ikskel.h
	g++ -o $@ $< $(CPPFLAGS) $(LDFLAGS)
