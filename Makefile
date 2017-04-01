ifeq ($(OS),Windows_NT)
	LINKER_FLAGS = -lmingw32 -lSDL2main -lSDL2 -lSDL2_image -lmyo32
	COMPILER_FLAGS = -std=c++11 -Wall
	INCLUDE_PATHS = -I.\SDL2\include -I.\SDL_image\include -I.\myoSDK\include
	LIBRARY_PATHS = -L.\SDL2\lib -L.\SDL_image\lib -L.\myoSDK\lib

	RM = del /Q
	FixPath = $(subst /,\,$1)
else
	LINKER_FLAGS = -lSDL2 -lSDL2_image
	COMPILER_FLAGS = -std=c++11 -Wall
	#INCLUDE_PATHS = -I./SDL2/include -I./SDL_image/include
	#LIBRARY_PATHS = -L./SDL2/lib -L./SDL_image/lib

	RM = rm -f
	FixPath = $1
endif

OBJS = Display.cpp

OBJ_NAME = myoSign

all : $(OBJS)
	g++ $(OBJS) $(COMPILER_FLAGS) $(INCLUDE_PATHS) $(LIBRARY_PATHS) $(LINKER_FLAGS) -o $(OBJ_NAME)

clean:
	$(RM) sdlGame.exe
