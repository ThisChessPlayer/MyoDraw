 /*****************************************************************************

                                                         Author: Jason Ma
                                                         Date:   Feb 07 2017
                                      SDLGame

 File Name:     Display.h
 Description:   SDL interface, handles events and displays everything
                appropriately.
 *****************************************************************************/


#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <string>

#ifndef DISPLAY_H
#define DISPLAY_H

class Display{
  public:
  	int init();
  	int load();
  	int handleEvents();
  	void render();
  	void stop();

    SDL_Texture * loadTexture(std::string path);
    
};

#endif /* DISPLAY_H */

