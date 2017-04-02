 /*****************************************************************************

                                                         Author: Jason Ma
                                                         Date:   Apr 01 2017
                                      MyoDraw

 File Name:     Display.cpp
 Description:   SDL interface, handles events and displays everything
                appropriately.
 *****************************************************************************/

#include "SDL2/include/SDL2/SDL.h"
#include "SDL_image/include/SDL2/SDL_image.h"
#include "Display.h"

#include <iostream>
#include <cstdio>

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <myo/myo.hpp>

using std::cout;
using std::endl;

const int SCREEN_WIDTH = 1280;
const int SCREEN_HEIGHT = 720;

const int POSE_FIST = 0;
const int POSE_TAP = 1;
const int POSE_SPREAD = 2;
const int POSE_OTHER = 3;

const int X_SENS = 5;
const int Y_SENS = 3;

SDL_Window * window = NULL; //window to render to
SDL_Surface * screenSurface = NULL; //surface contained by window
SDL_Surface * drawSurface = NULL;

SDL_Renderer * renderer = NULL;
SDL_Texture * mouseTexture;
SDL_Texture * drawTexture;
SDL_Event event;
SDL_Rect mouseRect;
SDL_Rect tile;
SDL_Rect downRect;
SDL_Rect upRect;
SDL_Rect pointerRect;
bool mouseDown = false;
bool calibrate = false;

bool xInvert = false;
bool yInvert = false;

// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.
class DataCollector : public myo::DeviceListener {
  public:
    DataCollector()
    : onArm(false), isUnlocked(false), roll_w(0), pitch_w(0), yaw_w(0), currentPose() {}

    void onPair(myo::Myo * myo, uint64_t timestamp) {
      printf("Myo Paired");
    }

    // onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
    void onUnpair(myo::Myo* myo, uint64_t timestamp) {
      // We've lost a Myo.
      // Let's clean up some leftover state.
      roll_w = 0;
      pitch_w = 0;
      yaw_w = 0;
      onArm = false;
      isUnlocked = false;
    }
    // onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
    // as a unit quaternion.
    void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat) {
      using std::atan2;
      using std::asin;
      using std::sqrt;
      using std::max;
      using std::min;
      // Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
      float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
                         1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
      float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
      float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                      1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));
      // Convert the floating point angles in radians to a scale from 0 to 1800.
      roll_w = static_cast<int>((roll + (float)M_PI)/(M_PI * 2.0f) * 1800);
      pitch_w = static_cast<int>((pitch + (float)M_PI/2.0f)/M_PI * 1800);
      yaw_w = static_cast<int>((yaw + (float)M_PI)/(M_PI * 2.0f) * 1800);
      //if(yaw_w < 0) yaw_w += 1800;
    }
    // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
    // making a fist, or not making a fist anymore.
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose) {

      if(pose == myo::Pose::fist) {
        printf("\nFist pose\n");

      } else if(currentPose == myo::Pose::fist) {
        printf("\nFist stop\n");

      }
      else if(pose == myo::Pose::doubleTap) {
        zPitch = pitch_w - 900;
        zYaw = yaw_w - 900;

      }
      else {
      }
      currentPose = pose;

    }
    // onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
    // arm. This lets Myo know which arm it's on and which way it's facing.
    void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
                   myo::WarmupState warmupState) {
      printf("\nArm sync successful.\n");
      onArm = true;
      whichArm = arm;
    }
    // onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
    // it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
    // when Myo is moved around on the arm.
    void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
    {
      printf("\nArm unsynced.\n");
      onArm = false;
    }
    // onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
    void onUnlock(myo::Myo* myo, uint64_t timestamp)
    {
      printf("\nUnlocked.\n");
      isUnlocked = true;
    }
    // onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
    void onLock(myo::Myo* myo, uint64_t timestamp)
    {
      printf("\nLocked.\n");
      isUnlocked = false;
    }
    // There are other virtual functions in DeviceListener that we could override here, like onAccelerometerData().
    // For this example, the functions overridden above are sufficient.
    // We define this function to print the current values that were updated by the on...() functions above.
    void print()
    {
      cout << roll_w - zRoll << " " << pitch_w - zPitch << " " << yaw_w - zYaw << endl;
    }

    int getPose() {
      if(currentPose == myo::Pose::fist)
        return POSE_FIST;
      else if(currentPose == myo::Pose::doubleTap)
        return POSE_TAP;
      else if(currentPose == myo::Pose::fingersSpread)
        return POSE_SPREAD;
      else
        return POSE_OTHER;
    }

    int getRoll() {
      return roll_w - zRoll;
    }

    int getPitch() {
      return pitch_w - zPitch;
    }

    int getYaw() {
      return yaw_w - zYaw;
    }

    // These values are set by onArmSync() and onArmUnsync() above.
    bool onArm;
    myo::Arm whichArm;
    // This is set by onUnlocked() and onLocked() above.
    bool isUnlocked;
    // These values are set by onOrientationData() and onPose() above.
    int roll_w, pitch_w, yaw_w;

    int zRoll = 0;
    int zPitch = pitch_w - 900;
    int zYaw = yaw_w - 900;
    myo::Pose currentPose;
};

int Display::init() {

  //attempt to init SDL
  if(SDL_Init(SDL_INIT_VIDEO) < 0) {
    printf("SDL init failed! SDL_Error: %s\n", SDL_GetError());
    return -1;
  }
 
  //attempt to create window  
  window = SDL_CreateWindow("MyoDraw", 
      SDL_WINDOWPOS_UNDEFINED, 
      SDL_WINDOWPOS_UNDEFINED, 
      SCREEN_WIDTH, 
      SCREEN_HEIGHT, 
      SDL_WINDOW_SHOWN);

  if(window == NULL) {
    printf("Window creation failed! SDL_Error: %s\n", SDL_GetError());
    return -1;
  }

  renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

  if(renderer == NULL) {
    printf("Renderer could not be created! SDL Error: %s\n", SDL_GetError());
    return -1;
  }

  int imgFlags = IMG_INIT_PNG;

  if(!(IMG_Init(imgFlags) & imgFlags)) {
    printf("SDL_image could not initialize! SDL_image Error: %s\n", IMG_GetError());
  }

  screenSurface = SDL_GetWindowSurface(window);

  //drawSurface = SDL_GetWindowSurface(window);
  drawSurface = SDL_CreateRGBSurface(0, SCREEN_WIDTH, SCREEN_HEIGHT, 32, 0, 0, 0, 0);

  SDL_FillRect(screenSurface, NULL, 
      SDL_MapRGB(screenSurface->format, 0x00, 0x00, 0x00));
  
  SDL_FillRect(drawSurface, NULL, 
      SDL_MapRGB(drawSurface->format, 0x00, 0x00, 0x00));
  
  SDL_UpdateWindowSurface(window);

  SDL_ShowCursor(SDL_DISABLE);

  return 0;
}

int Display::load() {
  mouseTexture = loadTexture("crosshair16.png");
  //TODO actually check whether texture was loaded, unloaded -> NULL
  return 0;
}

SDL_Texture * Display::loadTexture(std::string path) {
  SDL_Texture * result = NULL;

  SDL_Surface * loadedSurface = IMG_Load(path.c_str());
  if(loadedSurface == NULL) {
    printf("Unable to load image %s! SDL_image Error: %s\n", path.c_str(), IMG_GetError());
    return result;
  }

  result = SDL_CreateTextureFromSurface(renderer, loadedSurface);

  if(result == NULL) {
    printf("Unable to create texture from %s! SDL Error: %s\n", path.c_str(), SDL_GetError());
    return result;
  }

  SDL_FreeSurface(loadedSurface);

  return result;
}

int Display::handleEvents() {
  int x, y;

  while(SDL_PollEvent(&event)) {
    switch(event.type) {
      case SDL_QUIT:
        return -1;
      case SDL_KEYDOWN:
        switch(event.key.keysym.sym) {
          case SDLK_q:
            return -1;
          case SDLK_x:
            if(xInvert) xInvert = false;
            else xInvert = true;
            break;
          case SDLK_y:
            if(yInvert) yInvert = false;
            else yInvert = true;
            break;
        }
        break;
      /*
      case SDL_MOUSEBUTTONDOWN:
        mouseDown = true;
        SDL_GetMouseState(&x, &y);
        downRect = {x - (x % 50), y - (y % 50), 50, 50};
        if(x / 50 == 0 && y / 50 == 0) {
          calibrate = true;
        }
        break;
      case SDL_MOUSEBUTTONUP:
        mouseDown = false;
        SDL_GetMouseState(&x, &y);
        upRect = {x - (x % 50), y - (y % 50), 50, 50};
        break;
      */
      case SDL_MOUSEMOTION:

        //get mouse position, draw rect
        SDL_GetMouseState(&x, &y);
        mouseRect = {x - 8, y - 8, 16, 16};

        //cout << x << " " << y << endl;
        break;
    }
  }
  return 0;
}

void Display::render() {

  //clear screen
  SDL_SetRenderDrawColor(renderer, 0x00, 0x00, 0x00, 0xFF);
  SDL_RenderClear(renderer);

  //copy the drawing to the screen
  drawTexture = SDL_CreateTextureFromSurface(renderer, drawSurface);
  SDL_RenderCopy(renderer, drawTexture, NULL, NULL);
  SDL_DestroyTexture(drawTexture);

  //render crosshair
  SDL_RenderCopy(renderer, mouseTexture, NULL, &mouseRect);

  //render another crosshair
  SDL_RenderCopy(renderer, mouseTexture, NULL, &pointerRect);

  //show frame
  SDL_RenderPresent(renderer);
  //SDL_UpdateWindowSurface(window);
}

void Display::stop() {
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}

int main(int argc, char * argv[]) {

  //init Myo
  // We catch any exceptions that might occur below -- see the catch statement for more details.
  try {
    // First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
    // publishing your application. The Hub provides access to one or more Myos.
    myo::Hub hub("com.example.myoSign");
    //hub.setLockingPolicy(libmyo_locking_policy_none);
    hub.setLockingPolicy(myo::Hub::lockingPolicyNone);

    std::cout << "Attempting to find a Myo..." << std::endl;
    // Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
    // immediately.
    // waitForMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
    // if that fails, the function will return a null pointer.
    myo::Myo* myo = hub.waitForMyo(10000);
    // If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
    if (!myo) {
        throw std::runtime_error("Unable to find a Myo!");
    }
    // We've found a Myo.
    std::cout << "Connected to a Myo armband!" << std::endl << std::endl;
    // Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
    DataCollector collector;
    // Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
    // Hub::run() to send events to all registered device listeners.
    hub.addListener(&collector);

  Display disp;
  if(disp.init()) {
    cout << "Init Error" << endl;
    return -1;
  }

  if(disp.load()) {
    cout << "Load Error" << endl;
    return -1;
  }

  int quit = 0;
  int frames = 0;

  int i = 255;
  int j = 0;
  int k = 0;

  int lastX, lastY;

  mouseRect = {0, 0, 10, 10};
  unsigned int begin = SDL_GetTicks();

  bool firstFist;


  //main loop
  while(!quit) {
    //fps counter
    if(SDL_GetTicks() - begin > 1000) {
      begin = SDL_GetTicks();
      cout << "FPS: " << frames << endl;
      frames = 0;
    }

    //check for end loop
    if(disp.handleEvents() == -1)
      break;

    //handle myo events
    hub.run(1);

    collector.print();

    //calculate myo position
    int x;
    if(xInvert)
      x = (collector.getYaw() - 900) * X_SENS * SCREEN_WIDTH / 1800.0 + 900 * SCREEN_WIDTH / 1800.0;
    else
      x = (900 - collector.getYaw()) * X_SENS * SCREEN_WIDTH / 1800.0 + 900 * SCREEN_WIDTH / 1800.0;
    int y;

    if(yInvert)
      y = (collector.getPitch() - 900) * Y_SENS * SCREEN_HEIGHT / 1800.0 + 900 * SCREEN_HEIGHT / 1800.0;
    else
      y = (900 - collector.getPitch()) * Y_SENS * SCREEN_HEIGHT / 1800.0 + 900 * SCREEN_HEIGHT / 1800.0;
    
    //cout << x << " " << y << endl;
    //collector.print();

    //get myo pose
    int pose = collector.getPose();

    SDL_Rect rect2 = {x, y, (collector.getRoll() - 300) / 200, (collector.getRoll() - 300) / 200};
    //SDL_Rect rect3 = {0, 0, SCREEN_WIDTH, SCREEN_HEIGHT};

    switch(pose) {
      case POSE_FIST:

        if(firstFist) {
          lastX = x;
          lastY = y;
          firstFist = false;
        }
        //draw to screen
        while(rect2.x != lastX || rect2.y != lastY) {
          SDL_FillRect(drawSurface, &rect2, SDL_MapRGB(drawSurface->format, i, j, k));
          if(rect2.x < lastX) rect2.x += 1;
          else if(rect2.x > lastX) rect2.x -= 1;

          if(rect2.y < lastY) rect2.y += 1;
          else if(rect2.y > lastY) rect2.y -= 1;
        }


        if(i == 255 && j < 255 && k == 0) {
          j++;
        }
        else if(i > 0 && j == 255) {
          i--;
        }
        else if(j == 255 && k < 255) {
          k++;
        }
        else if(j > 0 && k == 255) {
          j--;
        }
        else if(k == 255 && i < 255) {
          i++;
        }
        else {
          k--;
        }

        lastX = x;
        lastY = y;

        break;
      case POSE_SPREAD:
        //clear drawings
        SDL_FillRect(drawSurface, NULL, SDL_MapRGB(drawSurface->format, 0, 0, 0));
        break;
      case POSE_TAP:
        lastX = x;
        lastY = y;
        break;
      case POSE_OTHER:
        firstFist = true;
        break;
    }

    pointerRect = {x - 8, y - 8, 16, 16};
    disp.render();
    frames++;
  }

  disp.stop();

  //TODO clear change
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    std::cerr << "Press enter to continue.";
    std::cin.ignore();
    return 1;
  }

  return 0; 
}