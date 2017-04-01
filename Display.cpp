 /*****************************************************************************

                                                         Author: Jason Ma
                                                         Date:   Apr 01 2017
                                      MyoSign

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
// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo/myo.hpp>

using std::cout;
using std::endl;

const int SCREEN_WIDTH = 1280;
const int SCREEN_HEIGHT = 720;

SDL_Window * window = NULL; //window to render to
SDL_Surface * screenSurface = NULL; //surface contained by window
SDL_Renderer * renderer = NULL;
SDL_Texture * mouseTexture;
SDL_Event event;
SDL_Rect mouseRect;
SDL_Rect tile;
SDL_Rect downRect;
SDL_Rect upRect;
bool mouseDown = false;

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
      // Convert the floating point angles in radians to a scale from 0 to 18.
      roll_w = static_cast<int>((roll + (float)M_PI)/(M_PI * 2.0f) * 18);
      pitch_w = static_cast<int>((pitch + (float)M_PI/2.0f)/M_PI * 18);
      yaw_w = static_cast<int>((yaw + (float)M_PI)/(M_PI * 2.0f) * 18);
    }
    // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
    // making a fist, or not making a fist anymore.
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose) {
      currentPose = pose;

      if(pose == myo::Pose::fist) {
        printf("\nFist pose\n");
      }
      if (pose != myo::Pose::unknown && pose != myo::Pose::rest) {
        // Tell the Myo to stay unlocked until told otherwise. We do that here so you can hold the poses without the
        // Myo becoming locked.
        //myo->unlock(myo::Myo::unlockHold);
        // Notify the Myo that the pose has resulted in an action, in this case changing
        // the text on the screen. The Myo will vibrate.
        //myo->notifyUserAction();
      } else {
        printf("\nFist stop\n");

        // Tell the Myo to stay unlocked only for a short period. This allows the Myo to stay unlocked while poses
        // are being performed, but lock after inactivity.
        //myo->unlock(myo::Myo::unlockTimed);
      }
      myo->unlock(myo::Myo::unlockHold);
        myo->notifyUserAction();

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
      // Clear the current line
      std::cout << '\r';
      // Print out the orientation. Orientation data is always available, even if no arm is currently recognized.
      std::cout << '[' << std::string(roll_w, '*') << std::string(18 - roll_w, ' ') << ']'
                << '[' << std::string(pitch_w, '*') << std::string(18 - pitch_w, ' ') << ']'
                << '[' << std::string(yaw_w, '*') << std::string(18 - yaw_w, ' ') << ']';
      if (onArm) {
        // Print out the lock state, the currently recognized pose, and which arm Myo is being worn on.
        // Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
        // output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
        // that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().
        std::string poseString = currentPose.toString();
        std::cout << '[' << (isUnlocked ? "unlocked" : "locked  ") << ']'
                  << '[' << (whichArm == myo::armLeft ? "L" : "R") << ']'
                  << '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';
      } else {
        // Print out a placeholder for the arm and pose when Myo doesn't currently know which arm it's on.
        std::cout << '[' << std::string(8, ' ') << ']' << "[?]" << '[' << std::string(14, ' ') << ']';
      }
      std::cout << std::flush;
    }

    bool getfist() {
      if(currentPose == myo::Pose::fist) {
        return true;
      }
      else
        return false;
    }

    // These values are set by onArmSync() and onArmUnsync() above.
    bool onArm;
    myo::Arm whichArm;
    // This is set by onUnlocked() and onLocked() above.
    bool isUnlocked;
    // These values are set by onOrientationData() and onPose() above.
    int roll_w, pitch_w, yaw_w;
    myo::Pose currentPose;
};

int Display::init() {

  //attempt to init SDL
  if(SDL_Init(SDL_INIT_VIDEO) < 0) {
    printf("SDL init failed! SDL_Error: %s\n", SDL_GetError());
    return -1;
  }
 
  //attempt to create window  
  window = SDL_CreateWindow("Cubeception 3 Status Monitor", 
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

  SDL_FillRect(screenSurface, NULL, 
      SDL_MapRGB(screenSurface->format, 0x00, 0x00, 0x00));

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
        }
        break;
      case SDL_MOUSEBUTTONDOWN:
        mouseDown = true;
        SDL_GetMouseState(&x, &y);
        downRect = {x - (x % 50), y - (y % 50), 50, 50};
        break;
      case SDL_MOUSEBUTTONUP:
        mouseDown = false;
        SDL_GetMouseState(&x, &y);
        upRect = {x - (x % 50), y - (y % 50), 50, 50};
        break;
        break;
      case SDL_MOUSEMOTION:

        //get mouse position, draw rect
        SDL_GetMouseState(&x, &y);
        mouseRect = {x - 8, y - 8, 16, 16};

        //cout << x << " " << y << endl;
    }
  }
  return 0;
}

void Display::render() {
  //hub->run(1000);

  //clear screen
  SDL_SetRenderDrawColor(renderer, 0x00, 0x00, 0x00, 0xFF);
  SDL_RenderClear(renderer);

  //draw rect
  /*
  SDL_Rect rect = {400, 400, 400, 400};
  SDL_SetRenderDrawColor(renderer, 0xFF, 0xFF, 0xFF, 0xFF);
  SDL_RenderFillRect(renderer, &rect);
  */

  SDL_SetRenderDrawColor(renderer, 0x00, 0x88, 0x00, 0xFF);

  //render tiles
  for(int i = 0; i < 8; i++)
    for(int j = 0; j < 8; j++) {
      tile.x = i * 50;
      tile.y = j * 50;
      tile.w = 50;
      tile.h = 50;

      if((i + j) % 2 == 0)
        SDL_RenderFillRect(renderer, &tile);
    }

  SDL_SetRenderDrawColor(renderer, 0xFF, 0x00, 0x00, 0xFF);
  SDL_RenderFillRect(renderer, &downRect);

  //render crosshair
  SDL_RenderCopy(renderer, mouseTexture, NULL, &mouseRect);

  //show frame
  SDL_RenderPresent(renderer);

  //cout << x << " " << y << endl;
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
    // Finally we enter our main loop.
    /*
    while (1) {
        // In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
        // In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
        hub.run(1000/20);
        // After processing events, we call the print() member function we defined above to print out the values we've
        // obtained from any events that have occurred.
        collector.print();
    }
    */
    // If a standard exception occurred, we print out its message and exit.


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
  mouseRect = {0, 0, 10, 10};
  unsigned int begin = SDL_GetTicks();
  while(!quit) {
    if(SDL_GetTicks() - begin > 1000) {
      begin = SDL_GetTicks();
      cout << "FPS: " << frames << endl;
      frames = 0;
    }
    if(disp.handleEvents() == -1)
      break;

    //TODO clear change
    hub.run(1);

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