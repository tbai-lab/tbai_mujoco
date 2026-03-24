// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Copyright Drew Noakes 2013-2016

#ifndef __JOYSTICK_H__
#define __JOYSTICK_H__

#include <iostream>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sstream>
#include <map>
#include "unistd.h"


#define JS_EVENT_BUTTON 0x01 // button pressed/released
#define JS_EVENT_AXIS 0x02   // joystick moved
#define JS_EVENT_INIT 0x80   // initial state of device

class JoystickEvent
{
public:
  static const short MIN_AXES_VALUE = -32768;
  static const short MAX_AXES_VALUE = 32767;

  unsigned int time;
  short value;
  unsigned char type;
  unsigned char number;

  bool isButton()
  {
    return (type & JS_EVENT_BUTTON) != 0;
  }

  bool isAxis()
  {
    return (type & JS_EVENT_AXIS) != 0;
  }

  bool isInitialState()
  {
    return (type & JS_EVENT_INIT) != 0;
  }

  friend std::ostream &operator<<(std::ostream &os, const JoystickEvent &e);
};

std::ostream &operator<<(std::ostream &os, const JoystickEvent &e);

class Joystick
{
private:
  void openPath(std::string devicePath, bool blocking = false);
  int _fd;

public:
  ~Joystick();
  Joystick();
  Joystick(int joystickNumber);
  Joystick(std::string devicePath);
  Joystick(Joystick const &) = delete;
  Joystick(Joystick &&) = default;
  Joystick(std::string devicePath, bool blocking);

  bool isFound();
  void getState();

  JoystickEvent event_;
  int button_[20] = {0};
  int axis_[10] = {0};

  bool sample(JoystickEvent *event);
};

#endif
