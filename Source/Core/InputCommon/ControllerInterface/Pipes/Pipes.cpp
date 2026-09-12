// Copyright 2015 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "InputCommon/ControllerInterface/Pipes/Pipes.h"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <fcntl.h>
#include <iostream>
#include <locale>
#include <map>
#include <sstream>
#include <string>
#include <unistd.h>
#include <utility>
#include <vector>

#include "Common/FileUtil.h"
#include "Common/StringUtil.h"
#include "InputCommon/ControllerInterface/ControllerInterface.h"

namespace ciface::Pipes
{
static const std::array<std::string, 12> s_button_tokens{
    {"A", "B", "X", "Y", "Z", "START", "L", "R", "D_UP", "D_DOWN", "D_LEFT", "D_RIGHT"}};

static const std::array<std::string, 2> s_shoulder_tokens{{"L", "R"}};

// 2D axis pairs: a stick or a cursor position. Unsigned [0,1] per axis, 0.5 centered --
// the convention the 4-token `SET <name> <x> <y>` form has always used.
// IR is new here: it is the Wii Remote's pointer, which is a POSITION on the screen and so
// reads naturally in the same unsigned space a stick does.
static const std::array<std::string, 3> s_axis_tokens{{"MAIN", "C", "IR"}};

// 3D axis triples: an IMU reading. SIGNED [-1,1] per axis, 0 centered, because an
// accelerometer and a gyroscope are signed by nature and pinning them to a 0.5 rest point
// would put the neutral value in a different place than every consumer expects.
//
// These exist so an emulated Wii Remote can be driven over a pipe at all. Dolphin's emulated
// Wiimote binds motion through the IMUAccelerometer (Up/Down/Left/Right/Forward/Backward) and
// IMUGyroscope (Pitch/Roll/Yaw x2) control groups, which is 6 analog inputs; add the 2 for the
// IR pointer and a Wii Remote needs 8 axes. Before this, PipeDevice offered 6 in total (MAIN
// X/Y, C X/Y, L, R) and every one of them was already spoken for by a GameCube pad, so a
// Wiimote simply could not be expressed -- not approximately, at all.
//
// Magnitude is deliberately NOT handled here. A real swing is several g and several hundred
// degrees/second, both far outside [-1,1], but SetAxis() clamps to that range and widening the
// clamp would change what every existing axis means. The writer sends a NORMALIZED value and
// the binding expression scales it (`` `Axis GYR X +` * 20 ``), which keeps the wire format
// unitless and puts the per-game feel in config where it can be tuned without a rebuild.
static const std::array<std::string, 2> s_triaxis_tokens{{"ACC", "GYR"}};

static double StringToDouble(const std::string& text)
{
  std::istringstream is(text);
  // ignore current locale
  is.imbue(std::locale::classic());
  double result;
  is >> result;
  return result;
}

class InputBackend final : public ciface::InputBackend
{
public:
  using ciface::InputBackend::InputBackend;
  void PopulateDevices() override;
};

std::unique_ptr<ciface::InputBackend> CreateInputBackend(ControllerInterface* controller_interface)
{
  return std::make_unique<InputBackend>(controller_interface);
}

void InputBackend::PopulateDevices()
{
  // Search the Pipes directory for files that we can open in read-only,
  // non-blocking mode. The device name is the virtual name of the file.
  File::FSTEntry fst;
  std::string dir_path = File::GetUserPath(D_PIPES_IDX);
  if (!File::Exists(dir_path))
    return;
  fst = File::ScanDirectoryTree(dir_path, false);
  if (!fst.isDirectory)
    return;
  for (unsigned int i = 0; i < fst.size; ++i)
  {
    const File::FSTEntry& child = fst.children[i];
    if (child.isDirectory)
      continue;
    int fd = open(child.physicalName.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0)
      continue;
    g_controller_interface.AddDevice(std::make_shared<PipeDevice>(fd, child.virtualName));
  }
}

PipeDevice::PipeDevice(int fd, std::string name) : m_fd(fd), m_name(std::move(name))
{
  for (const auto& tok : s_button_tokens)
  {
    PipeInput* btn = new PipeInput("Button " + tok);
    AddInput(btn);
    m_buttons[tok] = btn;
  }
  for (const auto& tok : s_shoulder_tokens)
  {
    AddAxis(tok, 0.0);
  }
  for (const auto& tok : s_axis_tokens)
  {
    AddAxis(tok + " X", 0.5);
    AddAxis(tok + " Y", 0.5);
  }
  for (const auto& tok : s_triaxis_tokens)
  {
    // 0.5 is the resting state in AddAxis()' own unsigned space, i.e. a signed zero --
    // the same neutral the 2D axes start at.
    AddAxis(tok + " X", 0.5);
    AddAxis(tok + " Y", 0.5);
    AddAxis(tok + " Z", 0.5);
  }
}

PipeDevice::~PipeDevice()
{
  close(m_fd);
}

Core::DeviceRemoval PipeDevice::UpdateInput()
{
  // Read any pending characters off the pipe. If we hit a newline,
  // then dequeue a command off the front of m_buf and parse it.
  char buf[32];
  ssize_t bytes_read = read(m_fd, buf, sizeof buf);
  while (bytes_read > 0)
  {
    m_buf.append(buf, bytes_read);
    bytes_read = read(m_fd, buf, sizeof buf);
  }
  std::size_t newline = m_buf.find("\n");
  while (newline != std::string::npos)
  {
    std::string command = m_buf.substr(0, newline);
    ParseCommand(command);
    m_buf.erase(0, newline + 1);
    newline = m_buf.find("\n");
  }
  return Core::DeviceRemoval::Keep;
}

void PipeDevice::AddAxis(const std::string& name, double value)
{
  // Dolphin uses separate axes for left/right, which complicates things.
  PipeInput* ax_hi = new PipeInput("Axis " + name + " +");
  ax_hi->SetState(value);
  PipeInput* ax_lo = new PipeInput("Axis " + name + " -");
  ax_lo->SetState(value);
  m_axes[name + " +"] = ax_hi;
  m_axes[name + " -"] = ax_lo;
  AddFullAnalogSurfaceInputs(ax_lo, ax_hi);
}

void PipeDevice::SetAxis(const std::string& entry, double value)
{
  value = std::clamp(value, 0.0, 1.0);
  double hi = std::max(0.0, value - 0.5) * 2.0;
  double lo = (0.5 - std::min(0.5, value)) * 2.0;
  auto search_hi = m_axes.find(entry + " +");
  if (search_hi != m_axes.end())
    search_hi->second->SetState(hi);
  auto search_lo = m_axes.find(entry + " -");
  if (search_lo != m_axes.end())
    search_lo->second->SetState(lo);
}

void PipeDevice::ParseCommand(const std::string& command)
{
  const std::vector<std::string> tokens = SplitString(command, ' ');
  if (tokens.size() < 2 || tokens.size() > 5)
    return;
  if (tokens[0] == "PRESS" || tokens[0] == "RELEASE")
  {
    auto search = m_buttons.find(tokens[1]);
    if (search != m_buttons.end())
      search->second->SetState(tokens[0] == "PRESS" ? 1.0 : 0.0);
  }
  else if (tokens[0] == "SET")
  {
    if (tokens.size() == 3)
    {
      double value = StringToDouble(tokens[2]);
      SetAxis(tokens[1], (value / 2.0) + 0.5);
    }
    else if (tokens.size() == 4)
    {
      double x = StringToDouble(tokens[2]);
      double y = StringToDouble(tokens[3]);
      SetAxis(tokens[1] + " X", x);
      SetAxis(tokens[1] + " Y", y);
    }
    else if (tokens.size() == 5)
    {
      // An IMU triple. Signed [-1,1] on the wire, mapped into SetAxis()' unsigned space the
      // same way the single-value shoulder form above does it -- so 0.0 lands at 0.5, which
      // is the rest value the constructor gave these axes.
      double x = StringToDouble(tokens[2]);
      double y = StringToDouble(tokens[3]);
      double z = StringToDouble(tokens[4]);
      SetAxis(tokens[1] + " X", (x / 2.0) + 0.5);
      SetAxis(tokens[1] + " Y", (y / 2.0) + 0.5);
      SetAxis(tokens[1] + " Z", (z / 2.0) + 0.5);
    }
  }
}
}  // namespace ciface::Pipes
