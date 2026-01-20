/**
 * @file
 * @author Pierre Kancir <pierre.kancir.emn@gmail.com>
 * @author Jonas Withelm <IAV GmbH>
 * 
 * @section DESCRIPTION
 *
 * ArduPilot DataFlash binaries loader for Plotjuggler.
 * This load and decode ArduPilot DataFlash binaries to extract logging information in readable and plotable state.
 * The logic is derived from Dronekit-La software (https://github.com/dronekit/dronekit-la).
 *
 */

#pragma once

#include <QObject>
#include <QtPlugin>
#include "PlotJuggler/dataloader_base.h"
#include "logformat.h"

using namespace PJ;

class DataLoadAPBIN : public DataLoader
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID "facontidavide.PlotJuggler3.DataLoader")
  Q_INTERFACES(PJ::DataLoader)

public:
  DataLoadAPBIN();
  virtual const std::vector<const char*>& compatibleFileExtensions() const override;

  bool readDataFromFile(PJ::FileLoadInfo* fileload_info, PlotDataMapRef& destination) override;

  ~DataLoadAPBIN() override = default;

  virtual const char* name() const override
  {
    return "ArduPilot Bin";
  }

protected:

private:
  std::vector<const char*> extensions;


  // message_data holds the data of a message for each timestamp
  //  - std::string:  field name (label)
  //  - std::vector:  field data (fields)
  typedef std::vector<std::pair<std::string, std::vector<double>>> message_data;

  // messages_map is a nested map which contains all messages
  //  - key1:   message name
  //  - key2:   instance number
  //  - value:  message_data
  std::map<std::string, std::map<int8_t, message_data>> messages_map;


  // multipliers and units from MULT and UNIT messages
  std::map<char, double> multipliers;
  std::map<char, std::string> units;


  // format (FMT) and format unit (FMTU) handling variables
  static constexpr uint16_t MAX_FORMATS = 256;
  struct log_Format formats[MAX_FORMATS] = {};            // FMT
  struct log_Format_Units format_units[MAX_FORMATS] = {}; // FMTU

  bool has_fmt[MAX_FORMATS] = {false};    // indicator, if FMT for a given message id exists
  bool has_fmtu[MAX_FORMATS] = {false};   // indicator, if FMTU for a given message id exists


  // instance handling variables
  bool has_instance[MAX_FORMATS] = {false};     // indicator, if a message contains intances
  int instance_idx[MAX_FORMATS] = {-1};         // index of field, which contains the instance number
  uint32_t instance_offset[MAX_FORMATS] = {0};  // byte-offset of field, which containts the instance number


  // message name <-> message id mapping
  std::string msg_id2name[MAX_FORMATS] = {};
  std::map<std::string, uint8_t> msg_name2id;


  // field name <-> field idx mapping
  std::map<std::string, std::map<std::string, uint8_t>> field_name2idx;


  // helper
  std::string format_value(float val);

  // fill the message_data for a message according to the message format
  void handle_message_received(const struct log_Format& fmt, const uint8_t* msg);

  // create message_data for a message
  message_data create_message_data(const struct log_Format& fmt);

  // get the byte offset of a field in a message
  uint32_t get_field_byte_offset(const uint8_t& msg_id, const uint8_t& field_idx);
  uint32_t get_field_byte_offset(const uint8_t& msg_id, const std::string& field_name);

  // get the instance number from a message
  uint8_t get_instance(const struct log_Format& fmt, const uint8_t* msg);

  // get unit string for a field
  std::string get_unit(const std::string& msg_name, const std::string& field_name);

  // apply multipliers from FMTU and MULT messages to the messages_map
  void apply_multipliers(void);

  // apply time synchronization to the messages_map
  void apply_timesync(void);

  static double gps_to_unix_time(double gps_week, double gps_ms_of_week);

  std::vector<Parameter> _parameters;
  std::vector<StatusText> _status_texts;
  std::string _servo_function_labels[32];

  const std::map<int, const char*> SERVO_FUNCTION_MAP = {
    {-1, "GPIO"},
    {0, "Disabled"},
    {1, "RCPassThru"},
    {2, "Flap"},
    {3, "FlapAuto"},
    {4, "Aileron"},
    {6, "Mount1Yaw"},
    {7, "Mount1Pitch"},
    {8, "Mount1Roll"},
    {9, "Mount1Retract"},
    {10, "CameraTrigger"},
    {12, "Mount2Yaw"},
    {13, "Mount2Pitch"},
    {14, "Mount2Roll"},
    {15, "Mount2Retract"},
    {16, "DifferentialSpoilerLeft1"},
    {17, "DifferentialSpoilerRight1"},
    {19, "Elevator"},
    {21, "Rudder"},
    {22, "SprayerPump"},
    {23, "SprayerSpinner"},
    {24, "FlaperonLeft"},
    {25, "FlaperonRight"},
    {26, "GroundSteering"},
    {27, "Parachute"},
    {28, "Gripper"},
    {29, "LandingGear"},
    {30, "EngineRunEnable"},
    {31, "HeliRSC"},
    {32, "HeliTailRSC"},
    {33, "Motor1"},
    {34, "Motor2"},
    {35, "Motor3"},
    {36, "Motor4"},
    {37, "Motor5"},
    {38, "Motor6"},
    {39, "Motor7\\TailTiltServo"},
    {40, "Motor8"},
    {41, "TiltMotorsFront"},
    {45, "TiltMotorsRear"},
    {46, "TiltMotorRearLeft"},
    {47, "TiltMotorRearRight"},
    {51, "RCIN1\\Pitch"},
    {52, "RCIN2\\Roll"},
    {53, "RCIN3\\HeaveVertical"},
    {54, "RCIN4\\YawTurn"},
    {55, "RCIN5\\SurgeForward"},
    {56, "RCIN6\\SwayLateral"},
    {57, "RCIN7\\CameraPan"},
    {58, "RCIN8\\CameraTilt"},
    {59, "RCIN9"},
    {60, "RCIN10"},
    {61, "RCIN11"},
    {62, "RCIN12"},
    {63, "RCIN13"},
    {64, "RCIN14"},
    {65, "RCIN15"},
    {66, "RCIN16"},
    {67, "Ignition"},
    {69, "Starter"},
    {70, "Throttle"},
    {71, "TrackerYaw"},
    {72, "TrackerPitch"},
    {73, "ThrottleLeft"},
    {74, "ThrottleRight"},
    {75, "TiltMotorFrontLeft"},
    {76, "TiltMotorFrontRight"},
    {77, "ElevonLeft"},
    {78, "ElevonRight"},
    {79, "VTailLeft"},
    {80, "VTailRight"},
    {81, "BoostThrottle"},
    {82, "Motor9"},
    {83, "Motor10"},
    {84, "Motor11"},
    {85, "Motor12"},
    {86, "DifferentialSpoilerLeft2"},
    {87, "DifferentialSpoilerRight2"},
    {88, "Winch"},
    {89, "Main Sail"},
    {90, "CameraISO"},
    {91, "CameraAperture"},
    {92, "CameraFocus"},
    {93, "CameraShutterSpeed"},
    {94, "Script1"},
    {95, "Script2"},
    {96, "Script3"},
    {97, "Script4"},
    {98, "Script5"},
    {99, "Script6"},
    {100, "Script7"},
    {101, "Script8"},
    {102, "Script9"},
    {103, "Script10"},
    {104, "Script11"},
    {105, "Script12"},
    {106, "Script13"},
    {107, "Script14"},
    {108, "Script15"},
    {109, "Script16"},
    {110, "Airbrakes"},
    {120, "NeoPixel1"},
    {121, "NeoPixel2"},
    {122, "NeoPixel3"},
    {123, "NeoPixel4"},
    {124, "RateRoll"},
    {125, "RatePitch"},
    {126, "RateThrust"},
    {127, "RateYaw"},
    {128, "WingSailElevator"},
    {129, "ProfiLED1"},
    {130, "ProfiLED2"},
    {131, "ProfiLED3"},
    {132, "ProfiLEDClock"},
    {133, "Winch Clutch"},
    {134, "SERVOn_MIN"},
    {135, "SERVOn_TRIM"},
    {136, "SERVOn_MAX"},
    {137, "SailMastRotation"},
    {138, "Alarm"},
    {139, "Alarm Inverted"},
    {140, "RCIN1Scaled"},
    {141, "RCIN2Scaled"},
    {142, "RCIN3Scaled"},
    {143, "RCIN4Scaled"},
    {144, "RCIN5Scaled"},
    {145, "RCIN6Scaled"},
    {146, "RCIN7Scaled"},
    {147, "RCIN8Scaled"},
    {148, "RCIN9Scaled"},
    {149, "RCIN10Scaled"},
    {150, "RCIN11Scaled"},
    {151, "RCIN12Scaled"},
    {152, "RCIN13Scaled"},
    {153, "RCIN14Scaled"},
    {154, "RCIN15Scaled"},
    {155, "RCIN16Scaled"},
    {160, "Motor13"},
    {161, "Motor14"},
    {162, "Motor15"},
    {163, "Motor16"},
    {164, "Motor17"},
    {165, "Motor18"},
    {166, "Motor19"},
    {167, "Motor20"},
    {168, "Motor21"},
    {169, "Motor22"},
    {170, "Motor23"},
    {171, "Motor24"},
    {172, "Motor25"},
    {173, "Motor26"},
    {174, "Motor27"},
    {175, "Motor28"},
    {176, "Motor29"},
    {177, "Motor30"},
    {178, "Motor31"},
    {179, "Motor32"},
    {180, "CameraZoom"},
    {181, "Lights1"},
    {182, "Lights2"},
    {183, "VideoSwitch"},
    {184, "Actuator1"},
    {185, "Actuator2"},
    {186, "Actuator3"},
    {187, "Actuator4"},
    {188, "Actuator5"},
    {189, "Actuator6"}
  };
};
