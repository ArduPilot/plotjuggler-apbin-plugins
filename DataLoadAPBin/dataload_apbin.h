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
};