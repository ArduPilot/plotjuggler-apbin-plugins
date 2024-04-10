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

#include "dataload_apbin.h"
#include <QFile>
#include <QMessageBox>
#include <QProgressDialog>
#include <QDateTime>
#include <QInputDialog>
#include <QElapsedTimer>
#include <QDebug>
#include <chrono>
#include <cmath>


// Debugging 
//#define DEBUG_RUNTIME
//#define DEBUG_MESSAGES
//#define DEBUG_MULTIPLIERS
//#define DEBUG_UNITS


// Config
//#define LABEL_WITH_UNIT


bool is_nearly(double val, int val2)
{
  const double epsilon = 1e-10;
  if (std::abs(val - val2) < epsilon)
  {
    return true;
  }
  else
  {
    return false;
  }
}


DataLoadAPBIN::DataLoadAPBIN()
{
  extensions.push_back("BIN");  // TODO : this doesn't work for now as tolower() is hardcoded.
}

const std::vector<const char*>& DataLoadAPBIN::compatibleFileExtensions() const
{
  return extensions;
}

bool DataLoadAPBIN::readDataFromFile(FileLoadInfo* info, PlotDataMapRef& plot_data)
{
  QFile file(info->filename);
  if (!file.open(QFile::ReadOnly))
  {
    return false;
  }

  const QByteArray file_array = file.readAll();
  const int32_t file_size = file_array.size();

  const uint8_t* buf = reinterpret_cast<const uint8_t*>(file_array.data());
  const uint32_t len = file_array.size();
  uint32_t total_bytes_used = 0;

  // Progress box for large file
  QProgressDialog progress_dialog;
  progress_dialog.setLabelText("Loading ArduPilot logfile... please wait");
  progress_dialog.setWindowModality(Qt::ApplicationModal);
  progress_dialog.setRange(0, 100);
  progress_dialog.setAutoClose(true);
  progress_dialog.setAutoReset(true);
  progress_dialog.show();

  int progress{ 0 };
  int progress_update{ 0 };

  uint32_t bytes_skipped{ 0 };
  uint32_t msgs_skipped{ 0 };
  uint32_t msgs_read{ 0 };

  QElapsedTimer timer;
  timer.start();

  #ifdef DEBUG_RUNTIME
    std::chrono::duration<double, std::milli> fmt_ms{ 0 };
    std::chrono::duration<double, std::milli> fmtu_ms{ 0 };
    std::chrono::duration<double, std::milli> mult_ms{ 0 };
    std::chrono::duration<double, std::milli> unit_ms{ 0 };
    std::chrono::duration<double, std::milli> other_ms{ 0 };
    std::chrono::duration<double, std::milli> process_units_ms{ 0 };
    std::chrono::duration<double, std::milli> apply_mult_ms{ 0 };
    std::chrono::duration<double, std::milli> apply_tsync_ms{ 0 };
    std::chrono::duration<double, std::milli> publish_ms{ 0 };
  #endif

  while (true)
  {
    // update the progression dialog box
    progress_update = static_cast<int>((static_cast<double>(total_bytes_used) / static_cast<double>(file_size)) * 100.0);
    if ( (progress_update - 4) > progress )
    {
      progress = progress_update;
      progress_dialog.setValue(progress);
      QApplication::processEvents();
      if (progress_dialog.wasCanceled())
      {
        return false;
      }
    }

    // check if end of file is reached
    if (len - total_bytes_used < LOG_PACKET_HEADER_LEN)
    {
      progress_dialog.setValue(100);
      bytes_skipped += len - total_bytes_used;
      break;
    }

    // detect message start sequence (header)
    // skip through input until we find a valid header:
    if (buf[total_bytes_used] != HEAD_BYTE1 || buf[total_bytes_used + 1] != HEAD_BYTE2)
    {
      total_bytes_used += 1;
      bytes_skipped += 1;
      continue;
    }

    // get message-id from header
    const uint8_t type = buf[total_bytes_used + 2];


    // -------------------- handle FMT-message -------------------- //
    if (type == LOG_FORMAT_MSG)
    {
      #ifdef DEBUG_RUNTIME
        auto fmt_start = std::chrono::high_resolution_clock::now();
      #endif

      // check if we don't reach the end
      if ((uint32_t)(len - total_bytes_used) < sizeof(struct log_Format))
      {
        bytes_skipped += len - total_bytes_used;
        break;
      }

      // extract the message-id for which the FMT-message is defined and store FMT
      const uint8_t msg_id = ((struct log_Format*)(&(buf[total_bytes_used])))->type;
      has_fmt[msg_id] = true;
      struct log_Format& fmt = formats[msg_id];
      memcpy(&fmt, &buf[total_bytes_used], sizeof(struct log_Format));
      for (char i : fmt.name)
      {
        if (!isprint(i) && i != '\0')
        {
          // double check that this is a message
          // name is assumed to be printable ascii; it
          // looked like a format message, but wasn't.
          total_bytes_used++;
          bytes_skipped++;
          continue;
        }
      }

      // store message name <-> message id mapping
      uint8_t name_length = 0;
      for (char i : fmt.name)
      {
        if (i != '\0')
        {
          name_length++;
        }
      }
      std::string msg_name(fmt.name, name_length);
      msg_id2name[msg_id] = msg_name; 
      msg_name2id[msg_name] = msg_id;

      // store field name (label) <-> field idx mapping
      uint8_t label_length = 0;
      for (char i : fmt.labels)
      {
        if (i != '\0')
        {
          label_length++;
        }
      }
      std::string labels(fmt.labels, label_length);
      std::vector<std::string> labels_vec{};

      // split labels at delimiter ","
      size_t pos = 0;
      std::string label;
      while ( (pos = labels.find(",")) != std::string::npos )
      {
        label = labels.substr(0,pos);
        labels_vec.push_back(label);
        labels.erase(0, pos + 1);
      }
      labels_vec.push_back(labels);

      // go through labels
      for (size_t idx = 0; idx < labels_vec.size(); idx++)
      {
        const std::string& label = labels_vec[idx];
        field_name2idx[msg_name][label] = idx;

        /*
        This is not needed, since the detection based on the unit char '#' is sufficient
        // handle instances
        //  - check if labels contain "instance"
        if (strcmp(label.c_str(), "Instance") == 0)
        {
          has_instance[msg_id] = true;
          instance_idx[msg_id] = idx;
          instance_offset[msg_id] = get_field_byte_offset(msg_id, idx);
        }
        */
      }

      total_bytes_used += sizeof(struct log_Format);
      msgs_read++;

      #ifdef DEBUG_RUNTIME
        auto fmt_end = std::chrono::high_resolution_clock::now();
        fmt_ms += (fmt_end - fmt_start);
      #endif

      continue;
    }

    // get the full log format from the message type
    const struct log_Format& fmt = formats[type];

    // checks:
    //  - if length of message is zero, continue
    if ( fmt.length == 0 )
    {
      total_bytes_used += 1;
      bytes_skipped += 1;
      continue;
    }
    //  - if we reached the end of the log, just end
    if (len - total_bytes_used < fmt.length)
    {
      progress_dialog.setValue(100);
      bytes_skipped += len - total_bytes_used;
      break;
    }


    // -------------------- handle FMTU-message -------------------- //
    if ( memcmp(fmt.name, "FMTU", 4) == 0 )
    {
      #ifdef DEBUG_RUNTIME
        auto fmtu_start = std::chrono::high_resolution_clock::now();
      #endif

      // extract the message-id for which the FMTU-message is defined and store FMTU
      const uint8_t msg_id = ((struct log_Format_Units*)(&(buf[total_bytes_used])))->format_type;
      has_fmtu[msg_id] = true;
      struct log_Format_Units& fmtu = format_units[msg_id];
      memcpy(&fmtu, &buf[total_bytes_used], sizeof(struct log_Format_Units));


      // handle instances
      //  - check if units contain "#" (see also: logformat.h)
      if ( !has_instance[msg_id] )
      {
        uint8_t units_length = 0;
        for (char i : fmtu.units)
        {
          if (i != '\0')
          {
            units_length++;
          }
        }
        std::string units(fmtu.units, units_length);

        size_t pos = units.find("#");
        if ( pos != std::string::npos )
        {
          has_instance[msg_id] = true;
          instance_idx[msg_id] = pos;
          instance_offset[msg_id] = get_field_byte_offset(msg_id, pos);
        }
      } 
        
      total_bytes_used += fmt.length;
      msgs_read++;

      #ifdef DEBUG_RUNTIME
        auto fmtu_end = std::chrono::high_resolution_clock::now();
        fmtu_ms += (fmtu_end - fmtu_start);
      #endif

      continue;
    }


    // -------------------- handle MULT-message -------------------- //
    if ( memcmp(fmt.name, "MULT", 4) == 0 )
    {
      #ifdef DEBUG_RUNTIME
        auto mult_start = std::chrono::high_resolution_clock::now();
      #endif

      uint32_t id_offset = get_field_byte_offset(type, "Id");
      uint32_t mult_offset = get_field_byte_offset(type, "Mult");

      // todo: data type is hardcoded here, change that?!
      const unsigned char multiplier_char = *reinterpret_cast<const uint8_t*>(buf + total_bytes_used + id_offset);
      const double multiplier = *reinterpret_cast<const double*>(buf + total_bytes_used + mult_offset);

      multipliers[multiplier_char] = multiplier;
      
      total_bytes_used += fmt.length;
      msgs_read++;

      #ifdef DEBUG_RUNTIME
        auto mult_end = std::chrono::high_resolution_clock::now();
        mult_ms += (mult_end - mult_start);
      #endif

      continue;
    }


    // -------------------- handle UNIT-message -------------------- //
    if ( memcmp(fmt.name, "UNIT", 4) == 0 )
    {
      #ifdef DEBUG_RUNTIME
        auto unit_start = std::chrono::high_resolution_clock::now();
      #endif

      uint32_t id_offset = get_field_byte_offset(type, "Id");
      uint32_t label_offset = get_field_byte_offset(type, "Label");

      // todo: data type is hardcoded here, change that?!
      const unsigned char unit_char = *reinterpret_cast<const uint8_t*>(buf + total_bytes_used + id_offset);
      const char* unit = reinterpret_cast<const char*>(buf + total_bytes_used + label_offset);

      units[unit_char] = std::string(unit);

      total_bytes_used += fmt.length;
      msgs_read++;

      #ifdef DEBUG_RUNTIME
        auto unit_end = std::chrono::high_resolution_clock::now();
        unit_ms += (unit_end - unit_start);
      #endif

      continue;
    }


    // -------------------- handle any other message -------------------- //

    // discard some messages that should not be used:
    //  - ISBD, ISBH, MSG, PARM,

    if ( memcmp(fmt.name, "ISBD", 4) == 0 )
    {
      total_bytes_used += fmt.length;
      msgs_skipped++;
      continue;
    }
    if ( memcmp(fmt.name, "ISBH", 4) == 0 )
    {
      total_bytes_used += fmt.length;
      msgs_skipped++;
      continue;
    }
    if ( memcmp(fmt.name, "MSG", 3) == 0 )
    {
      total_bytes_used += fmt.length;
      msgs_skipped++;
      continue;
    }
    if ( memcmp(fmt.name, "PARM", 4) == 0 )
    {
      total_bytes_used += fmt.length;
      msgs_skipped++;
      continue;
    }

    #ifdef DEBUG_RUNTIME
      auto other_start = std::chrono::high_resolution_clock::now();
    #endif

    handle_message_received(fmt, &buf[total_bytes_used]);

    total_bytes_used += fmt.length;
    msgs_read++; // todo: this is incorrect, if message is read incomplete
    
    #ifdef DEBUG_RUNTIME
      auto other_end = std::chrono::high_resolution_clock::now();
      other_ms += (other_end - other_start);
    #endif
  }


  // -------------------- process UNITs -------------------- //
  #ifdef DEBUG_RUNTIME
      auto process_units_start = std::chrono::high_resolution_clock::now();
  #endif
  // - convert '/<unit>' spelling because it's incompatible with PlotJuggler
  //    - PlotJuggler uses '/' in names for data splitting into subtopics
  const std::string superscript_minus = "⁻";
  const std::array<std::string, 3> superscript_numbers = {"¹", "²", "³"};

  for (auto& unit_it : units)
  {
    std::string& unit = unit_it.second;

    std::map<std::string, uint8_t> counter;

    size_t pos = 0;
    while ( (pos = unit.find("/")) != std::string::npos )
    {
      std::string token = unit.substr(pos+1, 1);

      // push to map
      auto counter_it = counter.find(token);
      if (counter_it == counter.end())
      {
        counter[token] = 0;
      }
      else
      {
        counter[token]++;
      }
      
      // erase old '/<unit>' from string
      unit.erase(pos, pos + 1);
    }

    // append new style to string
    for (auto& counter_it : counter)
    {
      unit.append(" " + counter_it.first + superscript_minus + superscript_numbers[counter_it.second]);
    }
  }
  #ifdef DEBUG_RUNTIME
    auto process_units_end = std::chrono::high_resolution_clock::now();
    process_units_ms += (process_units_end - process_units_start);
  #endif


  // -------------------- apply multipliers -------------------- //
  #ifdef DEBUG_RUNTIME
    auto apply_mult_start = std::chrono::high_resolution_clock::now();
  #endif
  apply_multipliers();
  #ifdef DEBUG_RUNTIME
    auto apply_mult_end = std::chrono::high_resolution_clock::now();
    apply_mult_ms += (apply_mult_end - apply_mult_start);
  #endif


  // -------------------- apply timesync -------------------- //
  #ifdef DEBUG_RUNTIME
    auto apply_tsync_start = std::chrono::high_resolution_clock::now();
  #endif
  apply_timesync();
  #ifdef DEBUG_RUNTIME
    auto apply_tsync_end = std::chrono::high_resolution_clock::now();
    apply_tsync_ms += (apply_tsync_end - apply_tsync_start);
  #endif



  #ifdef DEBUG_MESSAGES
  std::printf("\n--------- DEBUG_MESSAGES ---------");
  for (int idx=0; idx < 256; idx++)
  {
    if (has_fmt[idx])
    {
      const struct log_Format& fmt = formats[idx];
      std::string msg_name = std::string(fmt.name, MAX_NAME_SIZE);
      std::string msg_labels = std::string(fmt.labels, MAX_LABELS_SIZE);
      std::string msg_format = std::string(fmt.format, MAX_FORMAT_SIZE);
      std::printf("\n%s:\n", msg_name.c_str());
      std::printf("  -id: \t\t%u\n", fmt.type);
      std::printf("  -labels: \t%s\n", msg_labels.c_str());
      std::printf("  -format: \t%s\n", msg_format.c_str());
      if (has_fmtu[idx])
      {
        const struct log_Format_Units& fmtu = format_units[idx];
        std::string msg_units = std::string(fmtu.units, MAX_UNITS_SIZE);
        std::string msg_multipliers = std::string(fmtu.multipliers, MAX_MULTIPLIERS_SIZE);
        std::printf("  -units: \t%s\n", msg_units.c_str());
        std::printf("  -multipliers: %s\n", msg_multipliers.c_str());
        if (has_instance[idx] == true)
        {
          std::printf("  -has instance at idx: %i\n", instance_idx[idx]);
        }        
      }
    }
  }
  std::printf("-------------- END --------------\n\n");
  #endif

  #ifdef DEBUG_MULTIPLIERS
  std::printf("\n------- DEBUG_MULTIPLIERS -------\n");
  for(const auto& multi_it : multipliers)
  {
    std::cout << multi_it.first << ": " << multi_it.second << std::endl;
  }
  std::printf("-------------- END --------------\n\n");
  #endif

  #ifdef DEBUG_UNITS
  std::printf("\n---------- DEBUG_UNITS ----------\n");
  for(const auto& unit_it : units)
  {
    std::cout << unit_it.first << ": " << unit_it.second << std::endl;
  }
  std::printf("-------------- END --------------\n\n");
  #endif



  // -------------------- publish to plotjuggler -------------------- //
  #ifdef DEBUG_RUNTIME
    auto publish_start = std::chrono::high_resolution_clock::now();
  #endif
  // iterate through messages
  for (const auto& msg_it : messages_map)
  {
    const std::string& msg_name = msg_it.first;

    // get message id for message name
    const uint8_t& msg_id = msg_name2id[msg_name];

    // only publish messages to plotjuggler, which have the "TimeUS" field!
    auto time_idx_it = field_name2idx[msg_name].find("TimeUS");
    if (time_idx_it == field_name2idx[msg_name].end())
    {
      std::printf("Ignoring message '%s' because it has no 'TimeUS' field!\n", msg_name.c_str());
      continue;
    }
    
    // iterate through instances
    const auto& instances_map = msg_it.second;
    for (const auto& inst_it : instances_map)
    {
      const message_data& msg_data = inst_it.second;

      // iterate through fields

      // extract timestamps from message data
      const uint8_t& time_idx = field_name2idx[msg_name]["TimeUS"];
      const std::vector<double>& timestamps = msg_data[time_idx].second;

      size_t idx = 0;
      for (const auto& field : msg_data)
      {
        if ( idx == time_idx || ( has_instance[msg_id] && (idx == instance_idx[msg_id]) ) )
        {
          idx++;
          continue;
        }

        const std::string instance_name = "#" + std::to_string(inst_it.first);
        const std::string& field_name = field.first;

        std::string series_name;
        
        if ( !has_instance[msg_id] )
        {
          series_name = "/" + msg_name + "/" + field_name;
        }
        else
        {
          series_name = "/" + msg_name + "/" + instance_name + "/" + field_name;
        }

        #ifdef LABEL_WITH_UNIT
          std::string unit_str = get_unit(msg_name, field_name);
          if ( !unit_str.empty() )
          {
            series_name = series_name + "\t[" + unit_str + "]";
          }
        #endif
                
        auto series = plot_data.addNumeric(series_name);

        for (size_t i = 0; i < field.second.size(); i++)
        {
          const double& msg_time = timestamps[i];
          PlotData::Point point(msg_time, field.second[i]);
          series->second.pushBack(point);
        }
        idx++;
      }
    }
  }
  #ifdef DEBUG_RUNTIME
    auto publish_end = std::chrono::high_resolution_clock::now();
    publish_ms += (publish_end - publish_start);
  #endif

  #ifdef DEBUG_RUNTIME
    std::chrono::duration<double, std::milli> total_ms = fmt_ms + fmtu_ms + mult_ms + unit_ms + other_ms + process_units_ms + apply_mult_ms + apply_tsync_ms + publish_ms;
    std::printf("\n--------- DEBUG_RUNTIME ---------");
    std::printf("\nFMT-Loading (ms): \t%.2f", fmt_ms.count());
    std::printf("\nFMTU-Loading (ms): \t%.2f", fmtu_ms.count());
    std::printf("\nMULT-Loading (ms): \t%.2f", mult_ms.count());
    std::printf("\nUNIT-Loading (ms): \t%.2f", unit_ms.count());
    std::printf("\nOTHER-Loading (ms): \t%.2f\n", other_ms.count());

    std::printf("\nProcess-Units (ms):\t%.2f", process_units_ms.count());
    std::printf("\nApply-Multipliers (ms):\t%.2f", apply_mult_ms.count());
    std::printf("\nApply-Timesync (ms):\t%.2f", apply_tsync_ms.count());
    std::printf("\nPublish (ms):\t\t%.2f", publish_ms.count());
    std::printf("\n---------------------------------");
    std::printf("\nTOTAL (ms):\t\t%.2f", total_ms.count());
    std::printf("\n-------------- END --------------\n\n");
  #endif
  
  file.close();

  qDebug() << "The loading operation took" << timer.elapsed() << "milliseconds";

  std::printf("\n  Read messages:\t%d", msgs_read);
  std::printf("\n  Skipped messages:\t%d", msgs_skipped);
  std::printf("\n  Skipped bytes:\t%d from %d bytes\n\n", bytes_skipped, len);

  return true;
}





void DataLoadAPBIN::handle_message_received(const struct log_Format& fmt, const uint8_t* msg)
{
  // message id
  const uint8_t& msg_id = fmt.type;

  // message name
  const std::string& msg_name = msg_id2name[msg_id];
  
  // instances
  int8_t instance = 0;
  if ( has_instance[msg_id] )
  {
    instance = get_instance(fmt, msg);
  }

  // check if message already exists in messages_map
  auto message_it = messages_map.find(msg_name);
  if (message_it == messages_map.end())
  {
    messages_map[msg_name];
  }
  
  // check if instance already exists in message_map[msg_name]
  auto instance_it = messages_map[msg_name].find(instance);
  if (instance_it == messages_map[msg_name].end())
  {
    messages_map[msg_name][instance] = create_message_data(fmt);
  }
  message_data& msg_data = messages_map[msg_name][instance];

  uint32_t msg_offset = LOG_PACKET_HEADER_LEN;  // discard header

  
  /*
    If you need to change this section, please also fix logformat.h (format_types)!
    AP_Logger: Format Types (https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Logger#format-types)
      - file: libraries/AP_Logger/LogStructure.h (commit: b80cc9a)
      - line: 9 - 28
  */

  // for each field, we get the data according to the format types
  // this uses pointer arithmetic, so just close your eyes
  double value{ 0 };
  for (int i = 0; i < msg_data.size(); i++)
  {
    const char typeCode = fmt.format[i];
    switch (typeCode)
    {
      case 'a':
        // not used, that is for ISBD
        msg_offset += sizeof(int16_t[32]);
        break;
      case 'b':
        value = static_cast<double>(*reinterpret_cast<const int8_t*>(msg + msg_offset));
        msg_offset += sizeof(int8_t);
        break;
      case 'B':
        value = static_cast<double>(*reinterpret_cast<const uint8_t*>(msg + msg_offset));
        msg_offset += sizeof(uint8_t);
        break;
      case 'h':
        value = static_cast<double>(*reinterpret_cast<const int16_t*>(msg + msg_offset));
        msg_offset += sizeof(int16_t);
        break;
      case 'H':
        value = static_cast<double>(*reinterpret_cast<const uint16_t*>(msg + msg_offset));
        msg_offset += sizeof(uint16_t);
        break;
      case 'i':
        value = static_cast<double>(*reinterpret_cast<const int32_t*>(msg + msg_offset));
        msg_offset += sizeof(int32_t);
        break;
      case 'I':
        value = static_cast<double>(*reinterpret_cast<const uint32_t*>(msg + msg_offset));
        msg_offset += sizeof(uint32_t);
        break;
      case 'f':
        value = static_cast<double>(*reinterpret_cast<const float*>(msg + msg_offset));
        msg_offset += sizeof(float);
        break;
      case 'd':
        value = static_cast<double>(*reinterpret_cast<const double*>(msg + msg_offset));
        msg_offset += sizeof(double);
        break;
      case 'n':
        // not used, that is for MSG or PARAM
        msg_offset += sizeof(char[4]);
        break;
      case 'N':
        // not used, that is for MSG or PARAM
        msg_offset += sizeof(char[16]);
        break;
      case 'Z':
        // not used, that is for MSG or PARAM
        msg_offset += sizeof(char[64]);
        break;
      case 'c':
        value = static_cast<double>(*reinterpret_cast<const int16_t*>(msg + msg_offset));
        msg_offset += sizeof(int16_t);
        break;
      case 'C':
        value = static_cast<double>(*reinterpret_cast<const uint16_t*>(msg + msg_offset));
        msg_offset += sizeof(uint16_t);
        break;
      case 'e':
        value = static_cast<double>(*reinterpret_cast<const int32_t*>(msg + msg_offset));
        msg_offset += sizeof(int32_t);
        break;
      case 'E':
        value = static_cast<double>(*reinterpret_cast<const uint32_t*>(msg + msg_offset));
        msg_offset += sizeof(uint32_t);
        break;
      case 'L':
        value = static_cast<double>(*reinterpret_cast<const int32_t*>(msg + msg_offset));
        msg_offset += sizeof(int32_t);
        break;
      case 'M':
        value = static_cast<double>(*reinterpret_cast<const uint8_t*>(msg + msg_offset));
        msg_offset += sizeof(uint8_t);
        break;
      case 'q':
        value = static_cast<double>(*reinterpret_cast<const int64_t*>(msg + msg_offset));
        msg_offset += sizeof(int64_t);
        break;
      case 'Q':
        value = static_cast<double>(*reinterpret_cast<const uint64_t*>(msg + msg_offset));
        msg_offset += sizeof(uint64_t);
        break;
      default:
        std::fprintf(stderr, "ERROR: format type '%c' is not defined!\n", typeCode); 
        // At this point the field offset is unknown, therefore we can not proceed to interpret the remaining fields!
        return;
    }

    msg_data[i].second.push_back(value);

  }
}



DataLoadAPBIN::message_data DataLoadAPBIN::create_message_data(const struct log_Format& fmt)
{
  QString labelStr(fmt.labels);
  labelStr.truncate(MAX_LABELS_SIZE);
  QStringList labels_list;
  if (labelStr.size() > 0)
  {
    labels_list = labelStr.split(",");
  }

  message_data msg_data;
  msg_data.reserve(labels_list.size());
  for (auto i = 0; i < labels_list.size(); i++)
  {
    msg_data.emplace_back(labels_list.at(i).toLocal8Bit().constData(), std::vector<double>());
  }
  return msg_data;
}



uint32_t DataLoadAPBIN::get_field_byte_offset(const uint8_t& msg_id, const uint8_t& field_idx)
{
  // check if FMT exists
  if ( !has_fmt[msg_id])
  {
    // raise error, because fmt does not exist!
    std::fprintf(stderr, "ERROR: FMT for message %u does not exist!\n", msg_id);
    exit(EXIT_FAILURE);
  }

  // get information from FMT
  const struct log_Format& fmt = formats[msg_id];
  const char* format = fmt.format;

  // calculate offsets:

  // - header offset
  uint32_t header_offset = LOG_PACKET_HEADER_LEN;

  // - data offset
  uint32_t data_offset = 0;
  for (uint8_t idx = 0; idx < field_idx; idx++)
  {
    const auto format_types_it = format_types.find(format[idx]);
    if ( format_types_it == format_types.end() )
    {
      // raise error, because format type definition is missing
      std::fprintf(stderr, "ERROR: format type '%c' is not defined!\n", format[idx]);
      exit(EXIT_FAILURE);
    }
    data_offset += format_types_it->second;
  }

  uint32_t total_offset = header_offset + data_offset;

  return total_offset;
}



uint32_t DataLoadAPBIN::get_field_byte_offset(const uint8_t& msg_id, const std::string& field_name)
{
  const std::string& msg_name = msg_id2name[msg_id];
  const uint8_t field_idx = field_name2idx[msg_name][field_name];

  return get_field_byte_offset(msg_id, field_idx);
}



uint8_t DataLoadAPBIN::get_instance(const struct log_Format& fmt, const uint8_t* msg)
{
  // Read sensor instance from raw message byte sequence

  // get message id from FMT
  const uint8_t& msg_id = fmt.type;
  
  // get instance byte offset
  const uint32_t& inst_offset = instance_offset[msg_id];

  // get instance
  uint8_t instance{ 0 };
  memcpy(&instance, &msg[inst_offset], sizeof(uint8_t));

  return instance;
}



std::string DataLoadAPBIN::get_unit(const std::string& msg_name, const std::string& field_name)
{
  // get message id for message name
  const uint8_t& msg_id = msg_name2id[msg_name];

  // check if FMTU exists
  if ( !has_fmtu[msg_id] )
  {
    std::fprintf(stderr, "\nWARNING: no FMTU for message %s found! Can not apply units!\n", msg_name.c_str());
    return "";
  }

  // get data index of field
  const uint8_t& idx = field_name2idx[msg_name][field_name];

  // get unit descriptor char
  const char& unit_char = format_units[msg_id].units[idx];

  // get unit string 
  const auto& unit_it = units.find(unit_char);
  if ( unit_it == units.end() )
  {
    std::fprintf(stderr, "WARNING: No unit for unit-id %c found! Can not apply unit in message: %s\n", unit_char, msg_name.c_str());
    return "";
  }

  return unit_it->second;
}



void DataLoadAPBIN::apply_multipliers(void)
{
  // Go through all messages, instances, fields and apply the correct multiplier from FMTU and MULT

  // iterate through messages
  for (auto& msg_it : messages_map)
  {
    const std::string& msg_name = msg_it.first;

    // get message id for message name
    const uint8_t& msg_id = msg_name2id[msg_name];

    // check if FMTU exists
    if ( !has_fmtu[msg_id] )
    {
      std::fprintf(stderr, "WARNING: No FMTU for message %s found. Can not apply multipliers!\n", msg_name.c_str());
      continue;
    }

    // iterate through instances
    auto& instances_map = msg_it.second;
    for (auto& inst_it : instances_map)
    {
      message_data& msg_data = inst_it.second;

      // iterate through fields
      for (int idx = 0; idx < msg_data.size(); idx++)
      {
        // get multiplier descriptor char
        const char& field_multiplier_char = format_units[msg_id].multipliers[idx];

        // get multiplier double
        const auto multiplier_it = multipliers.find(field_multiplier_char);
        if ( multiplier_it == multipliers.end() )
        {
          std::fprintf(stderr, "WARNING: No multiplier for multiplier-id %c found! Can not apply multiplier in message: %s\n", field_multiplier_char, msg_name.c_str());
          continue;
        }
        const double field_multiplier = multiplier_it->second;

        // check if multiplier is 0 or 1
        if ( is_nearly(field_multiplier, 0) || is_nearly(field_multiplier, 1) )
        {
          continue;
        }

        std::vector<double>& field_data = msg_data[idx].second;
        std::transform(field_data.begin(), field_data.end(), field_data.begin(), std::bind(std::multiplies<double>(), std::placeholders::_1, field_multiplier));
      }
    }
  }
}



void DataLoadAPBIN::apply_timesync(void)
{
  // ArduPilot logs should be comparable with rosbags, therefore the same time basis is needed...
  //  - rosbag:         unix time
  //  - ArduPilot log:  local time since power on
  //    -> the logged GNSS time can be used for synchronisation

  // extract GNSS time
  const auto msg_it = messages_map.find("GPS");
  if ( msg_it == messages_map.end() )
  {
    std::printf("Skipping timesync because the logfile does not contain GNSS data\n");
    return;
  }

  // take the first instance as reference
  // todo: change that?
  const message_data& gps_msg_data = messages_map["GPS"][0];

  // counter
  int idx;
  
  // get needed field indexes
  const auto& gps_time_idx = field_name2idx["GPS"]["TimeUS"];
  const auto& gps_week_idx = field_name2idx["GPS"]["GWk"]; // GWk -> GPS week
  const auto& gps_ms_idx = field_name2idx["GPS"]["GMS"];   // GMS -> GPS seconds in week (ms)

  // constant time offset variables
  static constexpr double GPS2UNIX_TIME_OFFSET = 315964800;   // time offset between unix and gps time
  static constexpr double GPS2UNIX_LEAP_SECONDS = -18;        // additional time offset due to leap seconds (must be adjusted if number of leap seconds changes!)
  static constexpr double SECONDS_PER_WEEK = 604800;          // number of seconds per week

  const double& gps_week = gps_msg_data[gps_week_idx].second[1];
  const double gps_week_seconds = gps_msg_data[gps_ms_idx].second[1] * 0.001;

  const double unix_time = gps_week * SECONDS_PER_WEEK + gps_week_seconds + GPS2UNIX_TIME_OFFSET + GPS2UNIX_LEAP_SECONDS;

  const double& log_time = gps_msg_data[gps_time_idx].second[1];

  const double time_offset = unix_time - log_time;


  // iterate through messages
  for (auto& msg_it : messages_map)
  {
    const auto& msg_name = msg_it.first;

    auto time_idx_it = field_name2idx[msg_name].find("TimeUS");
    if (time_idx_it == field_name2idx[msg_name].end())
    {
      continue;
    }
    auto& time_idx = time_idx_it->second;

    // iterate through instances
    auto& instances_map = msg_it.second;
    for (auto& inst_it : instances_map)
    {
      // add time offset
      message_data& msg_data = inst_it.second;

      std::vector<double>& timestamps = msg_data[time_idx].second;
      std::transform(timestamps.begin(), timestamps.end(), timestamps.begin(), std::bind(std::plus<double>(), std::placeholders::_1, time_offset));
    }
  }
}