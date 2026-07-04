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
#include <QElapsedTimer>
#include <QDebug>
#include <QXmlStreamReader>
#include <QFileInfo>
#include <QDir>
#include <QCoreApplication>
#include "LogMessageDescriptions.h"
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <cstring>
#include <cctype>

// Debugging 
//#define DEBUG_RUNTIME
//#define DEBUG_MESSAGES
//#define DEBUG_MULTIPLIERS
//#define DEBUG_UNITS

// Config
#define LABEL_WITH_UNIT

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
  qRegisterMetaType<std::vector<Parameter>>();
  qRegisterMetaType<std::vector<StatusText>>();
}

const std::vector<const char*>& DataLoadAPBIN::compatibleFileExtensions() const
{
  return extensions;
}

bool DataLoadAPBIN::readDataFromFile(FileLoadInfo* info, PlotDataMapRef& plot_data)
{
  // Reset all per-load state. Without this, member variables persist across
  // file loads in the same PlotJuggler session: stale has_instance / instance_offset
  // entries from a previous log get applied to the current one whenever ArduPilot
  // recycles a msg_id, producing bogus per-instance topics (e.g. HEAT split into
  // dozens of "#N" children) and skewed timestamps from re-applied multipliers.
  std::fill(std::begin(has_fmt), std::end(has_fmt), false);
  std::fill(std::begin(has_fmtu), std::end(has_fmtu), false);
  std::fill(std::begin(has_instance), std::end(has_instance), false);
  std::fill(std::begin(instance_idx), std::end(instance_idx), -1);
  std::fill(std::begin(instance_offset), std::end(instance_offset), 0u);
  for (auto& f : formats) { f = {}; }
  for (auto& fu : format_units) { fu = {}; }
  for (auto& n : msg_id2name) { n.clear(); }
  msg_name2id.clear();
  field_name2idx.clear();
  messages_map.clear();
  multipliers.clear();
  units.clear();

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
    // check if end of file is reached
    if (len - total_bytes_used < LOG_PACKET_HEADER_LEN)
    {
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
      const auto* msg = reinterpret_cast<const log_message*>(&buf[total_bytes_used]);
      _status_texts.push_back({msg->time_us, 0, msg->msg});

      total_bytes_used += fmt.length;
      msgs_read++;
      continue;
    }
    if ( memcmp(fmt.name, "PARM", 4) == 0 )
    {
      const auto* param = reinterpret_cast<const log_param*>(&buf[total_bytes_used]);
      std::string name = std::string(param->name);
      _parameters.push_back({name + "          Default: " + format_default_value(param->default_value) , param->value});

      #ifdef LABEL_RCOU_FUNCTION
      // save servo functions
      QRegExp rx("SERVO(\\d+)_FUNCTION");
      if (rx.indexIn(QString::fromStdString(name)) != -1) {
        bool is_int = false;
        int servo_idx = rx.cap(1).toInt(&is_int);

        if (is_int && servo_idx > 0 && servo_idx <= 32) {
          const int function_id = static_cast<int>(param->value);
          // skip over the "disabled" ones
          if (function_id != 0) {
            auto it = SERVO_FUNCTION_MAP.find(function_id);
            if (it != SERVO_FUNCTION_MAP.end()) {
              _servo_function_labels[servo_idx - 1] = it->second;
            }
          }
        }
      }
      #endif

      total_bytes_used += fmt.length;
      msgs_read++;
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
#ifdef QT_CORE_LIB
  load_log_messages(info->filename);
#else
  load_log_messages(info->filename);
#endif
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
      bool message_desc_assigned = false;
      std::string message_desc;
      auto mit = _message_tooltips.find(msg_name);
      if (mit != _message_tooltips.end()) message_desc = mit->second;
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
        
        #ifdef LABEL_RCOU_FUNCTION
        // label servo functions
        if (msg_name == "RCOU" || msg_name == "RCO2") {
          QRegExp re(R"(C(\d+))");
          if(re.indexIn(QString::fromStdString(field_name)) != -1) {
            int label_idx = re.cap(1).toInt() - 1; // is 1-based

            if(label_idx >= 0 && label_idx < 32 && !_servo_function_labels[label_idx].empty()) {
              series_name = series_name + " (" + _servo_function_labels[label_idx] + ")";
            }
          }
        }
        #endif


        #ifdef LABEL_WITH_UNIT
          std::string unit_str = get_unit(msg_name, field_name);
          if ( !unit_str.empty() )
          {
            series_name = series_name + "\t[" + unit_str + "]";
          }
        #endif
                
        auto series = plot_data.addNumeric(series_name);

        // Prefer per-field descriptions. Only attach message description to the
        // first series for this message instance that does not have a field description.
        std::string field_desc;
        auto fit_map_it = _field_tooltips.find(msg_name);
        if (fit_map_it != _field_tooltips.end())
        {
          auto fit = fit_map_it->second.find(field_name);
          if (fit != fit_map_it->second.end())
          {
            field_desc = fit->second;
          }
        }

        if (!field_desc.empty())
        {
          series->second.setAttribute(PJ::TOOL_TIP, QString::fromStdString(field_desc));
        }
        else if (!message_desc_assigned && !message_desc.empty())
        {
          // attach message description once for this instance
          series->second.setAttribute(PJ::TOOL_TIP, QString::fromStdString(message_desc));
          message_desc_assigned = true;
        }

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

  #if defined(DEBUG_RUNTIME) && defined(LABEL_RCOU_FUNCTION)
  for (int i = 0; i < 32; i++) {
    if (i < 14) {
      std::printf("\nRCOU.C%d: %s", i + 1, _servo_function_labels[i].c_str());
    } else {
      std::printf("\nRCO2.C%d: %s", i + 1, _servo_function_labels[i].c_str());
    }
  }
  #endif

  // Now, add the parameters as time series.
  for (const auto& param : _parameters)
  {
    auto series = plot_data.addNumeric(std::string("/1. Parameters/") + param.name);
    series->second.pushBack({300, param.value});
  }


  #ifdef DEBUG_MESSAGES
  std::printf("Message Log\n---------------");
  #endif

  for(int i = 0; i < _status_texts.size(); i++)
  {
    #ifdef DEBUG_MESSAGES
    std::printf("\n %llu: %s", _status_texts[i].timestamp, _status_texts[i].msg.c_str());
    #endif

    // change "/" to "\" if a status_text contains one so it doesn't create a child entry
    std::string msg = _status_texts[i].msg;
    size_t start_pos = 0;
    while((start_pos = msg.find('/', start_pos)) != std::string::npos) {
        msg.replace(start_pos, 1, "\\");
        start_pos += 2; // Handles cases where the replacement also contains the search string
    }

    auto time_s = static_cast<double>(_status_texts[i].timestamp)/1e6;

    auto& timeline = plot_data.getOrCreateNumeric("/2. Message Timeline");
    timeline.pushBack({time_s + _time_offset, (double)(i + 1)});

    auto series = plot_data.addNumeric(std::string("/3. Message Log/") + std::to_string(i + 1) + ". " + msg);
    series->second.pushBack({0.0, time_s - _start_plot_time});
  }

  #ifdef DEBUG_MESSAGES
  std::printf("\n---------------\n");
  #endif

  return true;
}

std::string DataLoadAPBIN::format_default_value(float val) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << val;
    std::string s = ss.str();

    // trim trailing zeros
    s.erase(s.find_last_not_of('0') + 1);
    if (!s.empty() && s.back() == '.') s.pop_back();

    return s;
}

std::string DataLoadAPBIN::format_time(double seconds)
{
    int minutes = static_cast<int>(seconds / 60.0);
    double rem = seconds - minutes * 60.0;

    int secs = static_cast<int>(rem);
    int tenths = static_cast<int>(std::round((rem - secs) * 10.0));

    // handle rounding overflow (e.g. 59.96 -> 1:00.0)
    if (tenths == 10) {
        tenths = 0;
        secs++;
        if (secs == 60) {
            secs = 0;
            minutes++;
        }
    }

    char buf[16];
    std::snprintf(buf, sizeof(buf), "%d:%02d.%d", minutes, secs, tenths);
    return buf;
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


void DataLoadAPBIN::load_log_messages(const QString& datafile_path)
{
  // Helper to parse a QXmlStreamReader and merge descriptions
  auto parseXml = [&](QXmlStreamReader &xml) {
    while (!xml.atEnd() && !xml.hasError())
    {
      xml.readNext();
      if (xml.isStartElement() && xml.name() == QLatin1String("logformat"))
      {
        QString msgName = xml.attributes().value(QLatin1String("name")).toString();
        QString msgDesc;

        // process children until end of this logformat
        while (!(xml.isEndElement() && xml.name() == QLatin1String("logformat")))
        {
          xml.readNext();
          if (xml.isStartElement())
          {
            if (xml.name() == QLatin1String("description"))
            {
              msgDesc = xml.readElementText().trimmed();
              _message_tooltips[msgName.toStdString()] = msgDesc.toStdString();
            }
            else if (xml.name() == QLatin1String("field"))
            {
              QString fieldName = xml.attributes().value(QLatin1String("name")).toString();
              // read inside field until end
              while (!(xml.isEndElement() && xml.name() == QLatin1String("field")))
              {
                xml.readNext();
                if (xml.isStartElement() && xml.name() == QLatin1String("description"))
                {
                  QString fdesc = xml.readElementText().trimmed();
                  _field_tooltips[msgName.toStdString()][fieldName.toStdString()] = fdesc.toStdString();
                }
              }
            }
          }
        }
      }
    }
  };

  // First, parse embedded XMLs compiled into the library (if present).
  // Use a sanity check on the embedded C-string rather than relying on sizeof.
  if (kEmbeddedLogMessagesXml[0] != '\0')
  {
    const char* emb = kEmbeddedLogMessagesXml;
    size_t emb_len = std::strlen(emb);
    // require a minimal length and at least one '<' character after skipping whitespace
    if (emb_len > 10)
    {
      size_t i = 0;
      while (i < emb_len && std::isspace(static_cast<unsigned char>(emb[i]))) ++i;
      if (i < emb_len && emb[i] == '<')
      {
        QXmlStreamReader xmlEmbedded(QString::fromUtf8(emb));
        parseXml(xmlEmbedded);
        if (xmlEmbedded.hasError())
        {
          qDebug() << "Warning: embedded log message XML parse error:" << xmlEmbedded.errorString();
        }
      }
    }
  }
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


double DataLoadAPBIN::gps_to_unix_time(double gps_week, double gps_ms_of_week)
{
    static constexpr double SECONDS_PER_WEEK      = 60 * 60 * 24 * 7;   // 60 * 60 * 24 * 7
    static constexpr double MS_PER_SECOND         = 1000.0;
    static constexpr double GPS2UNIX_TIME_OFFSET  = 315964800.0; // Unix epoch vs GPS epoch
    static constexpr double GPS2UNIX_LEAP_SECONDS = -18.0;       // Current leap seconds

    const double gps_week_seconds = gps_ms_of_week / MS_PER_SECOND;

    return (gps_week * SECONDS_PER_WEEK)
         + gps_week_seconds
         + GPS2UNIX_TIME_OFFSET
         + GPS2UNIX_LEAP_SECONDS;
}

namespace
{

  // Small struct to carry all four resolved GPS field indices together,
  // instead of passing four separate uint8_t out-parameters around.
  struct GpsFieldIndices
  {
    uint8_t time_idx;
    uint8_t week_idx;
    uint8_t ms_idx;
    uint8_t nsats_idx;
  };

  // Resolves the index of each required GPS field by name.
  // Returns std::nullopt (and prints a clear message) if any field is
  // missing, instead of silently defaulting to index 0 via
  // std::map::operator[].
  std::optional<GpsFieldIndices> resolve_gps_field_indices(
      const std::map<std::string, std::map<std::string, uint8_t>>& field_name2idx)
  {
    const auto gps_fields_it = field_name2idx.find("GPS");
    if (gps_fields_it == field_name2idx.end())
    {
      std::printf("Skipping timesync because the logfile has no field definitions for GPS\n");
      return std::nullopt;
    }
    const auto& gps_fields = gps_fields_it->second;

    auto find_field = [&](const std::string& label) -> std::optional<uint8_t>
    {
      const auto it = gps_fields.find(label);
      if (it == gps_fields.end())
      {
        std::printf("Skipping timesync because GPS message has no '%s' field!\n", label.c_str());
        return std::nullopt;
      }
      return it->second;
    };

    const auto time_idx  = find_field("TimeUS");
    const auto week_idx  = find_field("GWk");
    const auto ms_idx    = find_field("GMS");
    const auto nsats_idx = find_field("NSats");

    if (!time_idx || !week_idx || !ms_idx || !nsats_idx)
    {
      return std::nullopt;
    }

    return GpsFieldIndices{ *time_idx, *week_idx, *ms_idx, *nsats_idx };
  }

  // Scans the logged GPS timeline for the first sample with a usable fix:
  // enough satellites, and non-zero TimeUS/GWk/GMS. GWk in particular must be
  // checked, since a sample can report NSats > 4 and non-zero TimeUS/GMS
  // while GWk is still 0 (week number not yet resolved) - accepting that
  // sample would anchor the whole log to the GPS epoch (1980-01-06).
  std::optional<size_t> find_first_valid_gps_sample(
      const std::vector<double>& time_vec,
      const std::vector<double>& week_vec,
      const std::vector<double>& ms_vec,
      const std::vector<double>& nsats_vec,
      double min_nsats)
  {
    for (size_t i = 0; i < time_vec.size(); ++i)
    {
      if (i < nsats_vec.size() && i < week_vec.size() && i < ms_vec.size() &&
          nsats_vec[i] >= min_nsats &&
          time_vec[i]  != 0.0 &&
          week_vec[i]  > 0.0  &&
          ms_vec[i]    > 0.0)
      {
        return i;
      }
    }
    return std::nullopt;
  }

  void shift_all_timestamps(
    std::map<std::string, std::map<int8_t,
        std::vector<std::pair<std::string, std::vector<double>>>>>& messages_map,
    const std::map<std::string, std::map<std::string, uint8_t>>& field_name2idx,
    double time_offset_sec)
  {
    for (auto& [msg_name, instances_map] : messages_map)
    {
      const auto msg_fields_it = field_name2idx.find(msg_name);
      if (msg_fields_it == field_name2idx.end())
      {
        continue;
      }

      const auto time_idx_it = msg_fields_it->second.find("TimeUS");
      if (time_idx_it == msg_fields_it->second.end())
      {
        continue;
      }
      const auto& msg_time_idx = time_idx_it->second;

      for (auto& [instance_id, msg_data] : instances_map)
      {
        std::vector<double>& timestamps = msg_data[msg_time_idx].second;
        std::transform(timestamps.begin(), timestamps.end(), timestamps.begin(),
                        [time_offset_sec](double t) { return t + time_offset_sec; });
      }
    }
  }

}

void DataLoadAPBIN::apply_timesync(void)
{
  static constexpr double MIN_VALID_NSATS = 4;

  const auto msg_it = messages_map.find("GPS");
  if (msg_it == messages_map.end() || msg_it->second.empty())
  {
    std::printf("Skipping timesync because the logfile does not contain GNSS data\n");
    return;
  }

  // Target the first available GPS instance (typically Instance 0)
  const auto& first_gps_instance = msg_it->second.begin()->second;

  const auto field_indices = resolve_gps_field_indices(field_name2idx);
  if (!field_indices)
  {
    return;
  }

  const auto& time_vec  = first_gps_instance.at(field_indices->time_idx).second;
  const auto& week_vec  = first_gps_instance.at(field_indices->week_idx).second;
  const auto& ms_vec    = first_gps_instance.at(field_indices->ms_idx).second;
  const auto& nsats_vec = first_gps_instance.at(field_indices->nsats_idx).second;

  const auto valid_sample_idx =
      find_first_valid_gps_sample(time_vec, week_vec, ms_vec, nsats_vec, MIN_VALID_NSATS);

  if (!valid_sample_idx)
  {
    std::printf("Skipping timesync because no sequential GPS sample with a valid fix was found\n");
    return;
  }

  const double gps_week    = week_vec[*valid_sample_idx];
  const double gps_week_ms = ms_vec[*valid_sample_idx];

  const double log_time_sec = time_vec[*valid_sample_idx];

  const double unix_time_sec   = gps_to_unix_time(gps_week, gps_week_ms);
  const double time_offset_sec = unix_time_sec - log_time_sec;

  // Expose the offset for the status-text / parameter / message-timeline series
  _start_plot_time = log_time_sec;
  _time_offset     = time_offset_sec;

  shift_all_timestamps(messages_map, field_name2idx, time_offset_sec);
}
