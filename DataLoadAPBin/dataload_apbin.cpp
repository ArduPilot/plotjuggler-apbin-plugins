#include "dataload_apbin.h"
#include <QFile>
#include <QMessageBox>
#include <QProgressDialog>
#include <QDateTime>
#include <QInputDialog>

DataLoadAPBIN::DataLoadAPBIN()
{
  _extensions.push_back("BIN");  // TODO : this doesn't work for now as tolower() is hardcoded.
}

const std::vector<const char*>& DataLoadAPBIN::compatibleFileExtensions() const
{
  return _extensions;
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

  // Progress box for large file
  QProgressDialog progress_dialog;
  progress_dialog.setLabelText("Loading... please wait");
  progress_dialog.setWindowModality(Qt::ApplicationModal);
  progress_dialog.setRange(0, 100);
  progress_dialog.setAutoClose(true);
  progress_dialog.setAutoReset(true);
  progress_dialog.show();

  const uint8_t* buf = reinterpret_cast<const uint8_t*>(file_array.data());
  const uint32_t len = file_array.size();
  uint32_t total_bytes_used = 0;
  while (true)
  {
    if (len - total_bytes_used < 3)  // less that the smallest so we end
    {
      break;
    }

    // skip through input until we find a valid header:
    if (buf[total_bytes_used] != HEAD_BYTE1 || buf[total_bytes_used + 1] != HEAD_BYTE2)
    {
      total_bytes_used += 1;
      continue;
    }
    // get the message type
    const uint8_t type = buf[total_bytes_used + 2];

    if (type == LOG_FORMAT_MSG)
    {
      //::fprintf(stderr, "Found LOG_FORMAT_MESSAGE\n");
      // check if we don't reach the end
      if ((uint32_t)(len - total_bytes_used) < sizeof(struct log_Format))
      {
        break;
      }
      // extract the FMT type
      const uint8_t defining_type = ((struct log_Format*)(&(buf[total_bytes_used])))->type;
      struct log_Format& f = formats[defining_type];
      memcpy(&f, &buf[total_bytes_used], sizeof(struct log_Format));
      for (char i : f.name)
      {
        if (!isprint(i) && i != '\0')
        {
          // double check that this is a message
          // name is assumed to be printable ascii; it
          // looked like a format message, but wasn't.
          total_bytes_used++;
          continue;
        }
      }
      //::fprintf(stderr, "Format %u = %s  length=%u\n", f.type, f.name, f.length);
      total_bytes_used += sizeof(struct log_Format);
      continue;
    }
    // get the full log format from the message type
    const struct log_Format& format = formats[type];
    // discard some messages that aren't numbers : PARAM (64), MSG (91), UNITS (177), MULTI (178)
    // remove unknown format
    if (format.type == 64 || format.type == 91 || format.type == 177 || format.type == 178 || format.length == 0)
    {
      total_bytes_used += 1;
      continue;
    }
    // if we are under the message length remaining, just end
    if (len - total_bytes_used < format.length)
    {
      break;
    }
    //::fprintf(stderr, "Received message of type %u\n", type);
    handle_message_received(format, &buf[total_bytes_used]);
    total_bytes_used += format.length;
    const auto tempProgress = static_cast<int>((static_cast<double>(total_bytes_used) / static_cast<double>(file_size)) * 100.0);
    // update the progression dialog box
    progress_dialog.setValue(tempProgress);
    QApplication::processEvents();
  }

  // convert from timeserie to plotjuggler plot format
  for (const auto& it : _timeseries_map)
  {
    const std::string& message_name = it.first;
    const Timeseries& timeseries = it.second;

    for (const auto& data : timeseries.data)
    {
      const std::string series_name = "/" + message_name + "/" + data.first;

      auto series = plot_data.addNumeric(series_name);

      for (size_t i = 0; i < data.second.size(); i++)
      {
        const double msg_time = static_cast<double>(timeseries.timestamps[i]) * 0.000001;
        PlotData::Point point(msg_time, data.second[i]);
        series->second.pushBack(point);
      }
    }
  }

  file.close();

  return true;
}

void DataLoadAPBIN::handle_message_received(const struct log_Format& format, const uint8_t* msg)
{
  uint8_t name_lenght = 0;
  for (char i : format.name)
  {
    if (i != '\0')
    {
      // schrödinger's name ... it can have null termination or not
      name_lenght++;
    }
  }
  std::string fname(format.name, name_lenght);
  // get the timeseries map or create if it doesn't exist
  auto ts_it = _timeseries_map.find(fname);
  if (ts_it == _timeseries_map.end())
  {  // TODO get the max size message from FTM, divide the remaining size to parse by the max size. reserve a block of
    // memory for each message.
    ts_it = _timeseries_map.insert({ fname, createTimeseries(format) }).first;
  }
  Timeseries& timeseries = ts_it->second;
  uint32_t msg_offset = 3;  // discard header
  // get the timestamps that is assumed to be the first field // TODO: correct that
  uint64_t msg_time{ 0 };
  memcpy(&msg_time, &msg[msg_offset], sizeof(uint64_t));
  timeseries.timestamps.push_back(msg_time);

  double value{ 0 };
  // for each label, we get the data according to the format length
  // this use pointer arithmetic, so just close your eyes
  for (int i = 0; i < timeseries.data.size(); i++)
  {
    const char typeCode = format.format[i];
    switch (typeCode)
    {
      case 'b':
        value = static_cast<double>(*reinterpret_cast<const int8_t*>(msg + msg_offset));
        msg_offset += sizeof(int8_t);
        break;
      case 'c':
        value = static_cast<double>(*reinterpret_cast<const int16_t*>(msg + msg_offset));
        msg_offset += sizeof(int16_t);
        break;
      case 'e':
        value = static_cast<double>(*reinterpret_cast<const int32_t*>(msg + msg_offset));
        msg_offset += sizeof(int32_t);
        break;
      case 'f':
        value = static_cast<double>(*reinterpret_cast<const float*>(msg + msg_offset));
        msg_offset += sizeof(float);
        break;
      case 'h':
        value = static_cast<double>(*reinterpret_cast<const int16_t*>(msg + msg_offset));
        msg_offset += sizeof(int16_t);
        break;
      case 'i':
        value = static_cast<double>(*reinterpret_cast<const int32_t*>(msg + msg_offset));
        msg_offset += sizeof(int32_t);
        break;
      case 'n':
        // not used, that is for MSG or PARAM
        // value = static_cast<double>(*reinterpret_cast<const char[4]>(msg + msg_offset));
        msg_offset += sizeof(char[4]);
        break;
      case 'B':
        value = static_cast<double>(*reinterpret_cast<const uint8_t*>(msg + msg_offset));
        msg_offset += sizeof(uint8_t);
        break;
      case 'C':
        value = static_cast<double>(*reinterpret_cast<const uint16_t*>(msg + msg_offset));
        msg_offset += sizeof(uint16_t);
        break;
      case 'E':
        value = static_cast<double>(*reinterpret_cast<const uint32_t*>(msg + msg_offset));
        msg_offset += sizeof(uint32_t);
        break;
      case 'H':
        value = static_cast<double>(*reinterpret_cast<const uint16_t*>(msg + msg_offset));
        msg_offset += sizeof(uint16_t);
        break;
      case 'I':
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
      case 'N':
        // not used, that is for MSG or PARAM
        // value = static_cast<double>(*reinterpret_cast<const char[16]>(msg + msg_offset));
        msg_offset += sizeof(char[16]);
        break;
      case 'Z':
        // not used, that is for MSG or PARAM
        // value = static_cast<double>(*reinterpret_cast<const char[64]>(msg + msg_offset));
        msg_offset += sizeof(char[64]);
        break;
      case 'q':
        value = static_cast<double>(*reinterpret_cast<const int64_t*>(msg + msg_offset));
        msg_offset += sizeof(int64_t);
        break;
      case 'Q':
        value = static_cast<double>(*reinterpret_cast<const uint64_t*>(msg + msg_offset));
        msg_offset += sizeof(uint64_t);
        break;
    }

    timeseries.data[i].second.push_back(value);
  }
}

DataLoadAPBIN::Timeseries DataLoadAPBIN::createTimeseries(const struct log_Format& format)
{
  QString tmpStr(format.labels);
  tmpStr.truncate(MAX_LABELS_SIZE);
  QStringList labels_list;
  if (tmpStr.size() > 0)
  {
    labels_list = tmpStr.split(",");
  }
  Timeseries timeseries;
  timeseries.data.reserve(labels_list.size());
  for (auto i = 0; i < labels_list.size(); i++)
  {
    timeseries.data.emplace_back(labels_list.at(i).toLocal8Bit().constData(), std::vector<double>());
  }
  return timeseries;
}