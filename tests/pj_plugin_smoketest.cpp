// Headless test through PlotJuggler 4's SDK loader and service registry.
//
// Usage: pj_plugin_smoketest <plugin> [logfile.bin] [--verify] [--expect-units]
// [--expect-rcou]
//
// Without --verify, the helper checks loading, parsing and
// lifecycle behavior. With --verify, it also asserts the exact label
// configuration and a known sample from tests/logs/00000001.BIN. Exit code 0
// means every requested check passed.
#include <QCoreApplication>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <optional>
#include <pj_base/sdk/plugin_data_api.hpp>
#include <pj_plugins/host/data_source_library.hpp>
#include <pj_plugins/host/service_registry_builder.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
std::string str(PJ_string_view_t s) {
  return std::string(s.data ? s.data : "", s.size);
}
void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}
void ok(PJ::Status status) {
  if (!status) throw std::runtime_error(status.error());
}
using Samples = std::vector<std::pair<int64_t, double>>;
void compareSeries(const std::map<std::string, Samples>& expected,
                   const std::map<std::string, Samples>& actual) {
  require(expected.size() == actual.size(),
          "repeated import changed the number of series");
  for (const auto& [name, samples] : expected) {
    require(actual.count(name) != 0 && actual.at(name).size() == samples.size(),
            "repeated import changed " + name);
    const auto& loaded = actual.at(name);
    for (size_t i = 0; i < samples.size(); ++i) {
      // ArduPilot records unavailable values as NaN, which compares unequal to
      // itself.
      require(
          samples[i].first == loaded[i].first &&
              (samples[i].second == loaded[i].second ||
               (std::isnan(samples[i].second) && std::isnan(loaded[i].second))),
          "repeated import changed sample " + std::to_string(i) + " of " +
              name);
    }
  }
}
struct RecordingHost {
  std::map<uint32_t, std::string> topics;
  std::map<std::pair<uint32_t, uint32_t>, std::string> fields;
  std::map<std::string, Samples> series;
  std::map<std::string, std::map<std::string, std::optional<double>>> last_rows;
  std::string metadata;
  bool cancel = false, cancel_progress = false, fail_writes = false,
       progress_finished = false;
  uint64_t progress_total = 0, progress_step = 0;
  static bool ensureTopic(void* ctx, PJ_string_view_t name,
                          PJ_topic_handle_t* out, PJ_error_t* error) noexcept {
    auto& h = *static_cast<RecordingHost*>(ctx);
    if (h.fail_writes) {
      PJ::sdk::fillError(error, 1, "test", "write refused");
      return false;
    }
    for (const auto& [id, topic] : h.topics)
      if (topic == str(name)) {
        *out = {id};
        return true;
      }
    out->id = static_cast<uint32_t>(h.topics.size() + 1);
    h.topics[out->id] = str(name);
    return true;
  }
  static bool ensureField(void* ctx, PJ_topic_handle_t topic,
                          PJ_string_view_t name, PJ_primitive_type_t type,
                          PJ_field_handle_t* out, PJ_error_t*) noexcept {
    auto& h = *static_cast<RecordingHost*>(ctx);
    if (type != PJ_PRIMITIVE_TYPE_FLOAT64 || !h.topics.count(topic.id))
      return false;
    for (const auto& [key, field] : h.fields)
      if (key.first == topic.id && field == str(name)) {
        *out = {topic, key.second};
        return true;
      }
    const auto id = static_cast<uint32_t>(h.fields.size() + 1);
    h.fields[{topic.id, id}] = str(name);
    *out = {topic, id};
    return true;
  }
  static bool appendBound(void* ctx, PJ_topic_handle_t topic, int64_t ts,
                          const PJ_bound_field_value_t* values, uint64_t count,
                          PJ_error_t*) noexcept {
    auto& h = *static_cast<RecordingHost*>(ctx);
    // The datastore fills omitted fields with null in each row.
    auto& row = h.last_rows[h.topics.at(topic.id)];
    row.clear();
    for (const auto& [key, name] : h.fields)
      if (key.first == topic.id) row[name] = std::nullopt;
    for (uint64_t i = 0; i < count; ++i) {
      const auto& v = values[i];
      const auto key = std::make_pair(topic.id, v.field.id);
      if (v.field.topic.id != topic.id || !h.fields.count(key) ||
          v.value.type != PJ_PRIMITIVE_TYPE_FLOAT64)
        return false;
      if (!v.is_null) {
        row[h.fields.at(key)] = v.value.data.as_float64;
        h.series[h.topics.at(topic.id) + "/" + h.fields.at(key)].emplace_back(
            ts, v.value.data.as_float64);
      }
    }
    return true;
  }
  static bool appendNamed(void* ctx, PJ_topic_handle_t topic, int64_t ts,
                          const PJ_named_field_value_t* values, uint64_t count,
                          PJ_error_t* error) noexcept {
    std::vector<PJ_bound_field_value_t> bound;
    bound.reserve(count);
    for (uint64_t i = 0; i < count; ++i) {
      PJ_field_handle_t field{};
      if (!ensureField(ctx, topic, values[i].name, values[i].value.type, &field,
                       error))
        return false;
      bound.push_back({.field = field,
                       .is_null = values[i].is_null,
                       .value = values[i].value});
    }
    return appendBound(ctx, topic, ts, bound.data(), bound.size(), error);
  }
  static PJ_source_write_host_vtable_t writeVtable() {
    PJ_source_write_host_vtable_t vt{};
    vt.abi_version = PJ_ABI_VERSION;
    vt.struct_size = sizeof(vt);
    vt.ensure_topic = ensureTopic;
    vt.ensure_field = ensureField;
    vt.append_record = appendNamed;
    vt.append_bound_record = appendBound;
    return vt;
  }
  static PJ_data_source_runtime_host_vtable_t runtimeVtable() {
    PJ_data_source_runtime_host_vtable_t vt{};
    vt.protocol_version = 1;
    vt.struct_size = sizeof(vt);
    vt.progress_start = [](void* ctx, PJ_string_view_t, uint64_t total, bool,
                           PJ_error_t*) noexcept {
      auto& h = *static_cast<RecordingHost*>(ctx);
      h.progress_total = total;
      h.progress_finished = false;
      return true;
    };
    vt.progress_update = [](void* ctx, uint64_t step) noexcept {
      auto& h = *static_cast<RecordingHost*>(ctx);
      h.progress_step = step;
      return !h.cancel_progress;
    };
    vt.progress_finish = [](void* ctx) noexcept {
      static_cast<RecordingHost*>(ctx)->progress_finished = true;
    };
    vt.is_stop_requested = [](void* ctx) noexcept {
      return static_cast<RecordingHost*>(ctx)->cancel;
    };
    vt.set_dataset_metadata = [](void* ctx, PJ_string_view_t json,
                                 PJ_error_t*) noexcept {
      static_cast<RecordingHost*>(ctx)->metadata = str(json);
      return true;
    };
    return vt;
  }
};
std::string config(const QString& path) {
  return QJsonDocument(QJsonObject{{"filepath", path}})
      .toJson(QJsonDocument::Compact)
      .toStdString();
}
void addFormat(QByteArray& bytes, uint8_t type, uint8_t length,
               const char* name, const char* format, const char* labels) {
  QByteArray fmt(89, '\0');
  fmt[0] = char(0xa3);
  fmt[1] = char(0x95);
  fmt[2] = char(128);
  fmt[3] = char(type);
  fmt[4] = char(length);
  std::memcpy(fmt.data() + 5, name, std::min<size_t>(4, std::strlen(name)));
  std::memcpy(fmt.data() + 9, format, std::strlen(format));
  std::memcpy(fmt.data() + 25, labels, std::strlen(labels));
  bytes += fmt;
}
template <typename T>
void appendLE(QByteArray& bytes, T value) {
  bytes.append(reinterpret_cast<const char*>(&value), sizeof(value));
}
QString syntheticLog(const QString& directory, bool modern_msg,
                     bool gps = false, uint64_t time_us = 2000000) {
  QByteArray bytes;
  addFormat(bytes, 42, 15, "TEST", "Qf", "TimeUS,Value");
  addFormat(bytes, 43, modern_msg ? 77 : 75, "MSG", modern_msg ? "QBBZ" : "QZ",
            modern_msg ? "TimeUS,ID,Seq,Message" : "TimeUS,Message");
  if (gps) addFormat(bytes, 44, 18, "GPS", "QHIB", "TimeUS,GWk,GMS,NSats");
  bytes.append(char(0xa3));
  bytes.append(char(0x95));
  bytes.append(char(42));
  appendLE(bytes, time_us);
  appendLE(bytes, float(42.5));
  if (gps) {
    bytes.append(char(0xa3));
    bytes.append(char(0x95));
    bytes.append(char(44));
    appendLE(bytes, uint64_t(2000000));
    appendLE(bytes, uint16_t(2200));
    appendLE(bytes, uint32_t(100000));
    bytes.append(char(6));
  }
  bytes.append(char(0xa3));
  bytes.append(char(0x95));
  bytes.append(char(43));
  appendLE(bytes, uint64_t(3000000));
  if (modern_msg) {
    bytes.append(char(7));
    bytes.append(char(0));
  }
  QByteArray text(64, '\0');
  std::memcpy(text.data(), "hello/world", 11);
  bytes += text;
  const auto path = directory + (modern_msg ? "/modern.BIN" : "/legacy.bin");
  QFile file(path);
  require(file.open(QFile::WriteOnly), "cannot create fixture");
  require(file.write(bytes) == bytes.size(), "cannot write fixture");
  return path;
}
QString snapshotOnlyLog(const QString& directory, bool parameters,
                        bool messages) {
  QByteArray bytes;
  addFormat(bytes, 45, 35, "PARM", "QNff", "TimeUS,Name,Value,Default");
  addFormat(bytes, 43, 75, "MSG", "QZ", "TimeUS,Message");
  if (parameters) {
    bytes.append(char(0xa3));
    bytes.append(char(0x95));
    bytes.append(char(45));
    appendLE(bytes, uint64_t(2000000));
    QByteArray name(16, '\0');
    std::memcpy(name.data(), "SNAPSHOT", 8);
    bytes += name;
    appendLE(bytes, float(42.5));
    appendLE(bytes, float(0));
  }
  if (messages) {
    bytes.append(char(0xa3));
    bytes.append(char(0x95));
    bytes.append(char(43));
    appendLE(bytes, uint64_t(3000000));
    QByteArray text(64, '\0');
    std::memcpy(text.data(), "snapshot/message", 16);
    bytes += text;
  }
  const auto path = directory + "/snapshots.bin";
  QFile file(path);
  require(file.open(QFile::WriteOnly), "cannot create snapshot fixture");
  require(file.write(bytes) == bytes.size(), "cannot write snapshot fixture");
  return path;
}
QString repeatedParameterLog(const QString& directory) {
  const auto path = syntheticLog(directory, false);
  QByteArray bytes;
  addFormat(bytes, 45, 35, "PARM", "QNff", "TimeUS,Name,Value,Default");
  // Unequal occurrence counts exercise null cells in grouped parameter rows.
  for (const auto& [name, value] :
       {std::pair{"STATIC", 10.0f}, std::pair{"TWICE", 20.0f},
        std::pair{"THRICE", 30.0f}, std::pair{"TWICE", 21.0f},
        std::pair{"THRICE", 31.0f}, std::pair{"THRICE", 32.0f}}) {
    bytes.append(char(0xa3));
    bytes.append(char(0x95));
    bytes.append(char(45));
    appendLE(bytes, uint64_t(4000000));
    QByteArray param_name(16, '\0');
    std::memcpy(param_name.data(), name, std::strlen(name));
    bytes += param_name;
    appendLE(bytes, value);
    appendLE(bytes, 0.0f);
  }
  QFile file(path);
  require(file.open(QFile::Append), "cannot append parameter fixture");
  require(file.write(bytes) == bytes.size(), "cannot write parameter fixture");
  return path;
}
}  // namespace
int main(int argc, char** argv) {
  QCoreApplication app(argc, argv);
  try {
    require(argc >= 2,
            "usage: pj_plugin_smoketest <plugin> [log] [--verify] "
            "[--expect-units] [--expect-rcou]");
    bool verify = false, units = false, rcou = false;
    QString log;
    for (int i = 2; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--verify")
        verify = true;
      else if (arg == "--expect-units")
        units = true;
      else if (arg == "--expect-rcou")
        rcou = true;
      else if (log.isEmpty() && !arg.starts_with("--"))
        log = QString::fromLocal8Bit(argv[i]);
      else
        throw std::runtime_error("unknown argument: " + arg);
    }
    require(verify || (!units && !rcou), "--expect-* requires --verify");
    auto library = PJ::DataSourceLibrary::load(std::filesystem::path(argv[1]));
    require(library.has_value(), library ? "" : library.error());
    auto source = library->createHandle();
    require(source.valid(), "plugin instance creation failed");
    const auto manifest =
        QJsonDocument::fromJson(QByteArray::fromStdString(source.manifest()))
            .object();
    require(manifest.value("id").toString() == "ardupilot-bin",
            "wrong plugin id");
    require(manifest.value("file_extensions").toArray().contains(".bin"),
            "missing .bin extension");
    require(source.capabilities() ==
                (PJ::kCapabilityFiniteImport | PJ::kCapabilityDirectIngest),
            "wrong capabilities");
    std::puts("OK: loaded PlotJuggler 4 ArduPilot Bin data source");
    if (log.isEmpty()) {
      require(!verify, "--verify requires the reference log");
      return 0;
    }
    RecordingHost host;
    const auto write_vt = RecordingHost::writeVtable();
    const auto runtime_vt = RecordingHost::runtimeVtable();
    PJ::ServiceRegistryBuilder registry;
    registry.registerService("pj.source_write.v1", 1, {&host, &write_vt});
    registry.registerService("pj.runtime.v1", 1, {&host, &runtime_vt});
    ok(source.bind(registry.view()));
    require(!source.loadConfig("{"), "invalid JSON accepted");
    require(!source.loadConfig("{}"), "missing filepath accepted");
    require(!source.loadConfig("{\"filepath\":42}"),
            "non-string filepath accepted");
    ok(source.loadConfig(config(QFileInfo(log).absoluteFilePath())));
    std::string saved;
    ok(source.saveConfig(saved));
    require(QJsonDocument::fromJson(QByteArray::fromStdString(saved))
                    .object()
                    .value("filepath")
                    .toString() == QFileInfo(log).absoluteFilePath(),
            "config roundtrip failed");
    ok(source.start());
    require(source.currentState() == PJ::DataSourceState::kStopped,
            "finite import did not finish");
    require(host.progress_finished && host.progress_step == host.progress_total,
            "progress did not finish");
    require(!host.series.empty(), "no samples loaded");
    // The reference log has SERVO1_FUNCTION=Throttle and RCOU.C1 in
    // microseconds. Its full name therefore identifies exactly which of the
    // four label builds is loaded, including the all-OFF case. A flag forced ON
    // or OFF fails here.
    std::string wanted = "/RCOU/C1";
    if (rcou) wanted += " (Throttle)";
    if (units) wanted += "\t[us]";
    if (verify)
      require(host.series.count(wanted) && !host.series.at(wanted).empty(),
              "wrong build labels: expected " + wanted);
    // Independently decoded first RCOU sample: TimeUS=28416489, C1=1002.
    if (verify)
      require(host.series.at(wanted).front() ==
                  std::pair<int64_t, double>{28416489000LL, 1002.0},
              "reference RCOU sample changed");
    require(!QJsonDocument::fromJson(QByteArray::fromStdString(host.metadata))
                 .object()
                 .value("field_descriptions")
                 .toObject()
                 .isEmpty(),
            "embedded descriptions missing");
    const auto first = host.series;
    host = RecordingHost{};
    ok(source.start());
    compareSeries(first, host.series);
    QTemporaryDir directory;
    require(directory.isValid(), "temporary directory unavailable");
    host = RecordingHost{};
    ok(source.loadConfig(config(repeatedParameterLog(directory.path()))));
    ok(source.start());
    const auto& parameter_row = host.last_rows.at("/1. Parameters");
    require(parameter_row.size() == 3 &&
                parameter_row.at("STATIC          Default: 0") == 10.0 &&
                parameter_row.at("TWICE          Default: 0") == 21.0 &&
                parameter_row.at("THRICE          Default: 0") == 32.0,
            "repeated parameters hide values in the latest snapshot");
    const auto& changes =
        host.series.at("/1. Parameters/TWICE          Default: 0");
    require(
        changes.size() >= 2 &&
            changes[0] == std::pair<int64_t, double>{300000000000LL, 20.0} &&
            changes[1] == std::pair<int64_t, double>{300000000000LL, 21.0},
        "parameter changes or snapshot timestamp lost");
    // Short logs can contain useful parameters/messages without numeric data.
    for (const auto [parameters, messages] :
         {std::pair{true, false}, std::pair{false, true},
          std::pair{true, true}}) {
      host = RecordingHost{};
      ok(source.loadConfig(
          config(snapshotOnlyLog(directory.path(), parameters, messages))));
      ok(source.start());
      require(source.currentState() == PJ::DataSourceState::kStopped &&
                  host.progress_finished &&
                  host.progress_step == host.progress_total,
              "snapshot-only import did not finish");
      require(
          host.series.size() == (parameters ? 1u : 0u) + (messages ? 2u : 0u),
          "snapshot-only import lost data or leaked previous series");
      if (parameters)
        require(host.series.at("/1. Parameters/SNAPSHOT          Default: 0") ==
                    Samples{{300000000000LL, 42.5}},
                "parameter-only value or timestamp lost");
      if (messages) {
        require(host.series.at("/2. Message Timeline/index") ==
                    Samples{{3000000000LL, 1.0}},
                "message-only timeline lost");
        require(host.series.at("/3. Message Log/1. snapshot\\message") ==
                    Samples{{0, 3.0}},
                "message-only text or relative time lost");
      }
    }
    host = RecordingHost{};
    ok(source.loadConfig(
        config(snapshotOnlyLog(directory.path(), false, false))));
    require(!source.start() && host.series.empty() && host.progress_finished,
            "format-only log without supported data accepted");
    // A GNSS-aligned import followed by a boot-time import checks offset reset.
    for (const auto [modern, gps] :
         {std::pair{false, false}, std::pair{true, true},
          std::pair{true, false}}) {
      const auto path = syntheticLog(directory.path(), modern, gps);
      host = RecordingHost{};
      ok(source.loadConfig(config(path)));
      ok(source.start());
      const int64_t time_ns = gps ? 1646524882000000000LL : 2000000000LL;
      require(host.series.at("/TEST/Value") == Samples{{time_ns, 42.5}},
              "TimeUS/GPS conversion or state reset failed");
      require(host.series.at("/2. Message Timeline/index") ==
                  Samples{{time_ns + 1000000000LL, 1.0}},
              "MSG layout or stale GPS offset");
      require(host.series.at("/3. Message Log/1. hello\\world") ==
                  Samples{{0, gps ? 1.0 : 3.0}},
              "MSG text or stale start time");
      require(host.series.size() == (gps ? 6u : 3u),
              "parameters/messages leaked from previous log");
    }
    host = RecordingHost{};
    host.cancel = true;
    require(!source.start() &&
                source.currentState() == PJ::DataSourceState::kFailed &&
                host.series.empty(),
            "stop request ignored");
    host = RecordingHost{};
    host.cancel_progress = true;
    require(!source.start() && host.progress_finished && host.series.empty(),
            "progress cancellation ignored");
    host = RecordingHost{};
    host.fail_writes = true;
    require(!source.start() &&
                source.currentState() == PJ::DataSourceState::kFailed,
            "host write error ignored");
    host = RecordingHost{};
    ok(source.loadConfig(config(directory.path() + "/missing.bin")));
    require(!source.start(), "missing file accepted");
    const auto empty = directory.path() + "/empty.bin";
    {
      QFile f(empty);
      require(f.open(QFile::WriteOnly), "cannot create empty fixture");
    }
    ok(source.loadConfig(config(empty)));
    require(!source.start(), "empty file accepted");
    host = RecordingHost{};
    ok(source.loadConfig(
        config(syntheticLog(directory.path(), false, false, UINT64_MAX))));
    require(!source.start() &&
                source.currentState() == PJ::DataSourceState::kFailed &&
                host.progress_finished,
            "timestamp overflow or exception cleanup failed");
    std::printf(
        "OK: real log (%zu series), repeated imports and parameters, "
        "snapshot-only logs, both MSG "
        "layouts, "
        "nanosecond timestamps, config, cancellation and errors\n",
        first.size());
    return 0;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "FAIL: %s\n", error.what());
    return 1;
  }
}
