// Headless smoke test for a PlotJuggler DataLoader plugin.
//
// Usage: pj_plugin_smoketest <plugin> [logfile.bin] [--verify] [--expect-units] [--expect-rcou]
//
// Exercises exactly what PlotJuggler does when it loads a plugin and opens a
// file, but without a GUI and without ever blocking: QPluginLoader, the
// qobject_cast to PJ::DataLoader, then readDataFromFile() on a real log.
//
// Without --verify the build configuration is only reported. With --verify it is
// asserted: the plugin must have been built with exactly the options named by
// --expect-units / --expect-rcou, and any option not named must be OFF. Both
// options work by changing series names, so the name of one known channel pins
// the build down exactly. Asserting the all-OFF case matters as much as the
// others -- an option wrongly forced on in the source looks fine otherwise.
//
// Exit code 0 means every requested check passed.

#include <algorithm>
#include <cstdio>
#include <exception>
#include <string>
#include <vector>

#include <QApplication>
#include <QFileInfo>
#include <QPluginLoader>

#include "PlotJuggler/dataloader_base.h"
#include "PlotJuggler/plotdata.h"

namespace
{
// tests/logs/00000001.BIN has SERVO1_FUNCTION set to Throttle, so this channel
// carries a function label whenever LABEL_RCOU_FUNCTION is compiled in, and a
// microsecond unit whenever LABEL_WITH_UNIT is. That makes exactly one of the
// four spellings below correct for any given build.
const char* const kChannel = "/RCOU/C1";
const char* const kFunction = " (Throttle)";
const char* const kUnit = "\t[us]";

std::string expectedName(bool with_units, bool with_rcou)
{
  std::string name = kChannel;
  if (with_rcou)
  {
    name += kFunction;
  }
  if (with_units)
  {
    name += kUnit;
  }
  return name;
}

std::string describe(bool with_units, bool with_rcou)
{
  return std::string("units=") + (with_units ? "ON" : "OFF") + " rcou=" + (with_rcou ? "ON" : "OFF");
}

void printUsage(const char* argv0)
{
  fprintf(stderr,
          "usage: %s <plugin> [logfile.bin] [--verify] [--expect-units] [--expect-rcou]\n",
          argv0);
}
}  // namespace

int main(int argc, char** argv)
{
  std::string plugin_path;
  std::string log_path;
  bool expect_units = false;
  bool expect_rcou = false;
  bool verify = false;

  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if (arg == "--verify")
    {
      verify = true;
    }
    else if (arg == "--expect-units")
    {
      expect_units = true;
    }
    else if (arg == "--expect-rcou")
    {
      expect_rcou = true;
    }
    else if (arg == "-h" || arg == "--help")
    {
      printUsage(argv[0]);
      return 0;
    }
    else if (arg.rfind("--", 0) == 0)
    {
      fprintf(stderr, "unknown option: %s\n", arg.c_str());
      printUsage(argv[0]);
      return 2;
    }
    else if (plugin_path.empty())
    {
      plugin_path = arg;
    }
    else if (log_path.empty())
    {
      log_path = arg;
    }
    else
    {
      printUsage(argv[0]);
      return 2;
    }
  }

  if (plugin_path.empty())
  {
    printUsage(argv[0]);
    return 2;
  }
  if (log_path.empty() && (verify || expect_units || expect_rcou))
  {
    fprintf(stderr, "FAIL: --verify needs a log file to check series names against\n");
    return 2;
  }
  if ((expect_units || expect_rcou) && !verify)
  {
    fprintf(stderr, "FAIL: --expect-* only means something with --verify\n");
    return 2;
  }

  // A QApplication must exist before the plugin is instantiated: the plugin
  // links Qt Widgets, and constructing a QWidget without one aborts.
  QApplication app(argc, argv);

  const QFileInfo plugin_info(QString::fromStdString(plugin_path));
  if (!plugin_info.isFile())
  {
    fprintf(stderr, "FAIL: no such plugin file: %s\n", plugin_path.c_str());
    return 2;
  }

  // Must be absolute: QPluginLoader resolves a relative path against
  // QCoreApplication::libraryPaths(), never the working directory, so a
  // perfectly good relative path is reported as "The shared library was not
  // found." -- see locatePlugin() in qpluginloader.cpp.
  const QString qplugin = plugin_info.absoluteFilePath();

  QPluginLoader loader(qplugin);
  QObject* instance = loader.instance();
  if (!instance)
  {
    // This is where an ABI mismatch surfaces, e.g. a missing GLIBCXX version.
    fprintf(stderr, "FAIL: could not load %s\n      %s\n", qPrintable(qplugin),
            qPrintable(loader.errorString()));
    return 1;
  }

  auto* dataloader = qobject_cast<PJ::DataLoader*>(instance);
  if (!dataloader)
  {
    fprintf(stderr, "FAIL: library loaded but is not a PJ::DataLoader plugin\n");
    return 1;
  }

  printf("OK  : loaded DataLoader plugin \"%s\"\n", dataloader->name());
  printf("      extensions:");
  for (const char* ext : dataloader->compatibleFileExtensions())
  {
    printf(" %s", ext);
  }
  printf("\n");

  if (log_path.empty())
  {
    printf("OK  : load-only check passed (pass a log file to also test parsing)\n");
    return 0;
  }

  const QString qlog = QString::fromStdString(log_path);
  if (!QFileInfo::exists(qlog))
  {
    fprintf(stderr, "FAIL: no such log file: %s\n", log_path.c_str());
    return 2;
  }

  PJ::PlotDataMapRef data;
  PJ::FileLoadInfo info;
  info.filename = qlog;

  bool parsed = false;
  try
  {
    parsed = dataloader->readDataFromFile(&info, data);
  }
  catch (const std::exception& err)
  {
    fprintf(stderr, "FAIL: exception while parsing: %s\n", err.what());
    return 1;
  }

  if (!parsed)
  {
    fprintf(stderr, "FAIL: readDataFromFile() returned false\n");
    return 1;
  }

  size_t samples = 0;
  for (const auto& series : data.numeric)
  {
    samples += series.second.size();
  }

  printf("OK  : parsed \"%s\"\n", qPrintable(QFileInfo(qlog).fileName()));
  printf("      numeric series: %zu (%zu samples)\n", data.numeric.size(), samples);
  printf("      string series : %zu\n", data.strings.size());

  if (data.numeric.empty())
  {
    fprintf(stderr, "FAIL: parsing reported success but produced no numeric series\n");
    return 1;
  }

  // Which of the four spellings of the reference channel is present tells us
  // exactly which ifdefs were compiled in. Requiring the other three to be
  // absent is what catches a flag stuck on -- or stuck off -- in the source.
  const std::string wanted = expectedName(expect_units, expect_rcou);
  std::vector<std::string> found;
  for (bool units : { false, true })
  {
    for (bool rcou : { false, true })
    {
      const std::string candidate = expectedName(units, rcou);
      if (data.numeric.count(candidate) != 0)
      {
        found.push_back(candidate);
        printf("      built with    : %s\n", describe(units, rcou).c_str());
      }
    }
  }

  if (found.empty())
  {
    fprintf(stderr, "FAIL: reference channel %s not found in this log under any\n", kChannel);
    fprintf(stderr, "      expected name. Names starting with %s:\n", kChannel);
    std::vector<std::string> nearby;
    for (const auto& series : data.numeric)
    {
      if (series.first.rfind(kChannel, 0) == 0)
      {
        nearby.push_back(series.first);
      }
    }
    std::sort(nearby.begin(), nearby.end());
    for (const auto& name : nearby)
    {
      fprintf(stderr, "        %s\n", name.c_str());
    }
    if (nearby.empty())
    {
      fprintf(stderr, "        (none)\n");
    }
    return 1;
  }

  if (found.size() > 1)
  {
    fprintf(stderr, "FAIL: reference channel matched %zu spellings at once, so the\n", found.size());
    fprintf(stderr, "      naming is ambiguous and this check cannot be trusted\n");
    return 1;
  }

  if (!verify)
  {
    // Nothing was asserted, so just report what the build looks like.
    printf("OK  : parse check passed (pass --verify to assert the build\n");
    printf("      configuration as well)\n");
    return 0;
  }

  if (found.front() != wanted)
  {
    fprintf(stderr, "FAIL: wrong build configuration\n");
    fprintf(stderr, "      expected %s -> \"%s\"\n", describe(expect_units, expect_rcou).c_str(),
            wanted.c_str());
    fprintf(stderr, "      actual            -> \"%s\"\n", found.front().c_str());
    fprintf(stderr, "      ADD_UNITS/ADD_RCOU_FUNCTION_LABELS are not reaching the binary\n");
    return 1;
  }

  printf("OK  : build configuration matches %s\n", describe(expect_units, expect_rcou).c_str());
  return 0;
}
