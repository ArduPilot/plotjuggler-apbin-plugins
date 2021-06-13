#include "dataload_simple_csv.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QDebug>
#include <QSettings>
#include <QProgressDialog>
#include <QDateTime>
#include <QInputDialog>

DataLoadSimpleCSV::DataLoadSimpleCSV()
{
  _extensions.push_back("pj_csv");
}

const QRegExp csv_separator("(\\,)");

const std::vector<const char*>& DataLoadSimpleCSV::compatibleFileExtensions() const
{
  return _extensions;
}

bool DataLoadSimpleCSV::readDataFromFile(FileLoadInfo* info, PlotDataMapRef& plot_data)
{
  QFile file(info->filename);
  if( !file.open(QFile::ReadOnly) )
  {
    return false;
  }
  QTextStream text_stream(&file);

  // The first line should contain the name of the columns
  QString first_line = text_stream.readLine();
  QStringList column_names = first_line.split(csv_separator);

  // create a vector of timeseries
  std::vector<PlotData*> plots_vector;

  for (unsigned i = 0; i < column_names.size(); i++)
  {
    QString column_name = column_names[i].simplified();

    std::string field_name = column_names[i].toStdString();

    auto it = plot_data.addNumeric(field_name);

    plots_vector.push_back(&(it->second));
  }

  //-----------------
  // read the file line by line
  int linecount = 1;
  while (!text_stream.atEnd())
  {
    QString line = text_stream.readLine();
    linecount++;

    // Split using the comma separator.
    QStringList string_items = line.split(csv_separator);
    if (string_items.size() != column_names.size())
    {
      auto err_msg = QString("The number of values at line %1 is %2,\n"
                             "but the expected number of columns is %3.\n"
                             "Aborting...")
          .arg(linecount)
          .arg(string_items.size())
          .arg(column_names.size());

      QMessageBox::warning(nullptr, "Error reading file", err_msg );
      return false;
    }

    // The first column should contain the timestamp.
    QString first_item = string_items[0];
    bool is_number;
    double t = first_item.toDouble(&is_number);

    // check if the time format is a DateTime
    if (!is_number )
    {
      QDateTime ts = QDateTime::fromString(string_items[0], "yyyy-MM-dd hh:mm:ss");
      if (!ts.isValid())
      {
          QMessageBox::warning(nullptr, tr("Error reading file"),
                  tr("Couldn't parse timestamp. Aborting.\n"));

          return false;
      }
      t = ts.toMSecsSinceEpoch()/1000.0;
    }

    for (int i = 0; i < string_items.size(); i++)
    {
      double y = string_items[i].toDouble(&is_number);
      if (is_number)
      {
        PlotData::Point point(t, y);
        plots_vector[i]->pushBack(point);
      }
    }
  }

  file.close();

  return true;
}






