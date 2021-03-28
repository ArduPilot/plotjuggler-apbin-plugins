#include "datastream_sample.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QDebug>
#include <thread>
#include <mutex>
#include <chrono>
#include <thread>
#include <math.h>

using namespace PJ;

DataStreamSample::DataStreamSample()
{
    // Create 10 numeric series
    for (int i = 0; i < 10; i++)
    {
        auto str = QString("data_vect/%1").arg(i).toStdString();
        DataStreamSample::Parameters param;
        param.A = 6 * ((double)rand() / (double)RAND_MAX) - 3;
        param.B = 3 * ((double)rand() / (double)RAND_MAX);
        param.C = 3 * ((double)rand() / (double)RAND_MAX);
        param.D = 20 * ((double)rand() / (double)RAND_MAX);
        _parameters.insert({str, param});
        auto& plotdata = dataMap().addNumeric(str)->second;
    }
    //------------
    // This is a String series. Can not be plotted, but you can see the value
    // in the tree view at elast.
    dataMap().addStringSeries("color");

    //------------
    // Demonstrate how to use Groups and properties
    auto& tc_default = dataMap().addNumeric("tc/blue")->second;
    auto& tc_red = dataMap().addNumeric("tc/red")->second;

    // create a PlotGroup
    auto tcGroup = std::make_shared<PJ::PlotGroup>("tc");
    tc_default.setGroup( tcGroup );
    tc_red.setGroup( tcGroup );

    tcGroup->setAttribute("text_color", QColor(Qt::blue) );
    // Series "text_color" property has a priority over the group color
    tc_red.setAttribute("text_color", QColor(Qt::red) );

}

bool DataStreamSample::start(QStringList*)
{
    _running = true;

    pushSingleCycle();

    // Create a thread that generate random data.
    // In a real world plugin, this data would come from an external
    // publisher.
    _thread = std::thread([this]() { this->loop(); });
    return true;
}

void DataStreamSample::shutdown()
{
    _running = false;
    if (_thread.joinable()){
        _thread.join();
    }
}

DataStreamSample::~DataStreamSample()
{
    shutdown();
}

void DataStreamSample::pushSingleCycle()
{
    static int count = 0;
    count++;
    const std::string colors[]= { "RED", "BLUE", "GREEN" };

    using namespace std::chrono;
    auto now = high_resolution_clock::now().time_since_epoch();
    double stamp = duration_cast<duration<double>>(now).count();

    auto& col_series = dataMap().strings.find("color")->second;
    auto& tc_default = dataMap().numeric.find("tc/default")->second;
    auto& tc_red = dataMap().numeric.find("tc/red")->second;

    // Important: you need to apply the lock every time you modify dataMap()
    std::lock_guard<std::mutex> lock(mutex());

    col_series.pushBack( { stamp, colors[ (count/10) % 3]});
    tc_default.pushBack( { stamp, double(count) });
    tc_red.pushBack( { stamp, double(count) });

    for (auto& it : _parameters)
    {
        auto& plot = dataMap().numeric.find(it.first)->second;
        const DataStreamSample::Parameters& p = it.second;

        double val = p.A*sin( p.B*stamp + p.C ) + p.D;
        plot.pushBack( {stamp, val} );
    }
}

void DataStreamSample::loop()
{
    _running = true;
    while (_running)
    {
        auto prev = std::chrono::high_resolution_clock::now();
        pushSingleCycle();
        emit dataReceived();
        std::this_thread::sleep_until(prev + std::chrono::milliseconds(20));  // 50 Hz
    }
}
