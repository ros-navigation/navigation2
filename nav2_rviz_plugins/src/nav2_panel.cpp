// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_rviz_plugins/nav2_panel.hpp"

#include <QHBoxLayout>
#include <QPushButton>
#include <QVBoxLayout>
#include <QtConcurrent/QtConcurrent>

#include <QtCharts/QBarSeries>
#include <QtCharts/QBarSet>
#include <QtCharts/QChartView>
#include <QtCharts/QLegend>
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QValueAxis>

#include "rviz_common/display_context.hpp"

using QtCharts::QBarSeries;
using QtCharts::QBarSet;
using QtCharts::QChart;
using QtCharts::QChartView;
using QtCharts::QLegend;
using QtCharts::QLineSeries;
using QtCharts::QScatterSeries;
using QtCharts::QValueAxis;

namespace nav2_rviz_plugins
{

Nav2Panel::Nav2Panel(QWidget * parent)
: Panel(parent)
{
  startup_button = new QPushButton("Startup");
  shutdown_button = new QPushButton("Shutdown");

  connect(startup_button, SIGNAL(clicked()), this, SLOT(onStartupClicked()));
  connect(shutdown_button, SIGNAL(clicked()), this, SLOT(onShutdownClicked()));

  startup_button->setToolTip("TODO");
  shutdown_button->setToolTip("TODO");

  QChart * chart = new QChart();
  QChartView * chartView = new QChartView(chart);
  chartView->setMinimumSize(400, 300);

  QValueAxis * axisX = new QValueAxis;
  axisX->setRange(0, 2000);
  axisX->setLabelFormat("%g");
  axisX->setTitleText("Loop Iteration");

  QValueAxis * axisY = new QValueAxis;
  axisY->setRange(0, 50);
  axisY->setTitleText("Time (ms)");

  QLineSeries * series = new QLineSeries;
  series->setName("loop rate samples");
  QPen pen1(QRgb(0xfdb157));
  pen1.setWidth(2);
  series->setPen(pen1);
  QVector<QPointF> points;
  for (int i = 0; i < 2000; i++) {
    points.append(QPointF(i, 25));
  }
  series->replace(points);

  QLineSeries * lineseries = new QLineSeries;
  lineseries->setName("target");
  QPen pen(QRgb(0xff0000));
  pen.setWidth(2);
  lineseries->setPen(pen);
  QVector<QPointF> points2;
  points2.append(QPointF(0, 10));
  points2.append(QPointF(2000, 10));
  lineseries->replace(points2);

  QPainterPath starPath;
  starPath.moveTo(28, 15);
  for (int i = 1; i < 5; i++) {
    starPath.lineTo(14 + 14 * qCos(0.8 * i * M_PI),
      15 + 14 * qSin(0.8 * i * M_PI));
  }
  starPath.closeSubpath();

  QImage star(30, 30, QImage::Format_ARGB32);
  star.fill(Qt::transparent);

  QPainter painter(&star);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(QRgb(0x0000ff));
  painter.setBrush(painter.pen().color());
  painter.drawPath(starPath);

#if 0
  QAbstractBarSeries QHorizontalBarSeries QScatterSeries
  QAbstractSeries QHorizontalPercentBarSeries QSplineSeries
  QAreaSeries QHorizontalStackedBarSeries QStackedBarSeries
  QBarSeries QLineSeries QXYSeries
  QBoxPlotSeries QPercentBarSeries
  QCandlestickSeries QPieSeries
#endif

#if 0
  QBarSet * set0 = new QBarSet("samples");
  for (int i = 0; i < 1000; i++) {
    *set0 << i;
  }
  QBarSeries * barseries = new QBarSeries();
  barseries->append(set0);
#endif

  QScatterSeries * series2 = new QScatterSeries();
  series2->setName("scatter3");
  series2->setMarkerShape(QScatterSeries::MarkerShapeRectangle);
  series2->setMarkerSize(30.0);

  *series2 << QPointF(1, 5) << QPointF(500, 25) << QPointF(1500, 50);
  series2->setBrush(star);
  series2->setPen(QColor(Qt::transparent));

  chart->setAxisX(axisX, series);
  chart->setAxisY(axisY, series);
  chart->addSeries(series);
  chart->addSeries(lineseries);
  chart->addSeries(series2);
  // chart->addSeries(barseries);
  chart->legend()->hide();
  chart->setTitle("DWB Loop Rate");

  series->attachAxis(axisX);
  series->attachAxis(axisY);
  lineseries->attachAxis(axisX);
  lineseries->attachAxis(axisY);
  series2->attachAxis(axisX);
  series2->attachAxis(axisY);
  // barseries->attachAxis(axisX);
  // barseries->attachAxis(axisY);

  QHBoxLayout * button_layout = new QHBoxLayout;
  button_layout->addWidget(startup_button);
  button_layout->addWidget(shutdown_button);
  button_layout->setContentsMargins(2, 0, 2, 2);

  QVBoxLayout * main_layout = new QVBoxLayout;
  main_layout->setContentsMargins(0, 0, 0, 0);
  main_layout->addWidget(chartView);
  main_layout->addLayout(button_layout);
  setLayout(main_layout);
}

void
Nav2Panel::onInitialize()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
}

void
Nav2Panel::onStartupClicked()
{
  QFuture<void> future =
    QtConcurrent::run(std::bind(&nav2_controller::Nav2ControllerClient::startup, &client_));
  startup_button->setEnabled(false);
}

void
Nav2Panel::onShutdownClicked()
{
  QFuture<void> future =
    QtConcurrent::run(std::bind(&nav2_controller::Nav2ControllerClient::shutdown, &client_));
  shutdown_button->setEnabled(false);
}

void
Nav2Panel::save(rviz_common::Config config) const
{
  Panel::save(config);
}

void
Nav2Panel::load(const rviz_common::Config & config)
{
  Panel::load(config);
}

}  // namespace nav2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::Nav2Panel, rviz_common::Panel)
