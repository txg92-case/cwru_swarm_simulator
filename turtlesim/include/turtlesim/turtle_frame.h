/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <QFrame>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QVector>

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <ros/ros.h>

# include <std_srvs/Empty.h>
# include <swarm_sim/Spawn.h>
# include <swarm_sim/Kill.h>
# include <swarm_sim/Betray.h>
# include <swarm_sim/SwarmList.h>
# include <map>

# include "turtle.h"
#endif

namespace swarm_sim
{

class TurtleFrame : public QFrame
{
  Q_OBJECT
public:
  TurtleFrame(QWidget* parent = 0, Qt::WindowFlags f = 0);
  ~TurtleFrame();

  std::string spawnTurtle(const std::string& name, float x, float y, float angle);
  std::string spawnTurtle(const std::string& name, float x, float y, float angle, size_t index);

protected:
  void paintEvent(QPaintEvent* event);

private slots:
  void onUpdate();

private:
  void updateTurtles();
  void clear();
  bool hasTurtle(const std::string& name);

  bool clearCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
  bool resetCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
  bool spawnCallback(swarm_sim::Spawn::Request&, swarm_sim::Spawn::Response&);
  bool killCallback(swarm_sim::Kill::Request&, swarm_sim::Kill::Response&);
  bool betrayCallback(swarm_sim::Betray::Request&, swarm_sim::Betray::Response&);
  bool listCallback(swarm_sim::SwarmList::Request&, swarm_sim::SwarmList::Response&);

  ros::NodeHandle nh_;
  QTimer* update_timer_;
  QImage path_image_;
  QPainter path_painter_;

  uint64_t frame_count_;

  ros::WallTime last_turtle_update_;

  ros::ServiceServer clear_srv_;
  ros::ServiceServer reset_srv_;
  ros::ServiceServer spawn_srv_;
  ros::ServiceServer kill_srv_;
  ros::ServiceServer betray_srv_;

  ros::ServiceServer list_srv_;

  typedef std::map<std::string, TurtlePtr> M_Turtle;
  M_Turtle turtles_;
  uint32_t id_counter_;

  QVector<QImage> turtle_images_;

  float meter_;
  float width_in_meters_;
  float height_in_meters_;
};

}
