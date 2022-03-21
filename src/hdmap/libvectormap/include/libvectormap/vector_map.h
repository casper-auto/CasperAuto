/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef LIBVECTORMAP_VECTOR_MAP_H
#define LIBVECTORMAP_VECTOR_MAP_H

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <map>
#include <string>

#include "Math.h"

#include <vector_map/vector_map.h>

typedef struct
{
  int pid;
  double bx;
  double ly;
  double h;
  double b;
  double l;
  int ref;
  int mcode1;
  int mcode2;
  int mcode3;
}
Point;

typedef struct
{
  int lid;
  int bpid;
  int fpid;
  int blid;
  int flid;
}
Line;

typedef struct
{
  int vid;
  int pid;
  double hang;
  double vang;
}
Vector;

typedef struct
{
  int id;
  int vid;
  int plid;
  int type;
  int linkid;
}
Signal;

typedef struct
{
  int id;
  int lid;
  double width;
  char color;
  int type;
  int linkid;
}
WhiteLine;

typedef struct
{
  int lid;
}
Mark;

typedef struct
{
  int did;
  double dist;
  int pid;
  double dir;
  double apara;
  double r;
  double slope;
  double cant;
  double lw;
  double rw;
}
DTLane;

typedef struct
{
  int lnid;
  int did;
  int blid;
  int flid;
  int bnid;
  int fnid;
  int jct;
  int blid2;
  int blid3;
  int blid4;
  int flid2;
  int flid3;
  int flid4;
  int clossid;
  double span;
  int lcnt;
  int lno;
}
Lane;

class VectorMap
{
public:
  bool loaded;
  std::map<int, Point> points;
  std::map<int, Line> lines;
  std::map<int, WhiteLine> whitelines;
  std::map<int, Lane> lanes;
  std::map<int, DTLane> dtlanes;
  std::map<int, Vector> vectors;
  std::map<int, Signal> signals;

  void load_points(const vector_map::PointArray& msg);
  void load_lines(const vector_map::LineArray& msg);
  void load_lanes(const vector_map::LaneArray& msg);
  void load_vectors(const vector_map::VectorArray& msg);
  void load_signals(const vector_map::SignalArray& msg);
  void load_whitelines(const vector_map::WhiteLineArray& msg);
  void load_dtlanes(const vector_map::DTLaneArray& msg);

  VectorMap() :
    loaded(false) {}

  inline Point3 getPoint(const int idx)
  {
    Point3 p;
    Point psrc = points[idx];
    p.x() = psrc.bx;
    p.y() = psrc.ly;
    p.z() = psrc.h;
    return p;
  }
};

#endif  // LIBVECTORMAP_VECTOR_MAP_H
