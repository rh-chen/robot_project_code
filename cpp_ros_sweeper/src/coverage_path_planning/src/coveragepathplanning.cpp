//
//  coveragepathplanning.cpp
//  CPP
//
//  Created by 欧阳伟斌 on 16/10/17.
//  Copyright © 2016年 欧阳伟斌. All rights reserved.
//

#include "coveragepathplanning.hpp"
#include <queue>
#include <stack>

using namespace cv;   //  NOLINT
using namespace std;  //  NOLINT
using namespace cpp;  //  NOLINT

static const int CLOSED = -1;
static const unsigned char BLOCKED = 0;

static const Point g_neighbours[8] = {Point(1, 0),  Point(1, 1),  Point(0, 1),
                                      Point(-1, 1), Point(-1, 0), Point(-1, -1),
                                      Point(0, -1), Point(1, -1)};

static int WaveFront(
    const Mat &_binary, Mat &_dt, const Point &_tar) {  //  NOLINT
  // init
  _dt =
      Mat(_binary.rows, _binary.cols, CV_32S,
          CLOSED);  // prepare matrix with same size to binary image
  Mat binary = _binary.clone();  // do not touch original source
  queue<Point> closed;
  Point cur;
  const Point *neighbours = g_neighbours;
  int dist;
  int col, row;

  // set initial distance
  _dt.at<int>(_tar.y, _tar.x) = 0;
  binary.at<unsigned char>(_tar.y, _tar.x) = 0;
  closed.push(_tar);

  // update neighbour distance
  while (!closed.empty()) {
    // get current point and its distance
    cur = closed.front();
    dist = _dt.at<int>(cur.y, cur.x);

// traverse current point's neighbour
#ifdef WAVEFRONT_CONNECT_8
    for (int i = 0; i < 8; ++i) {
#else
    for (int i = 0; i < 8; i += 2) {
#endif
      col = cur.x + neighbours[i].x;
      row = cur.y + neighbours[i].y;

      // check if valid and unvisited
      if (0 <= row && row < binary.rows && 0 <= col && col < binary.cols &&
          BLOCKED != binary.at<unsigned char>(row, col)) {
        _dt.at<int>(row, col) = dist + 1;              // update distance
        binary.at<unsigned char>(row, col) = BLOCKED;  // set visited
        closed.push(Point(col, row));
      }
    }

    closed.pop();
  }

  return 0;
}

static int IsValid(const Mat &_dt, int row, int col, int _radius) {
  // init
  int dist = 2 * _radius;
  int anchorX = col - _radius;
  int anchorY = row + _radius;
  int x, y;

  // traverse neighbours
  for (int i = 0; i <= dist; ++i) {
    x = anchorX + i;
    for (int j = 0; j >= -dist; --j) {
      y = anchorY + j;
      if (y < 0 || x < 0 || y >= _dt.rows || x >= _dt.cols ||
          CLOSED == _dt.at<int>(y, x)) {
        return 0;
      }
    }
  }

  return 1;
}

static int FindHighestNeighbour(
    const Mat &_dt,                                                  //  NOLINT
    const Point &_center, Point &_tar, int _radius, int &_direct) {  //  NOLINT
  // init
  int i, j, col, row, tarY = -1, tarX = -1, maxDist = -99, maxDistIndex = -1;
  const Point *neighbours = g_neighbours;
  static int start = 0;

// traverse center's neighbour and find the highest one
#ifdef PLANNING_CONNECT_8
  for (j = 0; j < 8; ++j) {
    i = (start + j) % 8;
#else
  for (j = 0; j < 4; ++j) {
    i = (start + 2 * j) % 8;
#endif

    // calculate candidate's coordinate
    col = _center.x + neighbours[i].x * (2 * _radius + 1);
    row = _center.y + neighbours[i].y * (2 * _radius + 1);

    if (col < 0 || row < 0 || col >= _dt.cols ||
        row >= _dt.rows) {  // ignore invalid point
      continue;
    }

    if (_dt.at<int>(row, col) > maxDist &&
        IsValid(_dt, row, col, _radius)) {  // found new max choice
      // update max distance information
      maxDist = _dt.at<int>(row, col);
      maxDistIndex = i;
      tarX = col;
      tarY = row;

#ifndef PLANNING_SOLID_BEGINNING
#ifndef PLANNING_ONE_ACCUMULATE_BEGINNING
      start = (i + 4) % 8;
// start = (i + 2) % 8;
#endif
#endif
    }
  }

  // update supposed direction
  _direct = maxDistIndex;

  if (-1 == maxDistIndex) {  // not found
    return -1;
  }

  // update target point
  _tar.x = tarX;
  _tar.y = tarY;

#ifndef PLANNING_SOLID_BEGINNING
#ifdef PLANNING_ONE_ACCUMULATE_BEGINNING
  start = (maxDistIndex + 4) % 8;
// start = (maxDistIndex + 2) % 8;
#endif
#endif

  return 0;
}

static int Visit(
    Mat &_dt, stack<Point> &_track,          //  NOLINT
    deque<Point> &_path, const Point &_tar,  //  NOLINT
    int _radius, bool isSameDirect) {        //  NOLINT
  // init
  int dist = 2 * _radius;
  int anchorX = _tar.x - _radius;
  int anchorY = _tar.y + _radius;
  int x, y;

  // close points in radius
  for (int i = 0; i <= dist; ++i) {
    for (int j = 0; j >= -dist; --j) {
      x = anchorX + i;
      y = anchorY + j;
      if (y < 0 || x < 0 || y >= _dt.rows ||
          x >= _dt.cols) {  // ignore invalid point
        continue;
      }
      _dt.at<int>(y, x) = CLOSED;
    }
  }

  // update track
  _track.push(_tar);

  // update path
  if (isSameDirect) {
    _path.pop_back();
  }
  _path.push_back(_tar);

  return 0;
}

static int Planning(
    const Mat &_dt, const Point &_start, const Point &_goal,
    deque<Point> &_path, int _radius) {  //  NOLINT
  // init
  stack<Point> track;
  Mat dt = _dt.clone();  // do not touch original source
  int rst;
  Point tar;
  bool isSameDirect = false;
  int direct = -1, curDirect = -1, lastDirect = -1;

  // close goal
  dt.at<int>(_goal.y, _goal.x) = CLOSED;

  // visit start point
  rst = Visit(dt, track, _path, _start, _radius, 0);

  // path planning
  while (!track.empty()) {
    rst = FindHighestNeighbour(dt, track.top(), tar, _radius, direct);

    // update direction information
    isSameDirect = (curDirect == direct && -1 != lastDirect);
    lastDirect = curDirect;
    curDirect = direct;

    if (0 == rst) {  // found
      Visit(dt, track, _path, tar, _radius, isSameDirect);
    } else {  // not found
      track.pop();
    }
  }

  // add goal to path
  _path.push_back(_goal);

  return 0;
}

int cpp::CoveragePathPlanning(
    const Mat &_binary, const Point &_start, const Point &_goal,
    deque<Point> &_path, int _radius) {  //  NOLINT
  Mat binary = _binary.clone();
  int rst = -1;

  Mat dt;  // distance transform
  if (0 != (rst = WaveFront(binary, dt, _goal))) {
    return -1;
  }

  if (0 != (rst = Planning(dt, _start, _goal, _path, _radius))) {
    return -1;
  }

  return 0;
}

/* ---------------------------------------------------------- */

struct Candidate
{
    Point m_pos;
    bool m_checkUp;
    bool m_checkDown;
};

static int IsValid(const Mat &_dt, Point _tar, int _radius) {
    //init
    int dist = 2 * _radius + 1;
    int anchorX = _tar.x - _radius;
    int anchorY = _tar.y + _radius;
    int x, y;
    
    //traverse neighbours
    for (int i = 0; i <= dist; ++i) {
        x = anchorX + i;
        for (int j = 0; j >= -dist; --j) {
            y = anchorY + j;
            if (y < 0 || x < 0 || y >= _dt.rows || x >= _dt.cols || 
              0 == _dt.at<unsigned char>(y, x)) {
                return 0;
            }
        }
    }
    
    return 1;
}

int UpdatePath(cv::Mat &_binary, const cv::Point &_tar, 
  std::deque<cv::Point> &_path, stack<Candidate> &_trace, int _radius,std::vector<cv::Point>& _sweeped_region) {
    // search left
    Point left = _tar;
    int left_dist = -1;
    while (IsValid(_binary, left, _radius)) {
        --left.x;
        ++left_dist;
    }
    ++left.x;

    // search right
    Point right = _tar;
    int right_dist = -1;
    while (IsValid(_binary, right, _radius)) {
        ++right.x;
        ++right_dist;
    }
    --right.x;

    // update
    Candidate tmp;
    if (left_dist < right_dist) { // from left to right
        _path.push_back(left);
        _path.push_back(right);
        for (int i = left.x; i <= right.x; ++i) {
            _binary.at<unsigned char>(_tar.y, i) = 0;
						_sweeped_region.push_back(cv::Point(_tar.y,i));
            tmp.m_pos.x = i;
            tmp.m_pos.y = _tar.y;
            tmp.m_checkUp = false;
            tmp.m_checkDown = false;
            _trace.push(tmp);
        }
    } else { // from right to left
        _path.push_back(right);
        _path.push_back(left);
        for (int i = right.x; i >= left.x; --i) {
            _binary.at<unsigned char>(_tar.y, i) = 0;
						_sweeped_region.push_back(cv::Point(_tar.y,i));
            tmp.m_pos.x = i;
            tmp.m_pos.y = _tar.y;
            tmp.m_checkUp = false;
            tmp.m_checkDown = false;
            _trace.push(tmp);
        }
    }

    return 0;
}

int cpp::ZigZagPathPlanning(const cv::Mat &_binary, 
  const cv::Point &_tar, std::deque<cv::Point> &_path, int _radius,std::vector<cv::Point>& _sweeped_region) {
    Mat binary = _binary.clone();
    stack<Candidate> trace;
		
		for(int i = 0;i < _sweeped_region.size();i++)
		{
			binary.at<unsigned char>(_sweeped_region.at(i).y,_sweeped_region.at(i).x) = 0;
		}

    if (IsValid(binary, _tar, _radius) != 1) {
        return -1;
    }

    if (0 != UpdatePath(binary, _tar, _path, trace, _radius,_sweeped_region)) {
        return -1;
    }

    while (trace.empty() != true) {
        //  search up
        if (trace.top().m_checkUp != true) {
            trace.top().m_checkUp = true;
            Point tmp(trace.top().m_pos.x, trace.top().m_pos.y - 2 * _radius - 1);
            if (IsValid(binary, tmp, _radius) == 1) {
                UpdatePath(binary, tmp, _path, trace, _radius,_sweeped_region);
                continue;
            }
        }

        //  search down
        if (trace.top().m_checkDown != true) {
            trace.top().m_checkDown = true;
            Point tmp(trace.top().m_pos.x, trace.top().m_pos.y + 2 * _radius + 1);
            if (IsValid(binary, tmp, _radius)) {
                UpdatePath(binary, tmp, _path, trace, _radius,_sweeped_region);
                continue;
            }
        }

        trace.pop();
    }

    return 0;
}
