/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#ifndef COSTMAP_2D_COSTMAP_MATH_H_
#define COSTMAP_2D_COSTMAP_MATH_H_

#include <math.h>
#include <algorithm>
#include <vector>
#include <geometry_msgs/Point.h>

/** @brief Return -1 if x < 0, +1 otherwise. */
inline double sign(double x)
{
  return x < 0.0 ? -1.0 : 1.0;
}

/** @brief Same as sign(x) but returns 0 if x is 0. */
inline double sign0(double x)
{
  return x < 0.0 ? -1.0 : (x > 0.0 ? 1.0 : 0.0);
}

inline double distance(double x0, double y0, double x1, double y1)
{
  return hypot(x1 - x0, y1 - y0);
}
/**
 * @brief distanceToLine    点p到直线01的距离
 * @param pX                点p的X坐标
 * @param pY                点p的的Y坐标
 * @param x0                线上点0的X坐标
 * @param y0                线上点0的Y坐标
 * @param x1                线上点1的X坐标
 * @param y1                线上点1的Y坐标
 * @return                  点p到直线01的距离
 */
double distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1);
/**
 * @brief intersects  通过射线法判断test点是否在多边形内部
 * @param polygon     指定的多边形
 * @param testx       test点x坐标
 * @param testy       test点y坐标
 * @return            如果点在多边形内部,则返回true,否则返回false
 */
bool intersects(std::vector<geometry_msgs::Point>& polygon, float testx, float testy);
/**
 * @brief intersects  判断两个多边形是否相交或者包含
 * @param polygon1    多边形1
 * @param polygon2    多边形2
 * @return            如果相交或者包含,返回true否则返回false
 */
bool intersects(std::vector<geometry_msgs::Point>& polygon1, std::vector<geometry_msgs::Point>& polygon2);

#endif  // COSTMAP_2D_COSTMAP_MATH_H_
