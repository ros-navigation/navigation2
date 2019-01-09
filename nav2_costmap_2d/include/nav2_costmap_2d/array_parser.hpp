/*
 * Copyright (c) 2012, Willow Garage, Inc.
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
 *
 * author: Dave Hershberger
 */
#ifndef NAV2_COSTMAP_2D__ARRAY_PARSER_HPP_
#define NAV2_COSTMAP_2D__ARRAY_PARSER_HPP_

#include <vector>
#include <string>

namespace nav2_costmap_2d
{

/** @brief Parse a vector of vectors of floats from a string.
 * @param error_return If no error, error_return is set to "".  If
 *        error, error_return will describe the error.
 * Syntax is [[1.0, 2.0], [3.3, 4.4, 5.5], ...]
 *
 * On error, error_return is set and the return value could be
 * anything, like part of a successful parse. */
std::vector<std::vector<float>> parseVVF(const std::string & input, std::string & error_return);

}  // end namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__ARRAY_PARSER_HPP_
