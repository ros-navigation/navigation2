/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
#ifndef TEST_CONSTANTS__TEST_CONSTANTS_H_
#define TEST_CONSTANTS__TEST_CONSTANTS_H_

/* Author: Brian Gerkey */

/* This file externs global constants shared among tests */

#include <vector>

extern const unsigned int g_valid_image_width;
extern const unsigned int g_valid_image_height;
extern const char g_valid_image_content[];
extern const char * g_valid_map_name;
extern const char * g_valid_png_file;
extern const char * g_valid_bmp_file;
extern const char * g_valid_pgm_file;
extern const char * g_valid_yaml_file;
extern const char * g_tmp_dir;

extern const double g_valid_image_res;
// *INDENT-OFF*
// Uncrustify may incorrectly guide to add extra spaces in < double > during CI tests
extern const std::vector<double> g_valid_origin;
// *INDENT-ON*
extern const double g_default_free_thresh;
extern const double g_default_occupied_thresh;

#endif  // TEST_CONSTANTS__TEST_CONSTANTS_H_
