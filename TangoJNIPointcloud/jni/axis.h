/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef AXIS_H
#define AXIS_H

#include "drawable_object.h"
#include "gl_util.h"

class Axis : public DrawableObject {
 public:
  Axis();
  void Render(glm::mat4 view_projection_mat);
 private:
  GLuint vertex_buffer;
  GLuint color_buffer;

  GLuint shader_program;
  GLuint attrib_vertices;
  GLuint attrib_colors;
  GLuint uniform_mvp_mat;
};

#endif  // AXIS_H