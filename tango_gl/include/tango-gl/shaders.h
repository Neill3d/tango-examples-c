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

#ifndef TANGO_GL_SHADERS_H_
#define TANGO_GL_SHADERS_H_

#include <string>

namespace tango_gl {
namespace shaders {
std::string GetBasicVertexShader();
std::string GetBasicFragmentShader();
std::string GetTexturedVertexShader();
std::string GetTexturedFragmentShader();
    std::string GetScreenVertexShader();
    std::string GetScreenFragmentShader();
    std::string GetColorVertexShader();
std::string GetVideoOverlayVertexShader();
std::string GetVideoOverlayFragmentShader();
std::string GetVideoOverlayTexture2DFragmentShader();
std::string GetShadedVertexShader();
}  // namespace shaders
}  // namespace tango_gl
#endif  // TANGO_GL_SHADERS_H_
