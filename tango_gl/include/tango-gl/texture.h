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

#ifndef TANGO_GL_TEXTURE_H_
#define TANGO_GL_TEXTURE_H_

#include <android/asset_manager.h>
#include <errno.h>
#include <png.h>

#include "tango-gl/util.h"

#ifndef GL_OES_compressed_ETC1_RGB8_texture
#define GL_OES_compressed_ETC1_RGB8_texture 1
#define GL_ETC1_RGB8_OES                  0x8D64
#endif /* GL_OES_compressed_ETC1_RGB8_texture */

namespace tango_gl {

class Texture {
 public:
  explicit Texture(AAssetManager* mgr, const char* file_path);
  explicit Texture(const char* file_path);
  Texture(GLenum texture_id, GLenum texture_target);
  Texture(const Texture& other) = delete;
  Texture& operator=(const Texture&) = delete;

    ~Texture()
    {
        FreeResources();
    }

  bool LoadFromPNG(FILE* file);
  GLuint GetTextureID() const;
  GLenum GetTextureTarget() const;

 protected:
  png_uint_32 width_, height_;
  int bit_depth_, color_type_;
  GLuint texture_id_;
  GLenum texture_target_;

    void FreeResources()
    {
        if (texture_id_ > 0)
        {
            glDeleteTextures(1, &texture_id_);
            texture_id_ = 0;
        }
    }
};

    // at the moment predefined size (256 x 128)
    class DynamicTexture : public Texture {
    public:
        // a constructor
        DynamicTexture()
                : Texture(nullptr)
        {
            // GL_COMPRESSED_RGB8_ETC2
            compressInternalFormat = GL_ETC1_RGB8_OES; // GL_COMPRESSED_ETC1_RGB8_OES;
            width_ = 256;
            height_ = 128;
            texture_id_ = 0;
            texture_target_ = GL_TEXTURE_2D;
            mLastSize = 0;

            curPBO = 0;
            pbo[0] = pbo[1] = 0;

            Allocate(width_, height_);
        }

        virtual ~DynamicTexture()
        {
            if (pbo[0] > 0)
                glDeleteBuffers(2, pbo);
        }

        // update by passing compressed ETC1 data
        void Update(int internalFormat, int w, int h, const unsigned char *data, const unsigned int size);

        void UpdateSimple(int internalFormat, int w, int h, const unsigned char *data, const unsigned int size);

    protected:

        GLuint      mLastSize;
        GLenum compressInternalFormat;

        GLuint  pbo[2];
        unsigned int curPBO;

        void Allocate(int w, int h);
        void SwapPBO();
    };


}  // namespace tango_gl
#endif  // TANGO_GL_TEXTURE_H_
