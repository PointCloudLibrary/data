/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 * @author: Cedric Cagniart, Koen Buys
 */

#ifndef PCL_PEOPLE_DATASYNTH_RGBDRENDERER_H_
#define PCL_PEOPLE_DATASYNTH_RGBDRENDERER_H_

#include "OpenGLContext.h"
#include <stdint.h>
#include <GL/gl.h>

namespace pcl
{
namespace people
{
namespace datasynth
{
class RGBDRenderer
{
public :

  struct LabeledVertex
  {
    GLfloat x, y, z;
    GLuint  label;
  };

  static const int mW = 640;
  static const int mH = 480;

  static const GLint DMAP_FORMAT         = GL_LUMINANCE16UI_EXT;
  static const GLint DMAP_COLORFORMAT    = GL_LUMINANCE_INTEGER_EXT;
  static const GLint DMAP_COLORTYPE      = GL_UNSIGNED_SHORT;

  static const GLint LMAP_FORMAT         = GL_LUMINANCE8UI_EXT;
  static const GLint LMAP_COLORFORMAT    = GL_LUMINANCE_INTEGER_EXT;
  static const GLint LMAP_COLORTYPE      = GL_UNSIGNED_BYTE;

  RGBDRenderer();
  ~RGBDRenderer();

  void draw(const float         *GLPMat,
            const float         *GLMMat,
            const uint16_t       BackgroundDepth,
            const int            numVertices,
            const int            numTriangles,
            const LabeledVertex *vBuffer,
            const GLuint        *iBuffer,
            uint16_t            *depth,
            uint8_t             *labels);

protected :
  // the opengl context
  OpenGLContext mGLContext;

  // the textures and fbo
  GLuint mGLFBOId;
  GLuint mDMapTex;
  GLuint mLMapTex;
  GLuint mGLDepthTex;

  // the shader
  GLint mGLId_shad_vshader;
  GLint mGLId_shad_fshader;
  GLint mGLID_prog;

  // the matrices uniforms
  GLint mGLID_uniform_proj;
  GLint mGLID_uniform_mview;

  // the attribs uniforms
  GLint mGLID_attrib_coord;
  GLint mGLID_attrib_label;
};
} // End namespace datasynth
} // End namespace people
} // End namespace pcl

#endif
