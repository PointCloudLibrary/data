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

#include <RGBDRenderer.h>
#include <iostream>
#include <limits>
#include <algorithm>

#include "OpenGLContext.h"
#include "GLerrors.h"

#include <GL/glext.h>

#include <cassert>

static const char *code_vertex   = "\
	#version 140                                                      \n\
	uniform mat4 projection_matrix, modelview_matrix;                 \n\
	in vec3          in_coord;                                        \n\
	in uint          in_label;                                        \n\
	smooth out float lin_z;                                           \n\
	flat   out uint  label;                                           \n\
	invariant gl_Position;                                            \n\
	void main()                                                       \n\
	{                                                                 \n\
		vec4 xcam   = modelview_matrix * vec4(in_coord ,1.0);         \n\
		lin_z       = xcam.z;                                         \n\
		label       = in_label;                                       \n\
		gl_Position = projection_matrix * xcam;                       \n\
	}                                                                 \n\
";

static const char *code_fragment = "\
	#version 140                                                      \n\
	smooth in float lin_z;                                            \n\
	flat   in uint  label;                                            \n\
	out unsigned char out_label;                                      \n\
	out uint out_depth;                                               \n\
	void main()                                                       \n\
	{                                                                 \n\
		out_depth = uint(lin_z*1000.);                                \n\
		out_label = unsigned char(label);                             \n\
	}                                                                 \n\
";

pcl::people::datasynth::RGBDRenderer::RGBDRenderer():
  mGLContext(":0.0")
{
  // ---------------------
  // create textures
  glGenTextures(1, &mLMapTex);
  glGenTextures(1, &mDMapTex);
  glGenTextures(1, &mGLDepthTex);


#define SETTEXPARAMS \
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);\
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);\
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);\
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

  // ---------------------
  // alloc textures
  glActiveTexture(GL_TEXTURE0);
  glEnable(GL_TEXTURE_2D);
  // class
  glBindTexture(GL_TEXTURE_2D, mLMapTex);
  glTexImage2D(GL_TEXTURE_2D, 0, LMAP_FORMAT, mW, mH, 0,
               LMAP_COLORFORMAT, LMAP_COLORTYPE, 0);
  SETTEXPARAMS
  // depth
  glBindTexture(GL_TEXTURE_2D, mDMapTex);
  glTexImage2D(GL_TEXTURE_2D, 0, DMAP_FORMAT, mW, mH, 0,
               DMAP_COLORFORMAT, DMAP_COLORTYPE, 0);
  SETTEXPARAMS
  // gldmap
  glBindTexture(GL_TEXTURE_2D, mGLDepthTex);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, mW, mH, 0,
               GL_DEPTH_COMPONENT, GL_FLOAT, 0);
  SETTEXPARAMS
  // - clean up
  glBindTexture(GL_TEXTURE_2D, 0);
  glDisable(GL_TEXTURE_2D);
#undef SETTEXPARAMS

  // NECESSARY. (not for 640x480 but still) or else gl will align it on 4
  //and expect bigger buffers when readpixels is called
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  checkGLErrors("textures allocated");

  // ---------------------
  // create FBO
  glGenFramebuffers(1, &mGLFBOId);
  glBindFramebuffer(GL_FRAMEBUFFER, mGLFBOId);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                         GL_TEXTURE_2D, mLMapTex, 0);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1,
                         GL_TEXTURE_2D, mDMapTex, 0);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                         GL_TEXTURE_2D, mGLDepthTex, 0);
  checkGLFrameBufferStatus();
  glBindFramebuffer(GL_FRAMEBUFFER, 0);


  // --------------------
  // shader stuff
  mGLId_shad_vshader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(mGLId_shad_vshader, 1, (const GLchar **) &code_vertex, NULL);
  glCompileShader(mGLId_shad_vshader);
  checkGLShader(mGLId_shad_vshader);
  checkGLErrors("vshade compile");
  // fragment shader
  mGLId_shad_fshader = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(mGLId_shad_fshader, 1, (const GLchar **) &code_fragment, NULL);
  glCompileShader(mGLId_shad_fshader);
  checkGLShader(mGLId_shad_fshader);
  checkGLErrors("fshade compile");
  // program
  mGLID_prog = glCreateProgram();
  glAttachShader(mGLID_prog, mGLId_shad_vshader);
  glAttachShader(mGLID_prog, mGLId_shad_fshader);
  glLinkProgram(mGLID_prog);
  checkGLErrors("prog compile");
  checkGLProgram(mGLID_prog);
  // index the uniforms
  mGLID_uniform_proj = glGetUniformLocation(mGLID_prog, "projection_matrix");
  mGLID_uniform_mview = glGetUniformLocation(mGLID_prog, "modelview_matrix");
  // index the inputs
  mGLID_attrib_coord = glGetAttribLocation(mGLID_prog, "in_coord");
  mGLID_attrib_label = glGetAttribLocation(mGLID_prog, "in_label");



  checkGLErrors("RGBDRenderer : end construct");
}

pcl::people::datasynth::RGBDRenderer::~RGBDRenderer()
{
  // ---------------------
  // texture stuff
  glDeleteFramebuffers(1, &mGLFBOId);
  glDeleteTextures(1, &mDMapTex);
  glDeleteTextures(1, &mLMapTex);
  glDeleteTextures(1, &mGLDepthTex);


  // --------------------
  // shader stuff
  glDeleteProgram(mGLID_prog);
  glDeleteShader(mGLId_shad_fshader);
  glDeleteShader(mGLId_shad_vshader);

  checkGLErrors("RGBDRenderer : destructor");
}

void scaleAndOffsetAndRotate(int     updim,
                             float   scale,
                             float   noffset,
                             pcl::people::datasynth::RGBDRenderer::LabeledVertex *xs)
{
  int offset = (updim + 1) % 3;
  float temp[4][3];
  temp[0][0] = -scale;
  temp[0][1] = -scale;
  temp[0][2] = noffset;
  temp[1][0] = +scale;
  temp[1][1] = -scale;
  temp[1][2] = noffset;
  temp[2][0] = +scale;
  temp[2][1] = +scale;
  temp[2][2] = noffset;
  temp[3][0] = -scale;
  temp[3][1] = +scale;
  temp[3][2] = noffset;

  for (int i = 0; i < 4; ++i)
  {
    xs[i].x = temp[i][(offset + 0) % 3];
    xs[i].y = temp[i][(offset + 1) % 3];
    xs[i].z = temp[i][(offset + 2) % 3];
    xs[i].label = 31;
  }
}

void pcl::people::datasynth::RGBDRenderer::draw(const float         *GLPMat,
    const float         *GLMMat,
    const uint16_t       BackgroundDepth,
    const int            numVertices,
    const int            numTriangles,
    const LabeledVertex *vBuffer,
    const GLuint        *iBuffer,
    uint16_t            *depth,
    uint8_t             *labels)
{
  glViewport(0, 0, mW, mH);


  // -------------
  // bind the framebuffer
  glBindFramebuffer(GL_FRAMEBUFFER, mGLFBOId);

  // -------------
  // set the buffers
  GLenum drawBuffs[] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1 };
  glDrawBuffers(2, drawBuffs);
  // bind the outputs
  glBindFragDataLocation(mGLID_prog, 0, "out_label");
  glBindFragDataLocation(mGLID_prog, 1, "out_depth");
  // -------------
  // clear the buffers
  GLuint clearLmap[] = {31, 31, 31, 31};
  GLuint clearDmap[] = {BackgroundDepth, BackgroundDepth,
                        BackgroundDepth, BackgroundDepth
                       };
  GLfloat clearDBuffer = std::numeric_limits<GLfloat>::max();
  // will clear the index i of the drawbuffs array passed with glDrawBuffers()
  glClearBufferuiv(GL_COLOR, 0, clearLmap);
  glClearBufferuiv(GL_COLOR, 1, clearDmap);
  glClearBufferfv(GL_DEPTH, 0, &clearDBuffer);

  checkGLErrors("RGBDRenderer : draw clear");




  // --------------------
  // do some rendering
  glEnable(GL_DEPTH_TEST);
  glUseProgram(mGLID_prog);
  glUniformMatrix4fv(mGLID_uniform_proj,  1, false, GLPMat);
  glUniformMatrix4fv(mGLID_uniform_mview, 1, false, GLMMat);
  glEnableVertexAttribArray(mGLID_attrib_coord);
  glEnableVertexAttribArray(mGLID_attrib_label);


  // --------------
  // copy to a buffer in the card
  GLuint GLIDbuffer;
  glGenBuffers(1, &GLIDbuffer);
  glBindBuffer(GL_ARRAY_BUFFER, GLIDbuffer);
  glBufferData(GL_ARRAY_BUFFER, numVertices * sizeof(LabeledVertex),
               vBuffer, GL_STATIC_DRAW);
  glVertexAttribPointer(mGLID_attrib_coord, 3, GL_FLOAT, GL_FALSE,
                        sizeof(LabeledVertex), (GLvoid *)0);
  glVertexAttribIPointer(mGLID_attrib_label, 1, GL_UNSIGNED_INT,
                         sizeof(LabeledVertex), (GLvoid *)(0 + 3 * sizeof(GLfloat)));
  glDrawElements(GL_TRIANGLES, 3 * numTriangles, GL_UNSIGNED_INT, iBuffer);


  // --------------
  // draw a plane on the floor ( Y=0)
  //    LabeledVertex floor[4];
  //    scaleAndOffsetAndRotate(0, 10, 0,floor);
  //    glBufferData(GL_ARRAY_BUFFER, 4*sizeof(LabeledVertex),
  //                                                      floor, GL_STATIC_DRAW);
  //    glVertexAttribPointer(mGLID_attrib_coord, 3, GL_FLOAT, GL_FALSE,
  //                                         sizeof(LabeledVertex), (GLvoid*)0);
  //    glVertexAttribIPointer(mGLID_attrib_label, 1, GL_UNSIGNED_INT,
  //                   sizeof(LabeledVertex), (GLvoid*)(0 + 3*sizeof(GLfloat)));
  //    glDrawArrays( GL_QUADS, 0, 4);

  //    // --------------
  //    // draws another plane
  //    scaleAndOffsetAndRotate(floor, 10, 1, -15);
  //    glBufferData(GL_ARRAY_BUFFER, 4*sizeof(LabeledVertex),
  //                                                      floor, GL_STATIC_DRAW);
  //    glVertexAttribPointer(mGLID_attrib_coord, 3, GL_FLOAT, GL_FALSE,
  //                                         sizeof(LabeledVertex), (GLvoid*)0);
  //    glVertexAttribIPointer(mGLID_attrib_label, 1, GL_UNSIGNED_INT,
  //                   sizeof(LabeledVertex), (GLvoid*)(0 + 3*sizeof(GLfloat)));
  //    glDrawArrays( GL_QUADS, 0, 4);

  // --------------
  // clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glDeleteBuffers(1, &GLIDbuffer);

  glDisableVertexAttribArray(mGLID_attrib_coord);
  glDisableVertexAttribArray(mGLID_attrib_label);
  glUseProgram(0);
  glDisable(GL_DEPTH_TEST);

  checkGLErrors("RGBDRenderer : draw");

  // -------------------
  // get the buffers back
  glReadBuffer(GL_COLOR_ATTACHMENT0);
  glReadPixels(0, 0, mW, mH, LMAP_COLORFORMAT, LMAP_COLORTYPE, labels);
  glReadBuffer(GL_COLOR_ATTACHMENT1);
  glReadPixels(0, 0, mW, mH, DMAP_COLORFORMAT, DMAP_COLORTYPE, depth);


  glBindFramebuffer(GL_FRAMEBUFFER, 0);


  checkGLErrors("RGBDRenderer : draw copy");
}
