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

#include "GLerrors.h"
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>
#include <iostream>

// #########################################################
// ERROR CHECKING STUFF
// #########################################################

void checkGLErrors(const char *label)
{
  GLenum errCode;
  const GLubyte *errStr;
  if ((errCode = glGetError()) != GL_NO_ERROR)
  {
    errStr = gluErrorString(errCode);
    std::cout << "OpenGL ERROR: ";
    std::cout << errStr;
    std::cout << "Label: ";
    std::cout << label << std::endl;
  }
}

void checkGLProgram(unsigned int progId)
{
  GLsizei length;
  GLchar infoLog[4096];
  glGetProgramInfoLog(progId, 4096, &length, infoLog);
  std::cout << infoLog;
}

void checkGLShader(unsigned int shaderId)
{
  GLsizei length;
  GLchar infoLog[4096];
  glGetShaderInfoLog(shaderId, 4096, &length, infoLog);
  std::cout << infoLog;
}

void checkGLFrameBufferStatus()
{
  GLenum status;
  status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
  switch (status)
  {
  case GL_FRAMEBUFFER_COMPLETE:
    //std::cout<<"GL_FRAMEBUFFER_COMPLETE_EXT"<<std::endl;
    break;
  case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
    std::cout << "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT" << std::endl;
    break;
  case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
    std::cout << "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT" << std::endl;
    break;
    //case GL_FRAMEBUFFER_INCOMPLETE_DUPLICATE_ATTACHMENT_EXT:
    //  std::cout<<"GL_FRAMEBUFFER_INCOMPLETE_DUPLICATE_ATTACHMENT_EXT"<<std::endl;
    //  break;
    //~ case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS:
    //~ std::cout<<"GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS"<<std::endl;
    //~ break;
    //~ case GL_FRAMEBUFFER_INCOMPLETE_FORMATS:
    //~ std::cout<<"GL_FRAMEBUFFER_INCOMPLETE_FORMATS"<<std::endl;
    //~ break;
  case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
    std::cout << "GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER" << std::endl;
    break;
  case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
    std::cout << "GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER" << std::endl;
    break;
  case GL_FRAMEBUFFER_UNSUPPORTED:
    std::cout << "GL_FRAMEBUFFER_UNSUPPORTED" << std::endl;
    break;
    //case GL_FRAMEBUFFER_STATUS_ERROR_EXT:
    //  std::cout<<"GL_FRAMEBUFFER_STATUS_ERROR_EXT"<<std::endl;
    //  break;
  default:
    std::cout << "UNKNOWN FBO ERROR" << std::endl;
    break;
  }
}



