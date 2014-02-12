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

#include <OpenGLContext.h>
#include <iostream>
#include <cstdlib>

pcl::people::datasynth::OpenGLContext::OpenGLContext(const char *displayName):
  mDisplayName(displayName)
{
  int attrib[] = { GLX_RGBA,
                   GLX_RED_SIZE, 8,
                   GLX_GREEN_SIZE, 8,
                   GLX_BLUE_SIZE, 8,
                   //GLX_DOUBLEBUFFER,
                   GLX_STENCIL_SIZE, 1,
                   GLX_DEPTH_SIZE, 24,
                   None
                 };


  // Open the display
  mDisplay  = XOpenDisplay(displayName);
  if (!mDisplay)
  {
    std::cout << "(ERROR) Could not open display" << std::endl;
  }

  // Create a Visual
  mVinfo    = glXChooseVisual(mDisplay, XDefaultScreen(mDisplay), attrib);
  if (!mVinfo)
  {
    std::cout << "(ERROR) Could not define a proper visual" << std::endl;
  }

  // Create a context
  mCtx      = glXCreateContext(mDisplay,
                               mVinfo,
                               NULL,     // Non sharing resources with another context
                               GL_TRUE); // DRI ON Direct rendering means we are talking directly to the hardware
  // and not being forwarded through X
  if (!mCtx)
  {
    std::cout << "(ERROR) Could not create context" << displayName << std::endl;
  }

  // create Pbuffer
  int nPbufferConfigs = 0;
  GLXFBConfig *pbufferConfigs = glXGetFBConfigs(mDisplay, XDefaultScreen(mDisplay), &nPbufferConfigs);
  int pbufferAttrib[] = { GLX_PBUFFER_WIDTH,  64, GLX_PBUFFER_HEIGHT, 64, None};
  mPBuffer =  glXCreatePbuffer(mDisplay, pbufferConfigs[0], pbufferAttrib);
  XFree(pbufferConfigs);

  // make the context current
  glXMakeCurrent(mDisplay, mPBuffer, mCtx);
  //glewInit();

  XSync(mDisplay, False);

}

pcl::people::datasynth::OpenGLContext::~OpenGLContext()
{
  glXDestroyPbuffer(mDisplay, mPBuffer);
  glXDestroyContext(mDisplay, mCtx);
  XFree(mVinfo);
  XSetCloseDownMode(mDisplay, DestroyAll);
  XCloseDisplay(mDisplay);
}

const std::string &pcl::people::datasynth::OpenGLContext::getDisplayName()const
{
  return mDisplayName;
}

void pcl::people::datasynth::OpenGLContext::makeCurrent()
{
  glXMakeCurrent(mDisplay, mPBuffer, mCtx);
}

