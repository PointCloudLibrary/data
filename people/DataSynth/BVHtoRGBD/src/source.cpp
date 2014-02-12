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

#include <libBVH/BVHMesh.h>
#include <libBVH/BVHMesh_smooth.h>
#include <libBVH/BVHMotionFile.h>
#include <libBVH/CMUTransforms.h>

#include <RGBDRenderer.h>

#include <OptionParser.h>

#include <cmath>
#include <fstream>
#include <algorithm>
#include <iomanip>
#include <stdexcept>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// boost random
#include <boost/random/mersenne_twister.hpp> // the randomness source
#include <boost/random/uniform_real.hpp> // the distribution
#include <boost/random/variate_generator.hpp>
#include <ctime> // for randomness seeding

using namespace pcl::people::BVH;
using namespace pcl::people::datasynth;

// #############################
// Matrix Building and Loading
// #############################
void buildPMat(float f, int W, int H, float zNear, float zFar, float *PMat)
{
  PMat[0 * 4 + 0] = 2.*f / W;
  PMat[0 * 4 + 1] =  0;
  PMat[0 * 4 + 2] = 0;
  PMat[0 * 4 + 3] = 0;
  PMat[1 * 4 + 0] = 0;
  PMat[1 * 4 + 1] =  2.*f / H;
  PMat[1 * 4 + 2] = 0;
  PMat[1 * 4 + 3] = 0;
  PMat[2 * 4 + 0] = 0.;
  PMat[2 * 4 + 1] =  0;
  PMat[2 * 4 + 2] = (zFar + zNear) / (zFar - zNear);
  PMat[2 * 4 + 3] = 1; //we ll have to add 1/H and 1/W to get the 1/2 pixel offet later
  PMat[3 * 4 + 0] = 0;
  PMat[3 * 4 + 1] =  0;
  PMat[3 * 4 + 2] = -(2 * zFar * zNear) / (zFar - zNear);
  PMat[3 * 4 + 3] = 0;
}

void loadMMat(const std::string &filename, float *MMat)
{
  std::ifstream fin(filename.c_str());
  if (!fin.is_open())
  {
    throw (std::runtime_error(std::string("(E)could not open: ") + filename));
  }
  for (int i = 0; i < 4; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      fin >> MMat[4 * i + j];
    }
  }
  if (fin.fail())
  {
    throw (std::runtime_error(std::string("(E) malformed matrix file: ") + filename));
  }
}

void export_PPM(const char *filename,
                int w, int h,
                const uint8_t *l)
{
  std::ofstream fout(filename);
  fout << "P2\n" << w << " " << h << "\n" << int(*std::max_element(l, l + w * h)) << "\n";
  for (int y = 0; y < h; ++y)
  {
    for (int x = 0; x < w; ++x)
    {
      fout << int(l[w * y + x]) << " ";
    }
    fout << "\n";
  }
}

void exportPPM_depth(const char *filename,
                     int w, int h,
                     const uint16_t *d)
{
  std::ofstream fout(filename);
  fout << "P2\n" << w << " " << h << "\n" << int(*std::max_element(d, d + w * h)) << "\n";
  for (int y = 0; y < h; ++y)
  {
    for (int x = 0; x < w; ++x)
    {
      fout << d[w * y + x] << " ";
    }
    fout << "\n";
  }
}

void printGLMat(const float *M, std::ostream &os)
{
  os << std::fixed;
  os << "[";
  for (int i = 0; i < 3; ++i)
  {
    os << "[";
    for (int j = 0; j < 3; ++j)
    {
      os << M[4 * j + i] << ",";
    }
    os << M[4 * 3 + i] << "],\n";
  }
  os << "[";
  for (int j = 0; j < 3; ++j)
  {
    os << M[4 * j + 3] << ",";
  }
  os << M[4 * 3 + 3] << "]]\n";
}

// #############################
// Main prog
// #############################
// The simulated camera will look down its Z positive
int main(int argc, char **argv)
{
  OptionParser opt(argc, argv);
  std::string MMatFile  = opt.getOption<std::string>("-MMat");
  std::string BVH_ref   = opt.getOption<std::string>("-BVH_ref");
  std::string fmotion   = opt.getOption<std::string>("-MotionFile");
  std::string fverts    = opt.getOption<std::string>("-verts");
  std::string ftris     = opt.getOption<std::string>("-tris");
  std::string fbones    = opt.getOption<std::string>("-bones");
  std::string flabels   = opt.getOption<std::string>("-labels");
  std::string doutbasename = opt.getOption<std::string>("-dout");
  std::string loutbasename = opt.getOption<std::string>("-lout");
  uint16_t    backgroundDepth_mm = opt.getOption<uint16_t>("-bgdepth_mm");
  int         firstFrame         = opt.getOption<int>("-F");
  int         lastFrame          = opt.getOption<int>("-L");
  double      angleRange         = opt.getOption<double>("-angleRange");

  // ---------------------------
  // load BVH reference
  std::vector<bvhJoint> joints_ref;
  std::vector<Vec3>     rest_pose;
  try
  {
    float                             period_ref;
    std::vector<std::vector<float> >  values_ref;
    bvhparseFile(BVH_ref, joints_ref, period_ref, values_ref);
    computeRestStateJointPos(joints_ref, rest_pose);
  }
  catch (std::exception &err)
  {
    std::cout << err.what() << std::endl;
    exit(0);
  }

  // --------------------------
  // load Mesh vectors
  std::vector<Triangle>     tris;
  std::vector<vertex_joint> vjs;
  std::vector<int>          vjs_bounds;
  std::vector<Vec3>         X0;
  std::vector<int>          labels;
  loadMesh_Tri(ftris.c_str(), tris);
  loadMesh_VJ_smooth(fbones.c_str(), joints_ref, vjs, vjs_bounds);
  loadMesh_Vertices(fverts.c_str(),  X0);
  loadMesh_labels(flabels.c_str(), labels);

  // -------------------------
  // create the renderer
  RGBDRenderer renderer;
  RGBDRenderer::LabeledVertex *vBuffer = new RGBDRenderer::LabeledVertex[X0.size()];
  GLuint                      *iBuffer = new GLuint[3 * tris.size()];
  // copy triangle data
  {
    GLuint *t_itr = iBuffer;
    for (size_t ti = 0; ti < tris.size(); ++ti)
    {
      *t_itr++ = tris[ti].v0;
      *t_itr++ = tris[ti].v1;
      *t_itr++ = tris[ti].v2;
    }
  }

  uint16_t *dmap = new uint16_t[640 * 480];
  uint8_t  *lmap = new uint8_t[640 * 480];
  float PMat[16];
  float MMat[16];
  buildPMat(640, 640, 480, 0.1, 100., PMat);
  loadMMat(MMatFile, MMat);

  printGLMat(PMat, std::cout);
  printGLMat(MMat, std::cout);

  // -------------------------
  // create the randomness source
  boost::mt19937 rng; // the random number generator
  rng.seed(static_cast<unsigned int>(std::time(0)));
  boost::uniform_real<double> distrib_a(-angleRange, angleRange);
  boost::variate_generator < boost::mt19937 &,
        boost::uniform_real<double> > angle(rng, distrib_a);

  // -------------------------
  // load the motion file
  BVHMotionFile MFile(fmotion);

  std::vector<float> values;
  for (int frameId = 0; frameId < firstFrame; ++frameId)
  {
    MFile.readNextFrame(joints_ref, values);
  }
  // -------------------------
  // load the new pose
  for (int frameid = firstFrame; frameid <= lastFrame; ++frameid)
  {
    // skip one half of the files horribly
    //MFile.readNextFrame(joints_ref, values);
    // actually get the file
    if (!MFile.readNextFrame(joints_ref, values))
    {
      std::cout << "(E) " << fmotion
                << " ended up after " << frameid << " samples" << std::endl;
      break;
    }
    // compute transforms
    std::vector<Transform3> Tis;
    computeTis(joints_ref, rest_pose, values, Tis);
    // rebase the mesh at the origin and scale it to get everything in meters
    double rangle = angle(); //generate a random angle
    CMU_scaleRotateHips(joints_ref, Tis, rangle);
    // accum
    std::vector<Transform3> TTis;
    computeTTis(joints_ref, Tis, TTis);

    // transform mesh
    std::vector<Vec3> X;
    transformMesh_smooth(TTis, vjs, vjs_bounds, X0, X);
    // copy the mesh to gl
    RGBDRenderer::LabeledVertex *v_itr = vBuffer;
    for (size_t vi = 0; vi < X.size(); ++vi)
    {
      v_itr->x = X[vi](0);
      v_itr->y = X[vi](1);
      v_itr->z = X[vi](2);
      v_itr->label = labels[vi];
      v_itr++;
    }

    // do the actual rendering
    renderer.draw(PMat, MMat, backgroundDepth_mm, X.size(), tris.size(), vBuffer, iBuffer, dmap, lmap) ;

    // save to the file
    //export_PPM("lmap.ppm", 640,480,lmap);
    //exportPPM_depth("dmap.ppm", 640,480,dmap);
    std::string Lout = buildFilename(loutbasename, frameid);
    std::string Dout = buildFilename(doutbasename, frameid);

    cv::Mat LImage(480, 640, CV_8U,  lmap);
    cv::Mat DImage(480, 640, CV_16U, dmap);
    if (!cv::imwrite(Lout.c_str(), LImage))
    {
      std::cout << "(E) BVHtoRGBD : unable to write : Lout filename " << Lout << std::endl;
    }
    if (!cv::imwrite(Dout.c_str(), DImage))
    {
      std::cout << "(E) BVHtoRGBD : unable to write : Dout filename " << Dout << std::endl;
    }

    // tell the world where we stand
    std::cout << "wrote " << frameid << " frame to disk with angle " << rangle << "\r";
    std::cout.flush();
  }

  // -----------------------
  // clean up
  delete[] dmap;
  delete[] lmap;
  delete[] vBuffer;
  delete[] iBuffer;
  return 0;
}

