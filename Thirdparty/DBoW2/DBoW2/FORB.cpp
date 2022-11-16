/**
 * File: FORB.cpp
 * Date: June 2012
 * Author: Dorian Galvez-Lopez
 * Description: functions for ORB descriptors
 * License: see the LICENSE.txt file
 *
 * Distance function has been modified 
 *
 */

 
#include <vector>
#include <string>
#include <sstream>

#include "FORB.h"

using namespace std;

namespace DBoW2 {

// --------------------------------------------------------------------------

const int FORB::L=128*4;

void FORB::meanValue(const std::vector<FORB::pDescriptor> &descriptors, 
  FORB::TDescriptor &mean)
{
  if(descriptors.empty())
  {
    mean.release();
    return;
  }
  else if(descriptors.size() == 1)
  {
    mean = descriptors[0]->clone();
  }
  else
  {

    mean.resize(FORB::L, 0);

    float s = descriptors.size();

    vector<FORB::pDescriptor>::const_iterator it;
    for (it = descriptors.begin(); it != descriptors.end(); ++it)
    {
      const FORB::TDescriptor &desc = **it;
      for (int i= 0; i < FORB::L; i+=4)
      {
        mean.at<float>(i+0) += desc.at<float>(i+0);
        mean.at<float>(i+1) += desc.at<float>(i+1);
        mean.at<float>(i+2) += desc.at<float>(i+2);
        mean.at<float>(i+3) += desc.at<float>(i+3);
      }
    }
  }
}

// --------------------------------------------------------------------------
  
float FORB::distance(const FORB::TDescriptor &a,
  const FORB::TDescriptor &b)
{

  float sqd = 0.;
  for(int i = 0; i < FORB::L; i += 4)
  {
    sqd += (a.at<float>(i+0) - b.at<float>(i+0))*(a.at<float>(i+0) - b.at<float>(i+0));
    sqd += (a.at<float>(i+1) - b.at<float>(i+1))*(a.at<float>(i+1) - b.at<float>(i+1));
    sqd += (a.at<float>(i+2) - b.at<float>(i+2))*(a.at<float>(i+2) - b.at<float>(i+2));
    sqd += (a.at<float>(i+3) - b.at<float>(i+3))*(a.at<float>(i+3) - b.at<float>(i+3));

  }
  return sqd;
}

// --------------------------------------------------------------------------
  
std::string FORB::toString(const FORB::TDescriptor &a)
{
  stringstream ss;
  const unsigned char *p = a.ptr<unsigned char>();
  
  for(int i = 0; i < a.cols; ++i, ++p)
  {
    ss << (int)*p << " ";
  }
  
  return ss.str();
}

// --------------------------------------------------------------------------
  
void FORB::fromString(FORB::TDescriptor &a, const std::string &s)
{
  a.create(1, FORB::L, CV_32F);
  unsigned char *p = a.ptr<unsigned char>();
  
  stringstream ss(s);
  for(int i = 0; i < FORB::L; ++i, ++p)
  {
    int n;
    ss >> n;
    
    if(!ss.fail()) 
      *p = (unsigned char)n;
  }
  
}

// --------------------------------------------------------------------------

void FORB::toMat32F(const std::vector<TDescriptor> &descriptors, 
  cv::Mat &mat)
{
  if(descriptors.empty())
  {
    mat.release();
    return;
  }
  
  const size_t N = descriptors.size();
  
  mat.create(N, FORB::L*8, CV_32F);
  float *p = mat.ptr<float>();
  
  for(size_t i = 0; i < N; ++i)
  {
    const int C = descriptors[i].cols;
    const unsigned char *desc = descriptors[i].ptr<unsigned char>();
    
    for(int j = 0; j < C; ++j, p += 8)
    {
      p[0] = (desc[j] & (1 << 7) ? 1 : 0);
      p[1] = (desc[j] & (1 << 6) ? 1 : 0);
      p[2] = (desc[j] & (1 << 5) ? 1 : 0);
      p[3] = (desc[j] & (1 << 4) ? 1 : 0);
      p[4] = (desc[j] & (1 << 3) ? 1 : 0);
      p[5] = (desc[j] & (1 << 2) ? 1 : 0);
      p[6] = (desc[j] & (1 << 1) ? 1 : 0);
      p[7] = desc[j] & (1);
    }
  } 
}

// --------------------------------------------------------------------------

void FORB::toMat8U(const std::vector<TDescriptor> &descriptors, 
  cv::Mat &mat)
{
  mat.create(descriptors.size(), 32, CV_32F);
  
  unsigned char *p = mat.ptr<unsigned char>();
  
  for(size_t i = 0; i < descriptors.size(); ++i, p += 32)
  {
    const unsigned char *d = descriptors[i].ptr<unsigned char>();
    std::copy(d, d+32, p);
  }
  
}

// --------------------------------------------------------------------------

} // namespace DBoW2


