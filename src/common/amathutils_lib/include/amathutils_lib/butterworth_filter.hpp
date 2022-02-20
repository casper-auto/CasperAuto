/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Ali Boyali, Simon Thompson
 */

#ifndef AMATHUTILS_LIB_BUTTERWORTH_FILTER_HPP
#define AMATHUTILS_LIB_BUTTERWORTH_FILTER_HPP

#include <cmath>
#include <complex>
#include <vector>

struct Order_Cutoff
{
  int N;
  double Wc;
};

struct DifferenceAnBn
{
  std::vector<double> An;
  std::vector<double> Bn;
};

class ButterworthFilter
{
public:
  // Prints the filter order and cutoff frequency

  void PrintFilter_Specs();
  void PrintFilter_ContinuousTimeRoots();

  void PrintContinuousTimeTF();
  void PrintDiscreteTimeTF();

  void Buttord(double Wp, double Ws, double Ap, double As);

  // resizes and intialises array for remembering previous filter values
  void initializeForFiltering();

  // Setters and Getters
  void
  setCuttoffFrequency(double Wc);  // Wc is the cut-off frequency in [rad/sec]

  // fc is cut-off frequency in [Hz] and fs is the sampling frequency in [Hz]
  void setCuttoffFrequency(double fc, double fs);
  void setOrder(int N);

  // Get the order, cut-off frequency and other filter properties
  Order_Cutoff getOrderCutOff();
  DifferenceAnBn getAnBn();

  std::vector<double> getAn();
  std::vector<double> getBn();

  // computes continous time transfer function
  void computeContinuousTimeTF(bool sampling_freqency = false);

  // computes continous time transfer function
  void computeDiscreteTimeTF(bool sampling_freqency = false);

  double filter(const double &u);
  void filtVector(const std::vector<double> &t, std::vector<double> &u,
                  bool init_first_value = true);
  void filtFiltVector(const std::vector<double> &t, std::vector<double> &u,
                      bool init_first_value = true);

private:
  int mOrder = 0;                  // filter order
  double mCutoff_Frequency = 0.0;  // filter cut-off frequency [rad/sec]

  // Boolean parameter when a sampling frequency is defined. Default is false
  bool prewarp = false;
  double mSampling_Frequency = 1.0;

  const double Td = 2.0;
  // Gain of the discrete time function
  std::complex<double> mDiscreteTimeGain{1.0, 0.0};

  // Continuous time transfer function roots
  std::vector<double> mPhaseAngles{0.0};
  std::vector<std::complex<double>> mContinuousTimeRoots{{0.0, 0.0}};

  // Discrete time zeros and roots
  std::vector<std::complex<double>> mDiscreteTimeRoots{{0.0, 0.0}};
  std::vector<std::complex<double>> mDiscreteTimeZeros{{-1.0, 0.0}};

  // Continuous time transfer function numerator denominators
  std::vector<std::complex<double>> mContinuousTimeDenominator{{0.0, 0.0}};
  double mContinuousTimeNumerator = 0.0;

  // Discrete time transfer function numerator denominators
  std::vector<std::complex<double>> mDiscreteTimeDenominator{{0.0, 0.0}};
  std::vector<std::complex<double>> mDiscreteTimeNumerator{{0.0, 0.0}};

  // Numerator and Denominator Coefficients Bn and An of Discrete Time Filter

  std::vector<double> mAn{0.0};
  std::vector<double> mBn{0.0};

  // METHODS
  // polynomial function returns the coefficients given the roots of a
  // polynomial
  std::vector<std::complex<double>>
  polynomialFromRoots(std::vector<std::complex<double>> &roots);

  /*
   * Implementation starts by computing the pole locations of the filter in the
   * polar coordinate system . The algorithm first locates the poles  computing
   * the phase angle and then poles as a complex number From the poles, the
   * coefficients of denominator polynomial is calculated.
   *
   * Therefore, without phase, the roots cannot be calculated. The following
   * three methods should be called successively.
   *
   * */

  // computes the filter root locations in the polar coordinate system
  void computePhaseAngles();

  // Computes continuous time roots from the phase angles
  void computeContinuousTimeRoots(bool use_sampling_freqency = false);

  std::vector<double> u_unfiltered;
  std::vector<double> u_filtered;
};

#endif  // AMATHUTILS_LIB_BUTTERWORTH_FILTER_HPP
