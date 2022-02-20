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
 * Aauthor Ali Boyali, Simon Thompson
 */

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

#include "amathutils_lib/butterworth_filter.hpp"

void ButterworthFilter::Buttord(double Wp, double Ws, double Ap, double As)
{
  /*
   *  Inputs are double Wp, Ws, Ap, As;
   *  Wp; passband frequency [rad/sc],
   *  Ws: stopband frequeny [rad/sc],
   *  Ap. {As}: pass {stop} band ripple [dB]
   *
   * */

  double alpha, beta;

  alpha = Ws / Wp;
  beta = sqrt((pow(10, As / 10.0) - 1.0) / (pow(10, Ap / 10.0) - 1.0));
  int order = std::ceil(log(beta) / log(alpha));

  setOrder(order);

  // right limit, left limit
  /*
   * The left and right limits of the magnitutes satisfy the specs at the
   * frequencies Ws and Wp Scipy.buttord gives left limit as the cut-off
   * frequency whereas Matlab gives right limit
   *
   * */
  double right_lim =
      Ws * (pow((pow(10.0, As / 10.0) - 1.0), -1.0 / (2 * mOrder)));
  //    double left_lim = Wp * (pow((pow(10.0, Ap / 10.0) - 1.0), -1.0 / (2 *
  //    mOrder)));

  // mCutoff_Frequency = left_lim;
  setCuttoffFrequency(right_lim);
}

void ButterworthFilter::initializeForFiltering()
{
  // record history of filtered and unfiltered signal for applying filter
  u_unfiltered.resize(mOrder + 1, 0.0);
  u_filtered.resize(mOrder + 1, 0.0);
}
void ButterworthFilter::setOrder(int N)
{
  mOrder = N;
  initializeForFiltering();
}

void ButterworthFilter::setCuttoffFrequency(double Wc)
{
  mCutoff_Frequency = Wc;
}

void ButterworthFilter::setCuttoffFrequency(double fc, double fs)
{
  /*
   * fc is the cut-off frequency in [Hz]
   * fs is the sampling frequency in [Hz]
   * */
  if (fc >= fs / 2)
  {
    std::cout << "\n  ERROR: Cut-off frequency  fc must be less than fs/2 \n";
    _Exit(0);
  }
  mCutoff_Frequency = fc * 2.0 * M_PI;
  mSampling_Frequency = fs;
}

std::vector<std::complex<double>>
ButterworthFilter::polynomialFromRoots(std::vector<std::complex<double>> &rts)
{
  std::vector<std::complex<double>> roots = rts;
  std::vector<std::complex<double>> coefficients(roots.size() + 1, {0, 0});  // NOLINT

  int n = roots.size();

  coefficients[0] = {1.0, 0.0};

  // Use Vieta's Formulas to calculate polynomial coefficients
  // relates sum and products of roots to coefficients
  for (int i = 0; i < n; i++)
  {
    for (int j = i; j != -1; j--)
    {
      coefficients[j + 1] = coefficients[j + 1] - (roots[i] * coefficients[j]);
    }
  }

  return coefficients;
}

void ButterworthFilter::computePhaseAngles()
{
  mPhaseAngles.resize(mOrder);
  int i = 1;

  for (auto &&x : mPhaseAngles)
  {
    x = M_PI_2 + (M_PI * (2.0 * i - 1.0) / (2.0 * mOrder));
    i++;
  }
}

void ButterworthFilter::computeContinuousTimeRoots(bool use_sampling_freqency)
{
  // First compute  the phase angles of the roots
  computePhaseAngles();

  mContinuousTimeRoots.resize(mOrder, {0.0, 0.0});  // NOLINT
  int i = 0;

  if (use_sampling_freqency)
  {
    double Fc = (mSampling_Frequency / M_PI) *
                tan(mCutoff_Frequency / (mSampling_Frequency * 2.0));

    for (auto &&x : mContinuousTimeRoots)
    {
      x = {cos(mPhaseAngles[i]) * Fc * 2.0 * M_PI,   // NOLINT
           sin(mPhaseAngles[i]) * Fc * 2.0 * M_PI};  // NOLINT
      i++;
    }
  }
  else
  {
    for (auto &&x : mContinuousTimeRoots)
    {
      x = {mCutoff_Frequency * cos(mPhaseAngles[i]),   // NOLINT
           mCutoff_Frequency * sin(mPhaseAngles[i])};  // NOLINT
      i++;
    }
  }
}

void ButterworthFilter::computeContinuousTimeTF(bool use_sampling_freqency)
{
  computeContinuousTimeRoots(use_sampling_freqency);
  mContinuousTimeDenominator.resize(mOrder + 1, {0.0, 0.0});  // NOLINT

  mContinuousTimeDenominator = polynomialFromRoots(mContinuousTimeRoots);
  mContinuousTimeNumerator = pow(mCutoff_Frequency, mOrder);
}

void ButterworthFilter::computeDiscreteTimeTF(bool use_sampling_freqency)
{
  /* @brief
   * This method assumes the continous time transfer function of filter has
   * already been computed and stored in the
   * object.
   *
   * */
  // Resizes the roots and zeros to the new order of discrete time then fills
  // the values by Bilinear Transformation

  mDiscreteTimeZeros.resize(
      mOrder, {-1.0, 0.0}); // Butter puts zeros at -1.0 for causality  // NOLINT
  mDiscreteTimeRoots.resize(mOrder, {0.0, 0.0});  // NOLINT
  mAn.resize(mOrder + 1, 0.0);
  mBn.resize(mOrder + 1, 0.0);

  mDiscreteTimeGain = {mContinuousTimeNumerator, 0.0};

  // Bi-linear Transformation of the Roots
  int i = 0;

  if (use_sampling_freqency)
  {
    for (auto &&dr : mDiscreteTimeRoots)
    {
      dr = (1.0 + mContinuousTimeRoots[i] / (mSampling_Frequency * 2.0)) /
           (1.0 - mContinuousTimeRoots[i] / (mSampling_Frequency * 2.0));
      i++;
    }

    mDiscreteTimeDenominator = polynomialFromRoots(mDiscreteTimeRoots);

    // Obtain the coefficients of numerator and denominator
    i = 0;
    mDiscreteTimeNumerator = polynomialFromRoots(mDiscreteTimeZeros);

    // Compute Discrete Time Gain
    std::complex<double> sum_num{0.0, 0.0};
    std::complex<double> sum_den{0.0, 0.0};

    for (auto &n : mDiscreteTimeNumerator)
    {
      sum_num += n;
    }

    for (auto &n : mDiscreteTimeDenominator)
    {
      sum_den += n;
    }

    mDiscreteTimeGain = (sum_den / sum_num);

    for (auto &&dn : mDiscreteTimeNumerator)
    {
      dn = dn * mDiscreteTimeGain;
      mBn[i] = dn.real();
      i++;
    }

    i = 0;
    for (auto &&dd : mDiscreteTimeDenominator)
    {
      mAn[i] = dd.real();
      i++;
    }
  }
  else
  {
    for (auto &&dr : mDiscreteTimeRoots)
    {
      dr = (1.0 + 2.0 * mContinuousTimeRoots[i] / 2.0) /
           (1.0 - Td * mContinuousTimeRoots[i] / 2.0);

      mDiscreteTimeGain = mDiscreteTimeGain / (1.0 - mContinuousTimeRoots[i]);
      i++;
    }

    mDiscreteTimeDenominator = polynomialFromRoots(mDiscreteTimeRoots);

    // Obtain the coefficients of numerator and denominator
    i = 0;
    mDiscreteTimeNumerator = polynomialFromRoots(mDiscreteTimeZeros);

    for (auto &&dn : mDiscreteTimeNumerator)
    {
      dn = dn * mDiscreteTimeGain;
      mBn[i] = dn.real();
      i++;
    }

    i = 0;
    for (auto &&dd : mDiscreteTimeDenominator)
    {
      mAn[i] = dd.real();
      i++;
    }
  }
}

Order_Cutoff ButterworthFilter::getOrderCutOff()
{
  Order_Cutoff NWc{mOrder, mCutoff_Frequency};

  return NWc;
}

DifferenceAnBn ButterworthFilter::getAnBn()
{
  //    DifferenceAnBn AnBn;
  //    AnBn.An.resize(mAn.size(), 0.0);
  //    AnBn.Bn.resize(mBn.size(), 0.0);
  //
  //    AnBn.An = mAn;
  //    AnBn.Bn = mBn;

  DifferenceAnBn AnBn{mAn, mBn};

  return AnBn;
}

std::vector<double> ButterworthFilter::getAn()
{
  return mAn;
}

std::vector<double> ButterworthFilter::getBn()
{
  return mBn;
}

void ButterworthFilter::PrintFilter_Specs()
{
  /*
   * Prints the order and cut-off angular frequency (rad/sec) of the filter
   *
   * */
  std::cout << "\nThe order of the filter : " << this->mOrder << std::endl;
  std::cout << "Cut-off Frequency : " << this->mCutoff_Frequency << " rad/sec\n"
            << std::endl;
}

void ButterworthFilter::PrintFilter_ContinuousTimeRoots()
{
  /*
   * Prints the order and cut-off angular frequency (rad/sec) of the filter
   * */
  std::cout << "\n Roots of Continous Time Filter Transfer Function "
               "Denominator are : "
            << std::endl;

  for (auto &&x : mContinuousTimeRoots)
  {
    std::cout << std::real(x) << " + j " << std::imag(x) << std::endl;
  }
  std::cout << std::endl;
}

void ButterworthFilter::PrintContinuousTimeTF()
{
  int n = mOrder;

  std::cout << "\nThe Continuous Time Transfer Function of the Filter is ;\n"
            << std::endl;

  std::cout << "         " << mContinuousTimeNumerator << std::endl;

  for (int i = 0; i <= n; i++)
  {
    std::cout << "--------";
  }

  std::cout << "--------\n";

  for (int i = n; i > 0; i--)
  {
    std::cout << mContinuousTimeDenominator[n - i].real() << " * ";
    std::cout << "z[-" << i << "] + ";
  }

  std::cout << mContinuousTimeDenominator[n].real() << std::endl;
}

void ButterworthFilter::PrintDiscreteTimeTF()
{
  int n = mOrder;
  std::cout << "\nThe Discrete Time Transfer Function of the Filter is ;\n"
            << std::endl;

  for (int i = n; i > 0; i--)
  {
    std::cout << mDiscreteTimeNumerator[n - i].real() << " * ";
    std::cout << "z[-" << i << "] + ";
  }

  std::cout << mDiscreteTimeNumerator[n].real() << std::endl;

  for (int i = 0; i <= n; i++)
  {
    std::cout << "--------";
  }

  std::cout << "--------\n";

  for (int i = n; i > 0; i--)
  {
    std::cout << mDiscreteTimeDenominator[n - i].real() << " * ";
    std::cout << "z[-" << i << "] + ";
  }

  std::cout << mDiscreteTimeDenominator[n].real() << std::endl;
  std::cout << std::endl;
}

double ButterworthFilter::filter(const double &u)
{
  double u_f = 0.0;

  for (int i = 0; i < mOrder + 1; i++)
  {
    if (i == 0)
    {
      u_f += mBn[0] * u;
    }
    else
    {
      u_f += mBn[i] * u_unfiltered[i];
      u_f -= mAn[i] * u_filtered[i];
    }
  }

  for (int i = mOrder; i >= 2; i--)
  {
    u_unfiltered[i] = u_unfiltered[i - 1];
    u_filtered[i] = u_filtered[i - 1];
  }
  u_unfiltered[1] = u;
  u_filtered[1] = u_f;

  return u_f;
}

void ButterworthFilter::filtVector(const std::vector<double> &t,
                                   std::vector<double> &u,
                                   bool init_first_value)
{
  // initialise trace values to 1st value

  if (init_first_value)
  {
    for (int i = 0; i < mOrder + 1; i++)
    {
      u_unfiltered[i] = t[0];
      u_filtered[i] = t[0];
    }
  }

  for (unsigned int i = 0; i < t.size(); ++i)
    u[i] = this->filter(t[i]);
}

void ButterworthFilter::filtFiltVector(const std::vector<double> &t,
                                       std::vector<double> &u,
                                       bool init_first_value)
{
  std::vector<double> u_rev(u);

  // forward filtering
  filtVector(t, u, init_first_value);

  // backward filtering
  std::reverse(u_rev.begin(), u_rev.end());
  filtVector(t, u_rev, init_first_value);
  std::reverse(u_rev.begin(), u_rev.end());

  // merge
  for (unsigned int i = 0; i < u.size(); ++i)
  {
    u[i] = (u[i] + u_rev[i]) * 0.5;
  }
}
