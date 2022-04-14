// Copyright (C) 2008 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: IpInexactRegOp.hpp 1861 2010-12-21 21:34:47Z andreasw $
//
// Authors:  Andreas Waechter            IBM    2008-09-05

#ifndef __IPINEXACTREGOP_HPP__
#define __IPINEXACTREGOP_HPP__

#include "IpSmartPtr.hpp"

namespace Ipopt
{
  class RegisteredOptions;

  void RegisterOptions_Inexact(const SmartPtr<RegisteredOptions>& roptions);

} // namespace Ipopt

#endif
