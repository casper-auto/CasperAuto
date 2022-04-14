// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: IpTaggedObject.cpp 2473 2014-04-05 17:56:57Z stefan $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-08-13

#include "IpTaggedObject.hpp"

namespace Ipopt
{

  TaggedObject::Tag IPOPT_THREAD_LOCAL TaggedObject::unique_tag_ = 1;

} // namespace Ipopt
