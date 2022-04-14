// Copyright (C) 2004, 2008 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: IpDiagMatrix.cpp 2269 2013-05-05 11:32:40Z stefan $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-08-13

#include "IpDiagMatrix.hpp"

namespace Ipopt
{

  DiagMatrix::DiagMatrix(const SymMatrixSpace* owner_space)
      :
      SymMatrix(owner_space)
  {}

  DiagMatrix::~DiagMatrix()
  {}

  void DiagMatrix::MultVectorImpl(Number alpha, const Vector &x,
                                  Number beta, Vector &y) const
  {
    //  A few sanity checks
    DBG_ASSERT(Dim()==x.Dim());
    DBG_ASSERT(Dim()==y.Dim());
    DBG_ASSERT(IsValid(diag_));

    // Take care of the y part of the addition
    if ( beta!=0.0 ) {
      y.Scal(beta);
    }
    else {
      y.Set(0.0);  // In case y hasn't been initialized yet
    }

    SmartPtr<Vector> tmp_vec = y.MakeNew();
    tmp_vec->Copy(x);
    tmp_vec->ElementWiseMultiply(*diag_);
    y.Axpy(alpha, *tmp_vec);
  }

  bool DiagMatrix::HasValidNumbersImpl() const
  {
    DBG_ASSERT(IsValid(diag_));
    return diag_->HasValidNumbers();
  }

  void DiagMatrix::ComputeRowAMaxImpl(Vector& rows_norms, bool init) const
  {
    DBG_ASSERT(IsValid(diag_));
    if (init) {
      rows_norms.Copy(*diag_);
      rows_norms.ElementWiseAbs();
    }
    else {
      SmartPtr<Vector> v = diag_->MakeNewCopy();
      v->ElementWiseAbs();
      rows_norms.ElementWiseMax(*v);
    }
  }

  void DiagMatrix::PrintImpl(const Journalist& jnlst,
                             EJournalLevel level,
                             EJournalCategory category,
                             const std::string& name,
                             Index indent,
                             const std::string& prefix) const
  {
    jnlst.Printf(level, category, "\n");
    jnlst.PrintfIndented(level, category, indent,
                         "%sDiagMatrix \"%s\" with %d rows and columns, and with diagonal elements:\n",
                         prefix.c_str(), name.c_str(), Dim());
    if (IsValid(diag_)) {
      diag_->Print(&jnlst, level, category, name, indent+1, prefix);
    }
    else {
      jnlst.PrintfIndented(level, category, indent,
                           "%sDiagonal elements not set!\n", prefix.c_str());
    }
  }
} // namespace Ipopt
