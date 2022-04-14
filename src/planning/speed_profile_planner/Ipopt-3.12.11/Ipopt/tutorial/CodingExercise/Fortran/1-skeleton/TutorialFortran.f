C Copyright (C) 2009 International Business Machines.
C All Rights Reserved.
C This code is published under the Eclipse Public License.
C
C    $Id: hs071_f.f.in 699 2006-04-05 21:05:18Z andreasw $
C
C Author:  Andreas Waechter               IBM    2009-04-02
C
C =============================================================================
C
C  This file is part of the Ipopt tutorial.  It is the skeleton for
C  the Fortran implemention of the coding exercise problem (in AMPL
C  formulation):
C
C  param n := 4;
C
C  var x {1..n} <= 0, >= -1.5, := -0.5;
C
C  minimize obj:
C    sum{i in 1..n} (x[i]-1)^2;
C
C  subject to constr {i in 2..n-1}:
C    (x[i]^2+1.5*x[i]-i/n)*cos(x[i+1]) - x[i-1] = 0;
C
C  The constant term "i/n" in the constraint is supposed to be input data
C
C =============================================================================
C
C
C =============================================================================
C
C                            Main driver program
C
C =============================================================================
C
      program tutorial
C
      implicit none
C
C     include the Ipopt return codes
C
      include 'IpReturnCodes.inc'
C
C     Size of the problem (number of variables and equality constraints)
C
      integer     NMAX, MMAX, IDX_STY
      parameter  (NMAX = 1000, MMAX = 1000, IDX_STY = 1 )
      integer     N,     M,     NELE_JAC,     NELE_HESS
C
C     Space for multipliers and constraints
C
      double precision LAM(MMAX)
      double precision G(MMAX)
C
C     Vector of variables
C
      double precision X(NMAX)
C
C     Vector of lower and upper bounds
C
      double precision X_L(NMAX), X_U(NMAX), Z_L(NMAX), Z_U(NMAX)
      double precision G_L(MMAX), G_U(MMAX)
C
C     Private data for evaluation routines
C     This could be used to pass double precision and integer arrays untouched
C     to the evaluation subroutines EVAL_*
C
      double precision DAT(MMAX)
      integer IDAT(1)
C
C     Place for storing the Ipopt Problem Handle
C
CC     for 32 bit platforms
C      integer IPROBLEM
C      integer IPCREATE
C     for 64 bit platforms:
      integer*8 IPROBLEM
      integer*8 IPCREATE
C
      integer IERR
      integer IPSOLVE, IPADDSTROPTION
      integer IPADDNUMOPTION, IPADDINTOPTION
      integer IPOPENOUTPUTFILE
C
      double precision f
      integer i
C
C     The following are the Fortran routines for computing the model
C     functions and their derivatives - their code can be found furhter
C     down in this file.
C
      external EV_F, EV_G, EV_GRAD_F, EV_JAC_G, EV_HESS
C
C     Set the problem size
C
      N = 300
C
C     Number of constraints
C
      M = 
C
C     Number of nonzeros in constraint Jacobian
C
      NELE_JAC = 
C
C     Number of nonzeros in Lagrangian Hessian
C
      NELE_HESS = 
C
C     Set initial point and bounds
C

C
C     Set bounds for the constraints
C

C
C     First create a handle for the Ipopt problem (and read the options
C     file)
C
      IPROBLEM = IPCREATE(N, X_L, X_U, M, G_L, G_U, NELE_JAC, NELE_HESS,
     1     IDX_STY, EV_F, EV_G, EV_GRAD_F, EV_JAC_G, EV_HESS)
      if (IPROBLEM.eq.0) then
         write(*,*) 'Error creating an Ipopt Problem handle.'
         stop
      endif
C
C     Open an output file
C
      IERR = IPOPENOUTPUTFILE(IPROBLEM, 'IPOPT.OUT', 5)
      if (IERR.ne.0 ) then
         write(*,*) 'Error opening the Ipopt output file.'
         goto 9000
      endif
C
C     Note: The following options are only examples, they might not be
C           suitable for your optimization problem.
C
C     Set a string option
C
      IERR = IPADDSTROPTION(IPROBLEM, 'mu_strategy', 'adaptive')
      if (IERR.ne.0 ) goto 9990
C
C     Set an integer option
C
      IERR = IPADDINTOPTION(IPROBLEM, 'max_iter', 3000)
      if (IERR.ne.0 ) goto 9990
C
C     Set a double precision option
C
      IERR = IPADDNUMOPTION(IPROBLEM, 'tol', 1.d-7)
      if (IERR.ne.0 ) goto 9990
C
C     Prepare the private data
C
      here we define a
C
C     Call optimization routine
C
      IERR = IPSOLVE(IPROBLEM, X, G, F, LAM, Z_L, Z_U, IDAT, DAT)
C
C     Output:
C
      if( IERR.eq.IP_SOLVE_SUCCEEDED ) then
         write(*,*)
         write(*,*) 'The solution was found.'
         write(*,*)
         write(*,*) 'The final value of the objective function is ',f
         write(*,*)
         write(*,*) 'The optimal values of X are:'
         write(*,*)
         do i = 1, N
            write(*,*) 'X  (',i,') = ',X(i)
         enddo
         write(*,*)
         write(*,*) 'The multipliers for the lower bounds are:'
         write(*,*)
         do i = 1, N
            write(*,*) 'Z_L(',i,') = ',Z_L(i)
         enddo
         write(*,*)
         write(*,*) 'The multipliers for the upper bounds are:'
         write(*,*)
         do i = 1, N
            write(*,*) 'Z_U(',i,') = ',Z_U(i)
         enddo
         write(*,*)
         write(*,*) 'The multipliers for the equality constraints are:'
         write(*,*)
         do i = 1, M
            write(*,*) 'LAM(',i,') = ',LAM(i)
         enddo
         write(*,*)
      else
         write(*,*)
         write(*,*) 'An error occoured.'
         write(*,*) 'The error code is ',IERR
         write(*,*)
      endif
C
 9000 continue
C
C     Clean up
C
      call IPFREE(IPROBLEM)
      stop
C
 9990 continue
      write(*,*) 'Error setting an option'
      goto 9000
      end
C
C =============================================================================
C
C                    Computation of objective function
C
C =============================================================================
C
      subroutine EV_F(N, X, NEW_X, F, IDAT, DAT, IERR)
      implicit none
      integer N, NEW_X
      double precision F, X(N)
      double precision DAT(*)
      integer IDAT(*)
      integer IERR
      integer i

      return
      end
C
C =============================================================================
C
C                Computation of gradient of objective function
C
C =============================================================================
C
      subroutine EV_GRAD_F(N, X, NEW_X, GRAD, IDAT, DAT, IERR)
      implicit none
      integer N, NEW_X
      double precision GRAD(N), X(N)
      double precision DAT(*)
      integer IDAT(*)
      integer IERR
      integer i

      return
      end
C
C =============================================================================
C
C                     Computation of equality constraints
C
C =============================================================================
C
      subroutine EV_G(N, X, NEW_X, M, G, IDAT, DAT, IERR)
      implicit none
      integer N, NEW_X, M
      double precision G(M), X(N)
      double precision DAT(*)
      integer IDAT(*)
      integer IERR
      integer j

      do j = 1, M
         G(j) = (X(j+1)**2 + 1.5d0*X(j+1) - DAT(j))*COS(X(j+2)) - X(j)
      enddo

      IERR = 0
      return
      end
C
C =============================================================================
C
C                Computation of Jacobian of equality constraints
C
C =============================================================================
C
      subroutine EV_JAC_G(TASK, N, X, NEW_X, M, NZ, ACON, AVAR, A,
     1     IDAT, DAT, IERR)
      integer TASK, N, NEW_X, M, NZ
      double precision X(N), A(NZ)
      integer ACON(NZ), AVAR(NZ)
      double precision DAT(*)
      integer IDAT(*)
      integer IERR
      integer j, inz
C
      return
      end
C
C =============================================================================
C
C                Computation of Hessian of Lagrangian
C
C =============================================================================
C
      subroutine EV_HESS(TASK, N, X, NEW_X, OBJFACT, M, LAM, NEW_LAM,
     1     NNZH, IRNH, ICNH, HESS, IDAT, DAT, IERR)
      implicit none
      integer TASK, N, NEW_X, M, NEW_LAM, NNZH
      double precision X(N), OBJFACT, LAM(M), HESS(NNZH)
      integer IRNH(NNZH), ICNH(NNZH)
      double precision DAT(*)
      integer IDAT(*)
      integer IERR
      integer i, inz
C
      return
      end
