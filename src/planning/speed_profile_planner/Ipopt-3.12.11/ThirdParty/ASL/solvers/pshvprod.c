/*******************************************************************
Copyright (C) 2017 AMPL Optimization, Inc.; written by David M. Gay.

Permission to use, copy, modify, and distribute this software and its
documentation for any purpose and without fee is hereby granted,
provided that the above copyright notice appear in all copies and that
both that the copyright notice and this permission notice and warranty
disclaimer appear in supporting documentation.

The author and AMPL Optimization, Inc. disclaim all warranties with
regard to this software, including all implied warranties of
merchantability and fitness.  In no event shall the author be liable
for any special, indirect or consequential damages or any damages
whatsoever resulting from loss of use, data or profits, whether in an
action of contract, negligence or other tortious action, arising out
of or in connection with the use or performance of this software.
*******************************************************************/

#include "jacpdim.h"

#ifdef __cplusplus
extern "C" {
#endif

 static void
hv_fwd(expr *e)
{
	argpair *da, *dae;
	expr *e1, **ep;
	real dO;

	for(; e; e = e->fwd) {
	    e->aO = e->adO = 0;
	    switch(e->a) {

		case Hv_timesR:
		case Hv_binaryR:
			e->dO.r = e->R.e->dO.r * e->dR;
			break;

		case Hv_timesLR:
		case Hv_binaryLR:
		case Hv_divLR:
			e->dO.r = e->L.e->dO.r*e->dL + e->R.e->dO.r*e->dR;
			break;

		case Hv_timesL:
		case Hv_unary:
			e->dO.r = e->L.e->dO.r * e->dL;
			break;

		case Hv_vararg:
		case Hv_if:
			if ((e1 = ((expr_va *)e)->valf)) {
				hv_fwd(e1);
				e->dO.r = ((expr_va *)e)->vale->dO.r;
				}
			else if ((e1 = ((expr_va *)e)->val) && e1->op != f_OPNUM)
				e->dO.r = e1->dO.r;
			else
				e->dO.r = 0;
			break;

		case Hv_plterm:
			e->dO.r = e->dL * e->R.e->dO.r;
			break;

		case Hv_sumlist:
			ep = e->R.ep;
			for(dO = 0; (e1 = *ep); ep++)
				dO += e1->dO.r;
			e->dO.r = dO;
			break;

		case Hv_func:
			da = ((expr_f *)e)->da;
			dae = ((expr_f *)e)->dae;
			for(dO = 0; da < dae; da++)
				dO += da->e->dO.r * *da->u.v;
			e->dO.r = dO;
			break;

		case Hv_negate:
			e->dO.r = -e->L.e->dO.r;
			break;

		case Hv_plusR:
			e->dO.r = e->R.e->dO.r;
			break;

		case Hv_plusL:
			e->dO.r = e->L.e->dO.r;
			break;

		case Hv_plusLR:
			e->dO.r = e->L.e->dO.r + e->R.e->dO.r;
			break;

		case Hv_minusR:
			e->dO.r = -e->R.e->dO.r;
			break;

		case Hv_minusLR:
			e->dO.r = e->L.e->dO.r - e->R.e->dO.r;
			break;

		default:/*DEBUG*/
			fprintf(Stderr, "bad e->a = %d in hv_fwd\n", e->a);
			exit(1);
		}
	    }
	}

 static void
func_back(expr_f *f)
{
	argpair *da, *da1, *dae;
	expr *e;
	real **fh;
	real aO, adO, t;

	fh = f->fh;
	aO = f->aO;
	adO = f->adO;
	da = f->da;
	for(dae = f->dae; da < dae; da++) {
		e = da->e;
		e->adO += (t = *da->u.v) * adO;
		e->aO += t * aO;
		t = adO*e->dO.r;
		for(da1 = f->da; da1 < dae; da1++) {
			e = da1->e;
			e->aO += t * **fh++;
			}
		}
	}

 static void
funnel_back(ASL_pfgh *asl, cexp *c, expr_v *v, real t)
{
	expr_v **vp, **vp1, **vpe;
	hes_fun *hf;
	ograd *og;
	real *g, *h;
	real aO, adO;

	aO = v->aO = t;
	adO = v->adO;
	hf = c->hfun;
	if ((og = hf->og)) {
		do {
			v = var_e + og->varno;
			v->adO += (t = og->coef) * adO;
			v->aO += t*aO;
			}
			while((og = og->next));
		return;
		}
	g = hf->grdhes;
	h = g + hf->n;
	vp = hf->vp;
	vpe = vp + hf->n;
	do {
		v = *vp++;
		v->adO += (t = *g++) * adO;
		v->aO += t*aO;
		t = adO * v->dO.r;
		vp1 = hf->vp;
		do (*vp1++)->aO += t * *h++;
		   while(vp1 < vpe);
		}
		while(vp < vpe);
	}

 static void
hv_back(expr *e)
{
	expr *e1, **ep, *e2;
	real adO, t1, t2;

	if (!e || (!e->aO && !e->adO))
		return;
	for(; e; e = e->bak)
	    switch(e->a) {
		case Hv_binaryR:
			e1 = e->R.e;
			e1->adO += e->adO * e->dR;
			e1->aO += e->aO * e->dR  +  e->adO * e1->dO.r * e->dR2;
			break;

		case Hv_binaryLR:
			e1 = e->L.e;
			e2 = e->R.e;
			adO = e->adO;
			t1 = adO * e1->dO.r;
			t2 = adO * e2->dO.r;
			e1->aO  += e->aO*e->dL + t1*e->dL2 + t2*e->dLR;
			e2->aO  += e->aO*e->dR + t1*e->dLR + t2*e->dR2;
			e1->adO += adO * e->dL;
			e2->adO += adO * e->dR;
			break;

		case Hv_divLR:
			e1 = e->L.e;
			e2 = e->R.e;
			adO = e->adO;
			t1 = adO * e1->dO.r;
			t2 = adO * e2->dO.r;
			e1->aO  += e->aO*e->dL + t2*e->dLR;
			e2->aO  += e->aO*e->dR + t1*e->dLR + t2*e->dR2;
			e1->adO += adO * e->dL;
			e2->adO += adO * e->dR;
			break;

		case Hv_unary:
			e1 = e->L.e;
			e1->adO += e->adO * e->dL;
			e1->aO  += e->aO * e->dL  +  e->adO * e1->dO.r * e->dL2;
			break;

		case Hv_vararg:
		case Hv_if:
			if ((e1 = ((expr_va *)e)->vale)) {
				e1->aO = e->aO;
				e1->adO = e->adO;
				hv_back(e1);
				}
			else {
				e1 = ((expr_va *)e)->val;
				if (e1->op != f_OPNUM) {
					e1->aO += e->aO;
					e1->adO += e->adO;
					}
				}
			break;

		case Hv_plterm:
			e->R.e->aO += e->dL * e->aO;
			break;

		case Hv_sumlist:
			ep = e->R.ep;
			t1 = e->aO;
			t2 = e->adO;
			while((e1 = *ep++)) {
				e1->aO += t1;
				e1->adO += t2;
				}
			break;

		case Hv_func:
			func_back((expr_f *)e);
			break;

		case Hv_negate:
			e1 = e->L.e;
 neg_end:
			e1->aO -= e->aO;
			e1->adO -= e->adO;
			break;

		case Hv_plusR:
			e1 = e->R.e;
			goto plus_end;

		case Hv_plusL:
			e1 = e->L.e;
 plus_end:
			e1->aO += e->aO;
			e1->adO += e->adO;
			break;

		case Hv_plusLR:
			e1 = e->L.e;
			e1->aO += t1 = e->aO;
			e1->adO += t2 = e->adO;
			e1 = e->R.e;
			e1->aO += t1;
			e1->adO += t2;
			break;

		case Hv_minusR:
			e1 = e->R.e;
			goto neg_end;

		case Hv_minusLR:
			e1 = e->L.e;
			e1->aO += t1 = e->aO;
			e1->adO += t2 = e->adO;
			e1 = e->R.e;
			e1->aO -= t1;
			e1->adO -= t2;
			break;

		case Hv_timesR:
			e1 = e->R.e;
			e1->adO += e->adO * (t1 = e->dR);
			e1->aO += e->aO * t1;
			break;

		case Hv_timesL:
			e1 = e->L.e;
			e1->adO += e->adO * (t1 = e->dL);
			e1->aO  += e->aO * t1;
			break;

		case Hv_timesLR:
			e1 = e->L.e;
			e2 = e->R.e;
			adO = e->adO;
			e1->aO  += e->aO*e->dL  +  adO * e2->dO.r;
			e2->aO  += e->aO*e->dR  +  adO * e1->dO.r;
			e1->adO += adO * e->dL;
			e2->adO += adO * e->dR;
			break;

		default:/*DEBUG*/
			fprintf(Stderr, "bad e->a = %d in hv_back\n", e->a);
			exit(1);
		}
	}

 static void
hv_fwd0(ASL_pfgh *asl, cexp *c, expr_v *v)
{
	expr_v **vp, **vpe;
	hes_fun *hf;
	int i;
	linarg *la;
	linpart *L, *Le;
	ograd *og;
	real *g, x;

	v->aO = v->adO = 0;
	if ((hf = c->hfun)) {
		x = 0;
		if ((og = hf->og))
			do x += og->coef * var_e[og->varno].dO.r;
			while((og = og->next));
		else {
			g = hf->grdhes;
			vp = hf->vp;
			vpe = vp + hf->n;
			do x += *g++ * (*vp++)->dO.r;
			   while(vp < vpe);
			}
		}
	else if (c->ef) {
		hv_fwd(c->ef);
		x = c->ee->dO.r;
		}
	else if (c->e->op != f_OPNUM)
		x = c->e->dO.r;
	else
		x = 0;
	if ((la = c->la)) {
		i = c - asl->I.cexps2_;
		x += asl->P.dv[i].scale * la->v->dO.r;
		}

	else if ((L = c->L)) {
		for(Le = L + c->nlin; L < Le; L++)
			x += L->fac * ((expr_v*)L->v.vp)->dO.r;
		}
	v->dO.r = x;
	}

 static void
hfg_fwd(expr *e)
{
	expr *e1;

	for(; e; e = e->fwd) {
		e->aO = 0;
		switch(e->a) {
		  case Hv_vararg:
		  case Hv_if:
			if ((e1 = ((expr_va *)e)->valf))
				hfg_fwd(e1);
		  }
		}
	}

 static void
hfg_back(expr *e)
{
	expr *e1, **ep;
	real aO;

	if (!e || !e->aO)
		return;
	for(; e; e = e->bak)
	    switch(e->a) {
		case Hv_timesR:
		case Hv_binaryR:
			e->R.e->aO += e->aO * e->dR;
			break;

		case Hv_binaryLR:
		case Hv_timesLR:
		case Hv_divLR:
			aO = e->aO;
			e->L.e->aO += aO * e->dL;
			e->R.e->aO += aO * e->dR;
			break;

		case Hv_unary:
		case Hv_timesL:
			e->L.e->aO += e->aO * e->dL;
			break;

		case Hv_vararg:
		case Hv_if:
			if ((e1 = ((expr_va *)e)->vale)) {
				e1->aO = e->aO;
				hfg_back(e1);
				}
			else {
				e1 = ((expr_va *)e)->val;
				if (e1->op != f_OPNUM)
					e1->aO += e->aO;
				}
			break;

		case Hv_plterm:
			e->R.e->aO += e->dL * e->aO;
			break;

		case Hv_sumlist:
			ep = e->R.ep;
			aO = e->aO;
			while((e1 = *ep++))
				e1->aO += aO;
			break;

		case Hv_func:
			func_back((expr_f *)e);
			break;

		case Hv_negate:
			e->L.e->aO -= e->aO;
			break;

		case Hv_plusR:
			e->R.e->aO += e->aO;
			break;

		case Hv_plusL:
			e->L.e->aO += e->aO;
			break;

		case Hv_plusLR:
			e->L.e->aO += aO = e->aO;
			e->R.e->aO += aO;
			break;

		case Hv_minusR:
			e->R.e->aO -= e->aO;
			break;

		case Hv_minusLR:
			e->L.e->aO += aO = e->aO;
			e->R.e->aO -= aO;
			break;

		default:/*DEBUG*/
			fprintf(Stderr, "bad e->a = %d in hfg_back\n", e->a);
			exit(1);
		}
	}

 static void
funnelhes(ASL_pfgh *asl)
{
	cexp *c;
	expr *e;
	expr_v *v, **vp, **vp1, **vpe;
	hes_fun *hf;
	int n;
	real *g, *h;

	x0kind &= ~ASL_need_funnel;
	for(hf = asl->I.hesthread; hf; hf = hf->hfthread) {
		if (hf->og)
			continue;
		n = hf->n;
		g = hf->grdhes;
		h = g + n;
		c = hf->c;
		vp = hf->vp;
		vpe = vp + n;

		do (*vp++)->aO = 0;
		   while(vp < vpe);

		hfg_fwd(c->ef);
		e = c->ee;
		e->aO = 1;
		hfg_back(e);

		vp = hf->vp;
		do {
			v = *vp++;
			*g++ = v->aO;
			v->dO.r = v->aO = v->adO = 0;
			}
			while(vp < vpe);

		vp = hf->vp;
		do {
			v = *vp++;
			v->dO.r = 1;
			if ((e = c->ef))
				hv_fwd(e);
			if ((e = c->ee)) {
				e->aO = 0;
				e->adO = 1;
				hv_back(e);
				}
			else if ((e = c->e)->op != f_OPNUM) {
				e->aO = 0;
				e->adO = 1;
				}
			v->dO.r = 0;
			vp1 = hf->vp;
			do {
				v = *vp1++;
				*h++ = v->aO;
				v->aO = v->adO = 0;
				}
				while(vp1 < vpe);
			}
			while(vp < vpe);
		}
	}

 static void
hvp0comp_ASL(ASL_pfgh *asl, real *hv, real *p, int nobj, real *ow, real *y)
	/* p = direction */
	/* y = Lagrange multipliers */
	/* hv = result */
{
	cexp *c, *c1, *ce;
	expr *e;
	expr_v *v, *x, *x0, *xe;
	int *dvsp0, i, i0, i1, n, nc, no, noe;
	linarg *la;
	linpart *L, *Le;
	ograd *og;
	ps_func *f, *f0;
	psb_elem *b, *be;
	psg_elem *g, *ge;
	real *cscale, t, t2, *p1;

#ifdef IGNORE_BOGUS_WARNINGS
	c1 = ce = 0;
	dvsp0 = 0;
	i1 = 0;
#endif

	if (x0kind & ASL_need_funnel)
		funnelhes(asl);
	if (nobj >= 0 && nobj < n_obj) {
		ow = ow ? ow + nobj : &edag_one_ASL;
		no = nobj;
		noe = no + 1;
		}
	else {
		no = noe = 0;
		if (ow)
			noe = n_obj;
		}
	n = c_vars >= o_vars ? c_vars : o_vars;
	for(la = asl->P.lalist; la; la = la->lnext) {
		og = la->nz;
		t = p[og->varno]*og->coef;
		while((og = og->next))
			t += p[og->varno]*og->coef;
		x = la->v;
		x->dO.r = t;
		x->aO = x->adO = 0;
		}
	p1 = p;
	x = x0 = var_e;
	for(xe = x + n; x < xe; x++) {
		x->dO.r = *p1++;
		x->aO = x->adO = 0.;
		}
	if (asl->P.ncom) {
		x = var_ex;
		dvsp0 = asl->P.dvsp0;
		c = cexps;
		i0 = *dvsp0;
		for(ce = c1 = c + asl->P.ncom; c < ce; c++) {
			for(i1 = *++dvsp0; i0 < i1; i0++)
				hv_fwd0(asl, c1++, asl->P.vp[i0]);
			hv_fwd0(asl, c, x++);
			}
		}
	if (!y || (nc = n_con) <= 0)
		goto no_y;
	cscale = asl->i.lscale;
	f0 = asl->P.cps;
	for(i = 0; i < nc; ++i) {
	    if ((t2 = y[i])) {
		if (cscale)
			t2 *= cscale[i];
		f = f0 + i;
		for(b = f->b, be = b + f->nb; b < be; b++) {
			if ((e = b->D.ef)) {
				hv_fwd(e);
				e = b->D.ee;
				e->aO = 0;
				e->adO = t2;
				hv_back(e);
				}
			else if ((e = b->D.e)->op != f_OPNUM) {
				e->aO = 0;
				e->adO = t2;
				}
			}
		for(g = f->g, ge = g + f->ng; g < ge; g++) {
			for(b = g->E, be = b + g->ns; b < be; b++) {
				if ((e = b->D.ef)) {
					hv_fwd(e);
					e = b->D.ee;
					e->aO = 0;
					e->adO = t2*g->g1;
					hv_back(e);
					}
				else if ((e = b->D.e)->op != f_OPNUM) {
					e->aO = 0;
					e->adO = t2*g->g1;
					}
				}
			if (g->g2) {
				t = 0.;
				for(og = g->og; og; og = og->next)
					t += og->coef * p[og->varno];
				t *= t2*g->g2;
				for(og = g->og; og; og = og->next)
					x0[og->varno].aO += t*og->coef;
				}
			}
		}
	    }
 no_y:
	for(; no < noe; no++) {
	    if ((t2 = *ow++)) {
		f = asl->P.ops + no;
		for(b = f->b, be = b + f->nb; b < be; b++) {
			if ((e = b->D.ef)) {
				hv_fwd(e);
				e = b->D.ee;
				e->aO = 0;
				e->adO = t2;
				hv_back(e);
				}
			else if ((e = b->D.e)->op != f_OPNUM) {
				e->aO = 0;
				e->adO = t2;
				}
			}
		for(g = f->g, ge = g + f->ng; g < ge; g++) {
			for(b = g->E, be = b + g->ns; b < be; b++) {
				if ((e = b->D.ef)) {
					hv_fwd(e);
					e = b->D.ee;
					e->aO = 0;
					e->adO = t2*g->g1;
					hv_back(e);
					}
				else if ((e = b->D.e)->op != f_OPNUM) {
					e->aO = 0;
					e->adO = t2*g->g1;
					}
				}
			if (g->g2) {
				t = 0.;
				for(og = g->og; og; og = og->next)
					t += og->coef * p[og->varno];
				t *= t2*g->g2;
				for(og = g->og; og; og = og->next)
					x0[og->varno].aO += t*og->coef;
				}
			}
		}
	    }
	if (asl->P.ncom) {
	    for(c = cexps; c < ce--; ) {
		for(i0 = *--dvsp0; i0 < i1; ) {
			v = asl->P.vp[--i1];
			--c1;
			if ((t = v->aO) && (L = c1->L))
				for(Le = L + c1->nlin; L < Le; L++)
					((expr_v*)L->v.vp)->aO += t * L->fac;
			if (c1->hfun)
				funnel_back(asl, c1, v, t);
			else if ((e = c1->ee)) {
				e->aO = t;
				e->adO = v->adO;
				hv_back(e);
				}
			else if ((e = c1->e)->op != f_OPNUM) {
				e->aO = t;
				e->adO = v->adO;
				}
			}
		if ((t = (--x)->aO) && (L = ce->L))
			for(Le = L + ce->nlin; L < Le; L++)
				((expr_v*)L->v.vp)->aO += t * L->fac;
		if (ce->hfun)
			funnel_back(asl, ce, x, t);
		else if ((e = ce->ee)) {
			e->aO = t;
			e->adO = x->adO;
			hv_back(e);
			}
		else if ((e = ce->e)->op != f_OPNUM) {
			e->aO = t;
			e->adO = x->adO;
			}
		}
	    }
	x = var_e;
	for(la = asl->P.lalist; la; la = la->lnext)
		if ((t = la->v->aO)) {
			og = la->nz;
			do x[og->varno].aO += t*og->coef;
				while((og = og->next));
			}
	while(x < xe)
		*hv++ = (x++)->aO;
	}

 static real *	/* Compute vector x0 = mat(h)*y0,	*/
		/* where h = upper triang of mat(h).	*/
dtmul(int n, real *x0, real *h, real *y0)
{
	int i;
	real *hi, t, *x, *y, *y1, yi;

	y1 = y0;
	--h;
	for(i = 0; i < n; i++) {
		hi = ++h + i;
		yi = *y1++;
		t = yi**hi;
		x = x0;
		y = y0;
		while(h < hi) {
			t += *y++**h;
			*x++ += yi**h++;
			}
		*x = t;
		}
	return x0;
	}

 void
hvpcomp_ASL(ASL *a, real *hv, real *p, int nobj, real *ow, real *y)
	/* p = direction */
	/* y = Lagrange multipliers */
	/* hv = result */
{
	ASL_pfgh *asl;
	Ihinfo *ihi;
	expr_v *v;
	int kp, kw, n, no, noe, ns, nv, *ui, *uie;
	linarg *la, **lap, **lape;
	ograd *og;
	ps_func *ps, *pe;
	psg_elem *g, *ge;
	range *r;
	real *cscale, *owi, t, t1, t2, *p0, *s, *w, *wi, *x;

	ASL_CHECK(a, ASL_read_pfgh, "hvpcomp");
	asl = (ASL_pfgh*)a;
	xpsg_check_ASL(asl, nobj, ow, y);
	nv = n_var;
	kp = htcl(nv*sizeof(real));
	p0 = 0;
	if ((s = asl->i.vscale)) {
		p0 = (real*)new_mblk(kp);
		for(n = 0; n < nv; n++)
			p0[n] = s[n] * p[n];
		p = p0;
		}
	if (!asl->P.ihdcur) {
		if (asl->P.ndhmax <= 0) {
			hvp0comp_ASL(asl,hv,p,nobj,ow,y);
			goto done;
			}
		if (!(n = asl->P.nhvprod))
			n = asl->P.ihdmin;
		if (n >= asl->P.ihdmin)
			hvpinit_ASL(a, ihd_limit, nobj, ow, y);
		}
	asl->P.nhvprod++;
	memset(hv, 0, nv*sizeof(real));
	for(la = asl->P.lalist; la; la = la->lnext) {
		og = la->nz;
		t = p[og->varno]*og->coef;
		while((og = og->next))
			t += p[og->varno]*og->coef;
		v = la->v;
		v->dO.r = t;
		v->aO = v->adO = 0;
		}
	kw = kp + 1;
	w = (real*)new_mblk(kw);
	x = w + n_var;
	s = asl->P.dOscratch;
	ns = 0;
	for(ihi = asl->P.ihi1; ihi; ihi = ihi->next) {
		r = ihi->r;
		if (ihi->hest)
		    for(; r; r = r->rlist.prev) {
			n = r->n;
			nv = r->nv;
			wi = w;
			if (n < nv) {
				lap = r->lap;
				lape = lap + n;
				do {
					og = (*lap++)->nz;
					t = p[og->varno]*og->coef;
					while((og = og->next))
						t += p[og->varno]*og->coef;
					*wi++ = t;
					}
					while(lap < lape);
				wi = dtmul(n, x, r->hest, w);
				lap = r->lap;
				do if ((t = *wi++)) {
					og = (*lap)->nz;
					do hv[og->varno] +=
						t*og->coef;
					   while((og = og->next));
					}
					while(++lap < lape);
				}
			else {
				ui = r->ui;
				uie = ui + nv;
				do *wi++ = p[*ui++];
					while(ui < uie);
				wi = dtmul(nv, x, r->hest, w);
				ui = r->ui;
				do hv[*ui++] += *wi++;
					while(ui < uie);
				}
			}
		else
		    for(; r; r = r->rlist.prev) {
			n = r->n;
			if (ns < n)
				ns = n;
			wi = s;
			lap = r->lap;
			lape = lap + n;
			do {
				og = (*lap++)->nz;
				t = p[og->varno]*og->coef;
				while((og = og->next))
					t += p[og->varno]*og->coef;
				*wi++ = t;
				}
				while(lap < lape);
			pshv_prod_ASL(asl, r, nobj, ow, y);
			lap = r->lap;
			do {
				la = *lap++;
				if ((t = la->v->aO)) {
					og = la->nz;
					do hv[og->varno] += t*og->coef;
						while((og = og->next));
					}
				}
				while(lap < lape);
			}
		}
	del_mblk(kw,w);
	wi = s + ns;
	while(wi > s)
		*--wi = 0.;
	if (asl->P.nobjgroups) {
	    if (nobj >= 0 && nobj < n_obj) {
		owi = ow ? ow + nobj : &edag_one_ASL;
		no = nobj;
		noe = no + 1;
		}
	    else {
		nobj = -1;
		no = noe = 0;
		if ((owi = ow))
			noe = n_obj;
		}
	    for(; no < noe; no++)
		if ((t = *owi++)) {
		    ps = asl->P.ops + no;
		    g = ps->g;
		    for(ge = g + ps->ng; g < ge; g++)
			if ((t2 = g->g2) && (og = g->og)) {
				t1 = p[og->varno]*og->coef;
				while((og = og->next))
					t1 += p[og->varno]*og->coef;
				t2 *= t*t1;
				og = g->og;
				do hv[og->varno] += t2*og->coef;
					while((og = og->next));
				}
		}
	    }
	if (asl->P.ncongroups && y) {
		cscale = a->i.lscale;
		ps = asl->P.cps;
		for(pe = ps + n_con; ps < pe; ps++, y++)
		    if ((t = cscale ? *cscale++ * *y : *y))
			for(g = ps->g, ge = g + ps->ng; g < ge; g++)
			    if ((t2 = g->g2) && (og = g->og)) {
				t1 = p[og->varno]*og->coef;
				while((og = og->next))
					t1 += p[og->varno]*og->coef;
				t2 *= t*t1;
				og = g->og;
				do hv[og->varno] += t2*og->coef;
					while((og = og->next));
				}
		}
	v = var_e;
	for(la = asl->P.lalist; la; la = la->lnext)
		if ((t = la->v->aO)) {
			og = la->nz;
			do v[og->varno].aO += t*og->coef;
				while((og = og->next));
			}
 done:
	if (p0) {
		del_mblk(kp, p0);
		s = asl->i.vscale;
		w = hv + n_var;
		while(hv < w)
			*hv++ *= *s++;
		}
	}

 void
pshv_prod_ASL(ASL_pfgh *asl, range *r, int nobj, real *ow, real *y)
{
	cexp *c;
	expr *e;
	expr_v *v;
	int *cei, *cei0, *ceie, i;
	linarg *la, **lap, **lape;
	linpart *L, *Le;
	ps_func *p;
	psb_elem *b;
	psg_elem *g;
	real *cscale, *s, owi, t;

	cscale = asl->i.lscale;
	owi = 1.;
	if (nobj >= 0 && nobj < n_obj) {
		if (ow) {
			if ((owi = ow[nobj]) == 0.)
				nobj = -1;
			ow = 0;
			}
		}
	if (x0kind & ASL_need_funnel)
		funnelhes(asl);
	s = asl->P.dOscratch;
	lap = r->lap;
	lape = lap + r->n;
	while(lap < lape) {
		la = *lap++;
		v = la->v;
		v->dO.r = *s++;
		v->adO = v->aO = 0.;
		}
	if ((cei = cei0 = r->cei)) {
		i = *cei0++;
		ceie = (cei = cei0) + i;
		do {
			i = *cei++;
			hv_fwd0(asl, cexps + i, asl->P.vp[i]);
			}
			while(cei < ceie);
		}
	for(b = r->refs; b; b = b->next) {
		if ((i = b->conno) < 0) {
			i = -2 - i;
			if (i == nobj)
				t = owi;
			else if (ow) {
				if (!(t = ow[i]))
					continue;
				}
			else
				continue;
			p = asl->P.ops;
			}
		else {
			if (!y || !(t = y[i]))
				continue;
			if (cscale)
				t *= cscale[i];
			p = asl->P.cps;
			}
		if (b->groupno) {
			p += i;
			g = p->g + (b->groupno - 1);
			if (asl->P.pshv_g1)
				t *= g->g1;
			}
		if ((e = b->D.ef)) {
			hv_fwd(e);
			e = b->D.ee;
			e->aO = 0;
			e->adO = t;
			hv_back(e);
			}
		else if ((e = b->D.e)->op != f_OPNUM)
			e->adO += t;
		}
	while(cei > cei0) {
		i = *--cei;
		c = cexps + i;
		v = asl->P.vp[i];
		if ((t = v->aO) && (L = c->L)) {
		    if ((la = c->la))
			la->v->aO += t * asl->P.dv[i].scale;
		    else {
			for(Le = L + c->nlin; L < Le; L++)
				((expr_v*)L->v.vp)->aO += t * L->fac;
			}
		    }
		if (c->hfun)
			funnel_back(asl, c, v, t);
		else if ((e = c->ee)) {
			e->aO = t;
			e->adO = v->adO;
			hv_back(e);
			}
		else if ((e = c->e)->op != f_OPNUM) {
			e->aO += t;
			e->adO += v->adO;
			}
		}
	}

 void
funpset_ASL(ASL_pfgh *asl, funnel *f)
{
	cplist	*cl;
	derp	*d;

	for(; f; f = f->next) {
		memset(adjoints_nv1, 0, f->fcde.zaplen);
		cl = f->cl;
		do *cl->ca.rp = 0;
			while((cl = cl->next));
		d = f->fcde.d;
		*d->b.rp = 1.;
		do *d->a.rp += *d->b.rp * *d->c.rp;
			while((d = d->next));
		cl = f->cl;
		do *cl->cfa = *cl->ca.rp;
			while((cl = cl->next));
		}
	}

 void
hvpcompd_ASL(ASL *a, real *hv, real *p, int co)
	/* p = direction */
	/* hv = result */
	/* co >= 0: behave like hvpcomp_ASL with nobj = -1, ow = 0, y[i] = i == co ? 1. : 0. */
	/* co < 0: behave like hvpcomp_ASL with nobj = -1 - co, ow = 0, y = 0 */
{
	ASL_pfgh *asl;
	cexp *c, *c1, *ce;
	cgrad *cg, *cg0;
	expr *e;
	expr_v *v, *x, *x0;
	int *dvsp0, i0, i1, kp, n, no, nx, oxk;
	linarg *la;
	linpart *L, *Le;
	ograd *og, *og0;
	ps_func *f;
	psb_elem *b, *be;
	psg_elem *g, *ge;
	real *s, t, t2, *p0;
	varno_t i;

#ifdef IGNORE_BOGUS_WARNINGS
	c1 = ce = 0;
	dvsp0 = 0;
	i1 = 0;
	v = 0;
	x = 0;
#endif
	ASL_CHECK(a, ASL_read_pfgh, "hvpcompi");
	asl = (ASL_pfgh*)a;
	if (x0kind == ASL_first_x) {
		if (!(s = X0))
			memset(s = Lastx, 0, n_var*sizeof(real));
		co_index = co;
		xp_check_ASL(asl, s);
		}
	nx = asl->i.nxval;
	oxk = asl->i.x_known;
	asl->i.x_known = 1;

	p0 = 0;
	cg0 = 0;
	og0 = 0;
	x0 = var_e;
	n = c_vars >= o_vars ? c_vars : o_vars;
	t2 = 1.;
	memset(hv, 0, n_var*sizeof(real));
	for(la = asl->P.lalist; la; la = la->lnext) {
		og = la->nz;
		t = p[og->varno]*og->coef;
		while((og = og->next))
			t += p[og->varno]*og->coef;
		x = la->v;
		x->dO.r = t;
		x->aO = x->adO = 0;
		}
	if (co >= 0) {
		if (co >= nlc)
			return;
		f = asl->P.cps + co;
		if (asl->i.ncxval[co] != nx)
			conpival_ASL(a, co, Lastx, 0);
		if (f->ng && f->nxval != nx)
			conpgrd_ASL(a, co, Lastx, 0, 0);
		if ((s = asl->i.lscale))
			t2 = s[co];
		cg = cg0 = Cgrad[co];
		if ((s = asl->i.vscale)) {
			kp = htcl(n*sizeof(real));
			p0 = (real*)new_mblk(kp);
			for(; cg; cg = cg->next) {
				i = cg->varno;
				x = x0 + i;
				x->dO.r = p0[i] = p[i]*s[i];
				x->aO = x->adO = 0.;
				}
			p = p0;
			}
		else {
			for(; cg; cg = cg->next) {
				i = cg->varno;
				x = x0 + i;
				x->dO.r = p[i];
				x->aO = x->adO = 0.;
				}
			}
		}
	else {
		no = -1 - co;
		if (no >= nlo)
			return;
		f = asl->P.ops + no;
		if (asl->i.ncxval[no] != nx)
			objpval_ASL(a, no, Lastx, 0);
		if (f->ng && f->nxval != nx)
			objpgrd_ASL(a, no, Lastx, 0, 0);
		og = og0 = Ograd[no];
		if ((s = asl->i.vscale)) {
			kp = htcl(n*sizeof(real));
			p0 = (real*)new_mblk(kp);
			for(; og; og = og->next) {
				i = og->varno;
				x = x0 + i;
				x->dO.r = p0[i] = p[i]*s[i];
				x->aO = x->adO = 0.;
				}
			p = p0;
			}
		else {
			for(; og; og = og->next) {
				i = og->varno;
				x = x0 + i;
				x->dO.r = p[i];
				x->aO = x->adO = 0.;
				}
			}
		}
	if (asl->i.Derrs) {
		asl->i.x_known = oxk;
		deriv_errchk_ASL(a, 0, co, 1);
		asl->i.x_known = 1;
		}
	if (asl->P.ncom) {
		x = var_ex;
		dvsp0 = asl->P.dvsp0;
		c = cexps;
		i0 = *dvsp0;
		for(ce = c1 = c + asl->P.ncom; c < ce; c++) {
			for(i1 = *++dvsp0; i0 < i1; i0++)
				hv_fwd0(asl, c1++, asl->P.vp[i0]);
			hv_fwd0(asl, c, x++);
			}
		}
	for(b = f->b, be = b + f->nb; b < be; b++) {
		if ((e = b->D.ef)) {
			hv_fwd(e);
			e = b->D.ee;
			e->aO = 0;
			e->adO = t2;
			hv_back(e);
			}
		else if ((e = b->D.e)->op != f_OPNUM) {
			e->aO = 0;
			e->adO = t2;
			}
		}
	for(g = f->g, ge = g + f->ng; g < ge; g++) {
		for(b = g->E, be = b + g->ns; b < be; b++) {
			if ((e = b->D.ef)) {
				hv_fwd(e);
				e = b->D.ee;
				e->aO = 0;
				e->adO = t2*g->g1;
				hv_back(e);
				}
			else if ((e = b->D.e)->op != f_OPNUM) {
				e->aO = 0;
				e->adO = t2*g->g1;
				}
			}
		if (g->g2) {
			t = 0.;
			for(og = g->og; og; og = og->next)
				t += og->coef * p[og->varno];
			t *= t2*g->g2;
			for(og = g->og; og; og = og->next)
				x0[og->varno].aO += t*og->coef;
			}
		}
	if (asl->P.ncom) {
	    for(c = cexps; c < ce--; ) {
		for(i0 = *--dvsp0; i0 < i1; ) {
			v = asl->P.vp[--i1];
			--c1;
			if ((t = v->aO) && (L = c1->L))
				for(Le = L + c1->nlin; L < Le; L++)
					((expr_v*)L->v.vp)->aO += t * L->fac;
			if (c1->hfun)
				funnel_back(asl, c1, v, t);
			else if ((e = c1->ee)) {
				e->aO = t;
				e->adO = v->adO;
				hv_back(e);
				}
			else if ((e = c1->e)->op != f_OPNUM) {
				e->aO = t;
				e->adO = v->adO;
				}
			}
		if ((t = (--x)->aO) && (L = ce->L))
			for(Le = L + ce->nlin; L < Le; L++)
				((expr_v*)L->v.vp)->aO += t * L->fac;
		if (ce->hfun)
			funnel_back(asl, ce, x, t);
		else if ((e = ce->ee)) {
			e->aO = t;
			e->adO = x->adO;
			hv_back(e);
			}
		else if ((e = ce->e)->op != f_OPNUM) {
			e->aO = t;
			e->adO = x->adO;
			}
		}
	    }
	x = var_e;
	for(la = asl->P.lalist; la; la = la->lnext)
		if ((t = la->v->aO)) {
			og = la->nz;
			do x[og->varno].aO += t*og->coef;
				while((og = og->next));
			}
	if ((cg = cg0)) {
		if (s) {
			while(cg) {
				i = cg->varno;
				hv[i] = s[i]*x0[i].aO;
				cg = cg->next;
				}
			}
		else {
			while(cg) {
				i = cg->varno;
				hv[i] = x0[i].aO;
				cg = cg->next;
				}
			}
		}
	else {
		og = og0;
		if (s) {
			while(og) {
				i = og->varno;
				hv[i] = s[i]*x0[i].aO;
				og = og->next;
				}
			}
		else {
			while(og) {
				i = og->varno;
				hv[i] = x0[i].aO;
				og = og->next;
				}
			}
		}
	if (p0)
		del_mblk(kp, p0);
	}

 varno_t
hvpcomps_ASL(ASL *a, real *hv, real *p, int co, varno_t nz, varno_t *z)
	/* p = direction */
	/* hv = result */
	/* co >= 0: behave like hvpcomp_ASL with nobj = -1, ow = 0, y[i] = i == co ? 1. : 0. */
	/* co < 0: behave like hvpcomp_ASL with nobj = -1 - co, ow = 0, y = 0 */
{
	ASL_pfgh *asl;
	cexp *c, *c1, *ce;
	cgrad *cg, *cg0;
	expr *e;
	expr_v *v, *x, *x0;
	int *dvsp0, i0, i1, kp, n, no, nx, oxk;
	linarg *la;
	linpart *L, *Le;
	ograd *og, *og0;
	ps_func *f;
	psb_elem *b, *be;
	psg_elem *g, *ge;
	real *hve, *p0, *s, t, t2, *vscale;
	varno_t F, i, rv, *ze;

#ifdef IGNORE_BOGUS_WARNINGS
	c1 = ce = 0;
	dvsp0 = 0;
	i1 = 0;
	v = 0;
	x = 0;
#endif
	ASL_CHECK(a, ASL_read_pfgh, "hvpcompi");
	asl = (ASL_pfgh*)a;
	if (x0kind == ASL_first_x) {
		if (!(s = X0))
			memset(s = Lastx, 0, n_var*sizeof(real));
		co_index = co;
		xp_check_ASL(asl, s);
		}
	nx = asl->i.nxval;
	oxk = asl->i.x_known;
	asl->i.x_known = 1;

	p0 = 0;
	cg0 = 0;
	og0 = 0;
	x0 = var_e;
	n = c_vars >= o_vars ? c_vars : o_vars;
	t2 = 1.;
	memset(hv, 0, n_var*sizeof(real));
	for(la = asl->P.lalist; la; la = la->lnext) {
		og = la->nz;
		t = p[og->varno]*og->coef;
		while((og = og->next))
			t += p[og->varno]*og->coef;
		x = la->v;
		x->dO.r = t;
		x->aO = x->adO = 0;
		}
	no = -1 - co;
	if (co >= 0) {
		if (co >= nlc)
			return 0;
		f = asl->P.cps + co;
		if (asl->i.ncxval[co] != nx)
			conpival_ASL(a, co, Lastx, 0);
		if (f->ng && f->nxval != nx)
			conpgrd_ASL(a, co, Lastx, 0, 0);
		if ((s = asl->i.lscale))
			t2 = s[co];
		cg = cg0 = Cgrad[co];
		if ((vscale = asl->i.vscale)) {
			kp = htcl(n*sizeof(real));
			p0 = (real*)new_mblk(kp);
			for(; cg; cg = cg->next) {
				i = cg->varno;
				x = x0 + i;
				x->dO.r = p0[i] = p[i]*vscale[i];
				x->aO = x->adO = 0.;
				}
			p = p0;
			}
		else {
			for(; cg; cg = cg->next) {
				i = cg->varno;
				x = x0 + i;
				x->dO.r = p[i];
				x->aO = x->adO = 0.;
				}
			}
		}
	else {
		if (no >= nlo)
			return 0;
		f = asl->P.ops + no;
		if (asl->i.ncxval[no] != nx)
			objpval_ASL(a, no, Lastx, 0);
		if (f->ng && f->nxval != nx)
			objpgrd_ASL(a, no, Lastx, 0, 0);
		og = og0 = Ograd[no];
		if ((vscale = asl->i.vscale)) {
			kp = htcl(n*sizeof(real));
			p0 = (real*)new_mblk(kp);
			for(; og; og = og->next) {
				i = og->varno;
				x = x0 + i;
				x->dO.r = p0[i] = p[i]*vscale[i];
				x->aO = x->adO = 0.;
				}
			p = p0;
			}
		else {
			for(; og; og = og->next) {
				i = og->varno;
				x = x0 + i;
				x->dO.r = p[i];
				x->aO = x->adO = 0.;
				}
			}
		}
	if (asl->i.Derrs) {
		asl->i.x_known = oxk;
		deriv_errchk_ASL(a, 0, co, 1);
		asl->i.x_known = 1;
		}
	if (asl->P.ncom) {
		x = var_ex;
		dvsp0 = asl->P.dvsp0;
		c = cexps;
		i0 = *dvsp0;
		for(ce = c1 = c + asl->P.ncom; c < ce; c++) {
			for(i1 = *++dvsp0; i0 < i1; i0++)
				hv_fwd0(asl, c1++, asl->P.vp[i0]);
			hv_fwd0(asl, c, x++);
			}
		}
	for(b = f->b, be = b + f->nb; b < be; b++) {
		if ((e = b->D.ef)) {
			hv_fwd(e);
			e = b->D.ee;
			e->aO = 0;
			e->adO = t2;
			hv_back(e);
			}
		else if ((e = b->D.e)->op != f_OPNUM) {
			e->aO = 0;
			e->adO = t2;
			}
		}
	for(g = f->g, ge = g + f->ng; g < ge; g++) {
		for(b = g->E, be = b + g->ns; b < be; b++) {
			if ((e = b->D.ef)) {
				hv_fwd(e);
				e = b->D.ee;
				e->aO = 0;
				e->adO = t2*g->g1;
				hv_back(e);
				}
			else if ((e = b->D.e)->op != f_OPNUM) {
				e->aO = 0;
				e->adO = t2*g->g1;
				}
			}
		if (g->g2) {
			t = 0.;
			for(og = g->og; og; og = og->next)
				t += og->coef * p[og->varno];
			t *= t2*g->g2;
			for(og = g->og; og; og = og->next)
				x0[og->varno].aO += t*og->coef;
			}
		}
	if (asl->P.ncom) {
	    for(c = cexps; c < ce--; ) {
		for(i0 = *--dvsp0; i0 < i1; ) {
			v = asl->P.vp[--i1];
			--c1;
			if ((t = v->aO) && (L = c1->L))
				for(Le = L + c1->nlin; L < Le; L++)
					((expr_v*)L->v.vp)->aO += t * L->fac;
			if (c1->hfun)
				funnel_back(asl, c1, v, t);
			else if ((e = c1->ee)) {
				e->aO = t;
				e->adO = v->adO;
				hv_back(e);
				}
			else if ((e = c1->e)->op != f_OPNUM) {
				e->aO = t;
				e->adO = v->adO;
				}
			}
		if ((t = (--x)->aO) && (L = ce->L))
			for(Le = L + ce->nlin; L < Le; L++)
				((expr_v*)L->v.vp)->aO += t * L->fac;
		if (ce->hfun)
			funnel_back(asl, ce, x, t);
		else if ((e = ce->ee)) {
			e->aO = t;
			e->adO = x->adO;
			hv_back(e);
			}
		else if ((e = ce->e)->op != f_OPNUM) {
			e->aO = t;
			e->adO = x->adO;
			}
		}
	    }
	x = var_e;
	for(la = asl->P.lalist; la; la = la->lnext)
		if ((t = la->v->aO)) {
			og = la->nz;
			do x[og->varno].aO += t*og->coef;
				while((og = og->next));
			}
	rv = 0;
	if ((ze = z))
		ze += nz;
	if ((hve = hv))
		hve += nz;
	F = Fortran;
	if ((cg = cg0)) {
		if (!hv) {
			while(cg) {
				++rv;
				if (z < ze)
					*z++ = cg->varno;
				cg = cg->next;
				}
			}
		else if (vscale) {
			while(cg) {
				++rv;
				i = cg->varno;
				if (z < ze)
					*z++ = F + i;
				if (hv < hve)
					*hv++ = vscale[i]*x0[i].aO;
				cg = cg->next;
				}
			}
		else {
			while(cg) {
				++rv;
				i = cg->varno;
				if (z < ze)
					*z++ = F + i;
				if (hv < hve)
					*hv++ = x0[i].aO;
				cg = cg->next;
				}
			}
		}
	else {
		og = Ograd[no];
		if (!hv) {
			while(og) {
				++rv;
				if (z < ze)
					*z++ = og->varno;
				og = og->next;
				}
			}
		else if (vscale) {
			while(og) {
				++rv;
				i = og->varno;
				if (z < ze)
					*z++ = F + i;
				if (hv < hve)
					*hv++ = vscale[i]*x0[i].aO;
				og = og->next;
				}
			}
		else {
			while(og) {
				++rv;
				i = og->varno;
				if (z < ze)
					*z++ = F + i;
				if (hv < hve)
					*hv++ = x0[i].aO;
				og = og->next;
				}
			}
		}
	if (p0)
		del_mblk(kp, p0);
	return rv;
	}

#ifdef __cplusplus
}
#endif
