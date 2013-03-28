/*

Copyright 2009 Topi Rinkinen  
email: gpl3 at domain topisoft.fi


This file is part of preisach modeling library.

    Preisach modeling library is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Preisach modeling library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Preisach modeling library.  If not, see <http://www.gnu.org/licenses/>.
*/
 
/*
	File version: 0.1
	File: preisach_integral.h
	
	History:
	
	2009-11-29	First test version 0.1

*/

#ifndef PREISACH_INTEGRAL_H_
#define PREISACH_INTEGRAL_H_

#include "preisach.h"

//Given an Everett function (eg, density function over the Preisach model)
//Compute the model output given an input
//This is effectively the area of the zig-zag triangle weighted by the Everett values
class preisach_everett
{
	public:
	void *ev_pri;
	
	double (&Everett)(void *pri, double const &x, double const &y);
	preisach_everett(double (&p_ev)(void *pri, double const &x, double const &y), void *pri):Everett(p_ev)
	{
		ev_pri=pri;
	}
	
	double calc(preisach_triangle const &p)
	{
		// See lib.tkk.fi/Diss/2008/isbn9789512292776/article1.pdf
		// page 101 for reference for the algorithm.
		double r=0;
		unsigned int i,n;
		double ak,bk,bkm1;	// alpha_k, beta_k, beta_(k-1)
		
		n=p.zig_zag.size();
		
		ak=p.zig_zag[0];
		if(n==1)
			bk=ak;
		else
			bk=p.zig_zag[1];
		bkm1=p.xmin;
		r=Everett(ev_pri,ak,bkm1)-Everett(ev_pri,ak,bk);
		
		for(i=2;i<n;i+=2)
		{
			ak=p.zig_zag[i];
			bkm1=bk;
			if(i+1==n)	// Last was increasing, zigzag ends to horizontal.
				bk=ak;
			else	// Last was decreasing, zigzag ends to vertical.
				bk=p.zig_zag[i+1];
			
			r+=Everett(ev_pri,ak,bkm1)-Everett(ev_pri,ak,bk);
		}
		r=r*2-Everett(ev_pri,0,0);
		
		return r;
	}
}; 

#endif /*PREISACH_INTEGRAL_H_*/
