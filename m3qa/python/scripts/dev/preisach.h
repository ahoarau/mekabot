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
	File: preisach.h
	
	History:
	
	2009-11-29	First test version 0.1

*/
 
#ifndef PREISACH_H_
#define PREISACH_H_

#include <vector>

using namespace std;

// Preisach hysteresis modeller, the triangle.
// See http://tinyurl.com/preisach for an applet describing
// the triangle behaviour.

//// zig_zag vector holds input values
// x_table vector holds x-coordinates.
// y_table vector holds y-coordinates.
// First entry (y0) is y coordinate of the first corner: (0,y0).
// Next entry (x1) is x coordinate of next corner: (x1,y0).
// Next entry (y2) is y coordinate of next corner: (x1,y2).
// etc...
// Every y-coordinate is in descending order.
// Every x-coordinate is in ascending order.



// Example cases:
// Coordinates are listed as y0,x1,y2,x3,....
// xmin=0, xmax=20

// Case 1:
// Status: 10,2,8,3,6
// Last direction was increasing.
// New input: 7
// Result: 10,2,8,3,7

// Case2:
// Status: 10,2,8,3,6
// Last direction was increasing.
// New input: 8
// Result: 10,2,8

// Case 3:
// Status: 10,2,8,3,6
// Last direction was increasing.
// New input: 9
// Result: 10,2,9

// Case 4:
// Status: 10,2,8,3,6
// Last direction was increasing.
// New input: 10
// Result: 10

// Case 5:
// Status: 10,2,8,3,6
// Last direction was increasing.
// New input: 11
// Result: 11

// Case 6:
// Status: 10
// Last direction was increasing.
// New input: 11
// Result: 11

// Case 7:
// Status: 10,2,8,3,6
// Last direction was increasing.
// New input: 5
// Result: 10,2,8,3,6,5

// Case 8:
// Status: 10,2,8,3,6
// Last direction was increasing.
// New input: 3
// Result: 10,2,8,3

// Case 9:
// Status: 10,2,8,3,6
// Last direction was increasing.
// New input: 2.5
// Result: 10,2,8,2.5

// Case 10:
// Status: 10,2,8,3,6
// Last direction was increasing.
// New input: 2
// Result: 10,2

// Case 11:
// Status: 10,2,8,3,6
// Last direction was increasing.
// New input: 1
// Result: 10,1

// Case 12:
// Status: 10,2,8,3,6
// Last direction was increasing.
// New input: 0
// Result: 0

// Case 13:
// Status: 10,2,8,3
// Last direction was decreasing.
// New input: 4
// Result: 10,2,8,3,4

// Case 14:
// Status: 10,2,8,3
// Last direction was decreasing.
// New input: 8
// Result: 10,2,8

// Case 15:
// Status: 10,2,8,3
// Last direction was decreasing.
// New input: 9
// Result: 10,2,9

// Case 16:
// Status: 10,2,8,3
// Last direction was decreasing.
// New input: 10
// Result: 10

// Case 17:
// Status: 10,2,8,3
// Last direction was decreasing.
// New input: 11
// Result: 11

// Case 18:
// Status: 10,2,8,3
// Last direction was decreasing.
// New input: 2.5
// Result: 10,2,8,2.5

// Case 19:
// Status: 10,2,8,3
// Last direction was decreasing.
// New input: 2
// Result: 10,2

// Case 19:
// Status: 10,2,8,3
// Last direction was decreasing.
// New input: 1
// Result: 10,1

// Case 20:
// Status: 10,2,8,3
// Last direction was decreasing.
// New input: 0
// Result: 0


// Needs more cases !!

//This class represents the preisach boundary as a zig-zag
//A series of piecewise linear vertical/horizontal line segments
//Only direction reversals affect the zig-zag
//No discretization is done

//u : input variable
//v: output variable
//a: upper bound (alpha)
//b: lower bound (beta)
//a>b by definition
//Preisach hysteron (gamma): v=g(a,b,u)=+1 if u>a, -1 if u<b, unchanged if b<=u<=a
//mu: density function mu(b,a) 
//mu(b,a)=0 if b<bo or a>ao (outer bounds of Preisach plane)
//w(t): Preisach operator (omega): integral over plane = sum(g(a,b)==1)-sum(g(a,b)==-1)
class preisach_triangle
{
public:
		// 1st entry is y coordinate of 1st corner
		// 2nd entry is x coordinate of 2nd corner
		// 3rd entry is y coordinate of 3rd corner
		// etc.
		// As additions and deletions are only done to the end
		// of container, vector is ideal container model
		// for storing corner coordinates.
	vector<double> zig_zag;
	//vector<double> x_table,y_table;
	
	double xmin,xmax;
	//double prev_input;	// The value of previous input value.
	int last_dir;		// -1 decending, +1 ascending
	//int num_entries;	// Number of x and y entries together.
	
	int dump(void)
	{
		unsigned int i;
		printf("zig_zag:\t");
		for(i=0;i<zig_zag.size();i++)
		{
			if(i)
				printf(", ");
			printf("%f",zig_zag[i]);
		}
		printf("\n");
		return 0;
	}
	
	preisach_triangle(double p_min,double p_max)
	{
		xmin=p_min;
		xmax=p_max;
		//prev_input=xmin;
		last_dir=-1;
		zig_zag.push_back(xmin);
		//num_entries=1;
	}
	
	
	int input(double const &x)
	{
		int i;
		double prev_input;
		
		prev_input = *(zig_zag.rbegin());
		
		if(x==prev_input)
			return 0;
		if(x>prev_input)	// Increasing direction.
		{
			//if(last_dir==-1)	// Changed direction.
			if(x >= xmax)
			{
				zig_zag.erase(zig_zag.begin()+1,zig_zag.end());
				zig_zag[0]=xmax;
			}
			else if(zig_zag.size()%2 == 0) // Last coord was Y. Changed direction to +1. Previous was decreasing.
			{
				// zig_zag.size() >= 2.
				i=zig_zag.size()-2;
				if(i>=0)	// Debugging test, can be removed later.
				{
					for(;i>=-2; i-=2)
					{
						if(i==-2 || x<zig_zag[i])
						{
							i+=2;
							zig_zag.erase(zig_zag.begin()+i,zig_zag.end());
							zig_zag.push_back(x);
							break;
						}
					}
				}
				else
				{
					printf("Error, should not happen!\n\n");
					//exit(-1);
				}
			}
			else	// No change in direction, still increasing.
			{
				// zig_zag.size() >= 1.
				i=zig_zag.size()-3;		// No need to analyze -1.
				if(i>=0)
				{
					for(;i>=-2; i-=2)
					{
						if(i==-2 || zig_zag[i]>x)	// Iterator might be faster than indexing.
						{
							i+=2;
							zig_zag.erase(zig_zag.begin()+i+1,zig_zag.end());
							zig_zag[i]=x;
							break;
						}
					}
				}
				else
				{
					zig_zag[0]=x;
				}
			}
		}
		if(x<prev_input)	// Decreasing direction.
		{
			if(x <= xmin)
			{
				zig_zag.erase(zig_zag.begin()+1,zig_zag.end());
				zig_zag[0]=xmin;
			}
			else if(zig_zag.size()%2 == 1) // Changed direction to -1. Previous was increasing.
			{
				// zig_zag.size() >= 1.
				i=zig_zag.size()-2;
				if(i>=0)
				{
					for(;i>=-1; i-=2)
					{
						//double dz=zig_zag[i];
						if(i==-1 || x>zig_zag[i])
						{
							i+=2;
							zig_zag.erase(zig_zag.begin()+i,zig_zag.end());
							zig_zag.push_back(x);
							break;
						}
					}
				}
				else
				{
					zig_zag.push_back(x);
				}
			}
			else	// No change in direction, still decreasing.
			{
				// zig_zag.size() >= 2.
				i=zig_zag.size()-3;		// No need to analyze -1.
				if(i>=0)
				{
					// zig_zag.size() >= 3.
					for(;i>=-1; i-=2)
					{
						//double dz=zig_zag[i];
						if(i==-1 || x>zig_zag[i])	// Iterator might be faster than indexing.
						{
							i+=2;
							zig_zag.erase(zig_zag.begin()+i+1,zig_zag.end());
							zig_zag[i]=x;
							break;
						}
					}
				}
				else
				{
					// zig_zag.size() = 2.
					zig_zag[1]=x;
				}
			}
		}
		prev_input=x;
		return 0;
	}
};

#endif /*PREISACH_H_*/
