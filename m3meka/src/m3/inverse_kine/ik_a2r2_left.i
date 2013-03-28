%module ik_a2r2_left
%{
	#include "ik_a2r2_left_for_swig.h"
%}
		
	%include "typemaps.i"
	%include "std_vector.i"

	namespace std
	{
		%template(DoubleVector) vector<double>;
		%template(IntVector) vector<int>;
	}

	
			
	class IKSolver {
		public:
			bool Solve(const std::vector<double> veetrans, const std::vector<double> veerot, double free);
			int GetNumSolutions();
			std::vector<double> GetSolution(int i, std::vector<double> free_angles);
			std::vector<int> GetFree(int i);	
	};
