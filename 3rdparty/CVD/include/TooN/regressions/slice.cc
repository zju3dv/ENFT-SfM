#include "regressions/regression.h"

int main()
{
	Vector<5> v = makeVector(1, 2, 3, 4, 5);
	cout << v.slice<0,3>() << endl;
}
