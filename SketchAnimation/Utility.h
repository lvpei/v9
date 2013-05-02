#ifndef _UTILITY_H_
#define _UTILITY_H_

#include <cassert>

//this template class is used to protect the derived class from being inherited and instancing more than once
template<class T>
class cFinal{

	friend T;//kao! don't do friend class T
private:

	cFinal(){
		if(ms_nInstanceCount == 0){
			++ms_nInstanceCount;
		}
		else{
			assert(0x0 && "More Instance of the Class with Sinleton Pattern will be created!");
		}
	}

	cFinal(const cFinal&);
	cFinal& operator = (const cFinal&);

private:

	static int ms_nInstanceCount;

};


#endif