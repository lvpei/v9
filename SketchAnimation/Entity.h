#ifndef _ENTITY_H_
#define _ENTITY_H_

class cEntity{

public:
	cEntity(){};
	virtual ~cEntity(){};

public:

	virtual bool Render() = 0;
	
};

#endif