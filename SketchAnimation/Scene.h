#ifndef _SCENE_H_
#define _SCENE_H_

#include "Utility.h"
#include "Entity.h"

#include <string>
#include <list>

class TiXmlElement;

//The Scene management class, this class has 
//responsibility to delete Entities in the scene 
class cScene : virtual public cFinal<cScene>{

public:

	cScene();
	cScene(const std::string &strSceneFile);

	~cScene();

//private:
//
//	cScene(const cScene&);
//	cScene& operator=(const cScene&);

public:

	void Add(cEntity*);
	void Delete(cEntity*);

	bool RsetScene(const std::string &strSceneFile);

private:

	void Clean();

	bool ParsePlane(const TiXmlElement*);

private:

	std::list<cEntity*> m_EntityList;
	typedef std::list<cEntity*>::iterator EntityItr;

	std::string m_strSceneFile;//the XML Config file path
}; 

#endif