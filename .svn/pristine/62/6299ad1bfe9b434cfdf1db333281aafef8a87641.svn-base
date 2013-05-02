#include "Scene.h"

#include "XMLDefinition.h"

#include "TinyXML/tinyxml.h"
#include "TinyXML/tinystr.h"

#include <boost\shared_ptr.hpp>

using namespace std;
using namespace std::tr1;

int cFinal<cScene>::ms_nInstanceCount = 0;

cScene::cScene()
	   :cFinal<cScene>()
{

}

cScene::cScene(const std::string &strSceneFile)
	   :cFinal<cScene>(), m_strSceneFile(strSceneFile)
{
	bool bSuccess = RsetScene(strSceneFile);
	assert(bSuccess && "cScene::cScene(strSceneFile) -- Invalid Config File");
}

void cScene::Clean()
{
	for(EntityItr itr = m_EntityList.begin() ; itr != m_EntityList.end() ; ++itr)
	{
		if(*itr){
			delete *itr;
		}
	}
	m_EntityList.clear();
}

void cScene::Add(cEntity* pEnt)
{
	assert(pEnt);

	m_EntityList.push_back(pEnt);

}

void cScene::Delete(cEntity* pEnt)
{
	assert(pEnt);

	for(EntityItr itr = m_EntityList.begin(); itr != m_EntityList.end(); ++itr)
	{
		if(*itr == pEnt){
			m_EntityList.erase(itr);
			return;
		}
	}

	return;

}

bool cScene::RsetScene(const string &strSceneFile)
{
	shared_ptr<TiXmlDocument> pDocument(new TiXmlDocument(strSceneFile.c_str()));
	pDocument->LoadFile();
	
	TiXmlElement *pRootElement = pDocument->RootElement();
	if(strcmp(pRootElement->Value(),XML_ROOT)){
		return false;
	}

	for(TiXmlElement *pElement = pRootElement->FirstChildElement(); pElement ; pElement = pElement->NextSiblingElement())
	{
		if(!strcmp(pElement->Value(),XML_PLANE_ROOT)){
			if(!ParsePlane(pElement)){
				return false;
			}
		}
	}

	return true;
}

bool cScene::ParsePlane(const TiXmlElement* pElement)
{
	return true;
}

