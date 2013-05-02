/*
 *  shape.cpp
 *  
 *
 *
 */

#include "shape.h"
#include <math.h>

Shape::Shape()
{		

}

Shape::Shape(const std::string& filename)
{
    LoadOBJ(filename);
}

void Shape::draw(QSharedPointer< QGLShaderProgram > shader)
{
	
	int position = shader->attributeLocation("positionIn") ;
	shader->enableAttributeArray(position);
	shader->setAttributeArray(position, &vertexBuffer[0],sizeof(QVector3D)) ;
	
	int normal = shader->attributeLocation("normalIn") ;
	shader->enableAttributeArray(normal) ;
	shader->setAttributeArray(normal, &normalBuffer[0], sizeof(QVector3D)) ;
	
	if(texcoordBuffer.size() > 0)
	{
		int texcoord = shader->attributeLocation("texcoordIn") ;
		shader->enableAttributeArray(texcoord) ;
		shader->setAttributeArray(texcoord, &texcoordBuffer[0], sizeof(QVector2D)) ;
	}
	
	glDrawElements(GL_TRIANGLES, indexBuffer.size(), GL_UNSIGNED_INT, &(indexBuffer[0])) ;
	
	shader->disableAttributeArray(position);
	shader->disableAttributeArray(normal);
	if(texcoordBuffer.size() > 0)
	{shader->disableAttributeArray(shader->attributeLocation("texcoordIn"));}
	
}
/*
void Shape::draw(QSharedPointer< QGLShaderProgram > shader)
{	
	if (shader_.isNull())
		shader_ = QSharedPointer< QGLShaderProgram >( new QGLShaderProgram );
	if (!shader_->isLinked()){
		shader_->addShaderFromSourceFile( QGLShader::Vertex  , "shaders/phong.vert" );
		shader_->addShaderFromSourceFile( QGLShader::Fragment, "shaders/phong.frag" );
		if (!shader_->link())
			qFatal("Shader linking failed!");
	}	
	render(shader_);
}
*/



// Compute or re-compute the per-vertex normals for this shape
// using the normals of adjoining faces.
void Shape::generateNormals()
{
    //
    // We compute the normals for hte mesh as the area-weighted average of
    // the normals of incident faces. This is a simple technique to implement,
    // but other techniques are possible.
    //
    
    //
    // Initialize all the normals to zero.
    //
    size_t numVertices = vertexBuffer.size();
    for (size_t ii = 0; ii < numVertices; ++ii) {
        normalBuffer.push_back(QVector3D(0,0,0));
    }
    
    //
    // Loop over faces, adding the normal of each face
    // to the vertices that use it.
    //
    size_t numFaces = indexBuffer.size() / 3;
    for (size_t ii = 0; ii < numFaces; ++ii) {
        
        QVector3D v0 = vertexBuffer[indexBuffer[3*ii]];
        QVector3D v1 = vertexBuffer[indexBuffer[3*ii+1]];
        QVector3D v2 = vertexBuffer[indexBuffer[3*ii+2]];
        
        //
        // We compute a cross-product of two triangle edges.
        // This direction of this vector will be the normal
        // direction, while the magnitude will be twice
        // the triangle area. We can thus use this directly
        // as a weighted normal.
        //
        QVector3D edge1 = v1 - v0;
        QVector3D edge2 = v2 - v0;
        QVector3D weightedNormal = QVector3D::crossProduct(edge1, edge2);
        
        //
        // We now add the face normal to the normal stored
        // in each of the vertices using the face.
        //
        normalBuffer[indexBuffer[3*ii]] += weightedNormal;
        normalBuffer[indexBuffer[3*ii+1]] += weightedNormal;
        normalBuffer[indexBuffer[3*ii+2]] += weightedNormal;
    }
    
    //
    // Each vertex now has an area-weighted normal. We need to
    // normalize them to turn them into correct unit-lenght
    // normal vectors for rendering.
    //
     for (unsigned int k = 0; k < normalBuffer.size(); k++) {
     if (normalBuffer[k].lengthSquared() != 0)
     {normalBuffer[k].normalize();}
     }

}

// Helper function to deal with the particular way
// that indices must be interpreted in an .obj file
static int OBJIndexing(int input, size_t numValues)
{
    if (input > 0) return input - 1;
    return numValues + input;
}


// Load the shape with vertices and faces from the OBJ file.
// Returns ST_ERROR on failure, ST_OK on success.
void Shape::LoadOBJ(const std::string& filename)
{
    static const int kMaxLine = 256;
    
    // The subset of the OBJ format that we handle has
    // the following commands:
    //
    // v    <x> <y> <z>         Define a vertex position.
    // vn   <x> <y> <z>         Define a vertex normal.
    // vt   <s> <t>             Define a vertex texture coordiante.
    // f    <p0>/<t0>/<n0> ...  Define a face from previous data.
    //
    // Every face in an OBJ file refers to previously-defines
    // positions, normals and texture coordinates by index.
    // Vertices in an STShape must define all three of these,
    // so we must generate one STShape vertex for each combination
    // of indices we see in the OBJ file.
    //
    
    //
    // Open the file.
    //
    FILE* file = fopen(filename.c_str(), "r");
    if (!file) {
        fprintf(stderr,
                "Shape::LoadOBJ() - Could not open shape file '%s'.\n", filename.c_str());
    }
    
    char lineBuffer[kMaxLine];
    int lineIndex = 0; // for printing error messages
    
    // Arrays to collect the positions, normals and texture
    // coordinates that we encounter.
    std::vector<QVector3D> positions;
    std::vector<QVector2D> texCoords;
    std::vector<QVector3D> normals;
    
    // Map to point us to previously-created vertices. This maps
    // triples of indices (a position, texcoord and normal index)
    // to a single index in the new shape.
    typedef std::pair<int,int> IntPair;
    typedef std::pair<int, IntPair> IntTriple;
    typedef std::map<IntTriple, size_t> IndexMap;
    IndexMap indexMap;
    
    // Keep track of whether the file contained normals...
    bool needsNormals = false;
    
    //
    // Read the file line-by-line
    //
    while (fgets(lineBuffer, kMaxLine, file)) {
        ++lineIndex;
        char* str = strtok(lineBuffer, " \t\n\r");
        
        //
        // Skip empty or comment lines.
        //
        if (!str || str[0] == '\0' || str[0] == '#')
            continue;
        if (str[0] == 'g' || str[0] == 's' || strcmp(str, "usemtl") == 0 || strcmp(str, "mtllib") == 0)
            continue;
        //
        // Process other lines based on their commands.
        //
        if (strcmp(str, "v") == 0) {
            // Vertex position line. Read the position data (x, y, z).
            str = strtok(NULL, "");
            float position_x, position_y, position_z;
            sscanf(str, "%f %f %f\n", &position_x, &position_y, &position_z);
            positions.push_back(QVector3D(position_x,position_y,position_z));
        }
        else if (strcmp(str, "vt") == 0) {
            // Vertex texture coordinate line. Read the texture coord data.
            str = strtok(NULL, "");
            float texCoord_x;
            float texCoord_y;
            sscanf(str, "%f %f\n", &texCoord_x, &texCoord_y);
            texCoords.push_back(QVector2D(texCoord_x,texCoord_y));
        }
        else if (strcmp(str, "vn") == 0) {
            // Vertex normal line. Read the normal data.
            str = strtok(NULL, "");
            float normal_x,normal_y,normal_z;
            sscanf(str, "%f %f %f\n", &normal_x, &normal_y, &normal_z);
            normals.push_back(QVector3D(normal_x,normal_y,normal_z));
        }
        else if (strcmp(str, "f") == 0) {
            // Face command. Each vertex in the face will be defined by
            // the indices of its position, texture coordinate and
            // normal.
            std::vector<Index> faceIndices;
            
            // Read each vertex entry.
            int curIndex = 0;
            Index indices[3];
            enum FaceFormat {
                PosTexNorm, // %d/%d/%d
                PosTex,     // %d/%d
                PosNorm,    // %d//%d
                Pos,         // %d
                LAST_FACE_FORMAT
            };
            
            const char* FaceFormatToString[LAST_FACE_FORMAT] = {
                "Position/Texture/Normal",
                "Position/Texture",
                "Position//Normal",
                "Position",
            };
            
            bool set_format = false;
            FaceFormat format;
            
            const int kNoTextureIndex = -1;
            const int kNoNormalIndex = -1;
            
            int positionIdx;
            int texCoordIdx;
            int normalIdx;
            
            while ((str = strtok(NULL, " \t\n\r")) != NULL) {
                
                if (sscanf(str, "%d/%d/%d", &positionIdx, &texCoordIdx, &normalIdx) == 3) {
                    if (!set_format) {
                        format = PosTexNorm;
                        set_format = true;
                    } else {
                        if (format != PosTexNorm) {
                            fprintf(stderr, "STShape::LoadOBJ() - "
                                    "Line %d: Current face format is %s, but received another vertex in format %s\n",
                                    lineIndex, FaceFormatToString[format], FaceFormatToString[PosTexNorm]);
                        }
                    }
                } else if (sscanf(str, "%d/%d", &positionIdx, &texCoordIdx) == 2) {
                    if (!set_format) {
                        format = PosTex;
                        set_format = true;
                    } else {
                        if (format != PosTex) {
                            fprintf(stderr, "STShape::LoadOBJ() - "
                                    "Line %d: Current face format is %s, but received another vertex in format %s\n",
                                    lineIndex, FaceFormatToString[format], FaceFormatToString[PosTex]);
                        }
                    }
                } else if (sscanf(str, "%d//%d", &positionIdx, &normalIdx) == 2) {
                    if (!set_format) {
                        format = PosNorm;
                        set_format = true;
                    } else {
                        if (format != PosNorm) {
                            fprintf(stderr, "STShape::LoadOBJ() - "
                                    "Line %d: Current face format is %s, but received another vertex in format %s\n",
                                    lineIndex, FaceFormatToString[format], FaceFormatToString[PosNorm]);
                        }
                    }
                    // Pass
                } else if (sscanf(str, "%d", &positionIdx) == 1) {
                    if (!set_format) {
                        format = Pos;
                        set_format = true;
                    } else {
                        if (format != Pos) {
                            fprintf(stderr, "STShape::LoadOBJ() - "
                                    "Line %d: Current face format is %s, but received another vertex in format %s\n",
                                    lineIndex, FaceFormatToString[format], FaceFormatToString[Pos]);
                        }
                    }
                } else {
                    // TODO(boulos): Print out line #?
                    fprintf(stderr, "STShape::LoadOBJ() - "
                            "Line %d: Bad face format given %s\n", lineIndex, str);
                    continue;
                }
                
                //
                // We look to see if we have already created a vertex
                // based on this position/texCoord/normal, and reuse it
                // if possible. Otherwise we add a new vertex.
                //
                positionIdx = OBJIndexing(positionIdx, positions.size());
                texCoordIdx = (format == PosTexNorm ||
                               format == PosTex)
                ? OBJIndexing(texCoordIdx,
                              texCoords.size())
                : kNoTextureIndex;
                normalIdx   = (format == PosTexNorm ||
                               format == PosNorm)
                ? OBJIndexing(normalIdx,
                              texCoords.size())
                : kNoNormalIndex;
                size_t newIndex;
                
                IntTriple key(positionIdx, IntPair(texCoordIdx, normalIdx));
                IndexMap::const_iterator ii = indexMap.find(key);
                if (ii != indexMap.end()) {
                    newIndex = ii->second;
                }
                else {
                    // Construct a new vertex from the indices given.
                    QVector3D position = positions[positionIdx];
                    QVector3D normal = (normalIdx != kNoNormalIndex) ? normals[normalIdx] : QVector3D(0,0,0);
                    QVector2D texCoord = (texCoordIdx != kNoTextureIndex) ? texCoords[texCoordIdx] : QVector2D(0,0);
                    
                    // If the vertex has no normal, then remember
                    // to create normals later...
                    if (normalIdx == kNoNormalIndex) {
                        needsNormals = true;
                    }
                    
                   // printf("%f %f %f\n",position.x(),position.y(),position.z());
                    
                    vertexBuffer.push_back(position);
                    newIndex = vertexBuffer.size() - 1;
                    //Vertex newVertex(position, normal, texCoord);
                    //newIndex = AddVertex(newVertex);
                    indexMap[key] = newIndex;
                }
                
                indices[curIndex++] = newIndex;
                // Keep triangle fanning
                if (curIndex == 3) {
                    //AddFace(Face(indices[0], indices[1], indices[2]));
                    indexBuffer.push_back(indices[0]);
                    indexBuffer.push_back(indices[1]);
                    indexBuffer.push_back(indices[2]);
                    indices[1] = indices[2];
                    curIndex = 2;
                }
            }
        }
        else {
            //
            // Unknown line - ignore and print a warning.
            //
            fprintf(stderr, "Shape::LoadOBJ() - "
                    "Unable to parse line %d: %s (continuing)\n",
                    lineIndex, lineBuffer);
        }
    }
    fclose(file);
    
    //
    // If the file didn't already have normals, then generate them.
    //
    generateNormals();
    
}

namespace Shapes
{
	Shape* CreateCylinder( float radius, float height,
						   unsigned int numSlices, unsigned int numStacks )
	{
		Shape* newShape = new Shape();

		for(unsigned int ii = 0; ii <= numStacks; ++ii) {
			for(unsigned int jj = 0; jj < numSlices; ++jj) {
				const float theta =
				float(jj) * 2.0f * float(M_PI) / float(numSlices);
				
				QVector3D position(radius * cosf(theta),radius * sinf(theta),height * (float(ii) / numStacks));
	
				newShape->vertexBuffer.push_back(position);
				newShape->normalBuffer.push_back(QVector3D(cosf(theta),sinf(theta),0));
				
			}
		}
		
		// Add faces
		
		for(unsigned int ii = 0; ii < numStacks; ii++) {
			for(unsigned int jj = 0; jj < numSlices; jj++) {
				int jjPlus1 = (jj + 1) % numSlices;
				
				// First triangle
				
				newShape->indexBuffer.push_back((ii + 1)*numSlices + jj);
				newShape->indexBuffer.push_back((ii + 0)*numSlices + jj);
				newShape->indexBuffer.push_back((ii + 0)*numSlices + jjPlus1);
				
				QVector3D v0 = newShape->vertexBuffer[(ii + 1)*numSlices + jj];
				QVector3D v1 = newShape->vertexBuffer[(ii + 0)*numSlices + jj];
				QVector3D v2 = newShape->vertexBuffer[(ii + 0)*numSlices + jjPlus1];


				// Second triangle
				newShape->indexBuffer.push_back((ii + 1)*numSlices + jj);
				newShape->indexBuffer.push_back((ii + 0)*numSlices + jjPlus1);
				newShape->indexBuffer.push_back((ii + 1)*numSlices + jjPlus1);
				
				QVector3D v3 = newShape->vertexBuffer[(ii + 1)*numSlices + jjPlus1];
				

			}
		}
        
        
		/*
		for (unsigned int k = 0; k < newShape->normalBuffer.size(); k++) {
			if (newShape->normalBuffer[k].lengthSquared() != 0)
			{newShape->normalBuffer[k].normalize();}
		}
         */
		
		return newShape;	
	}
	
    
    Shape* CreateClosedCylinder( float radius, float height,
                          unsigned int numSlices, unsigned int numStacks )
    {
        Shape* newShape = CreateCylinder(radius,height,numSlices,numStacks);
        
        // Cap the bottom
        
        newShape->vertexBuffer.push_back(QVector3D(0,0,0));
        newShape->normalBuffer.push_back(QVector3D(0,0,-1));
        int cap_center = newShape->vertexBuffer.size()-1;
        
        for(unsigned int jj = 0; jj < numSlices; ++jj) {
            int jjPlus1 = (jj + 1) % numSlices;
            newShape->indexBuffer.push_back(cap_center);
            newShape->indexBuffer.push_back( jj );
            newShape->indexBuffer.push_back( jjPlus1 );
            
        }

        // Cap the top
        
        newShape->vertexBuffer.push_back(QVector3D(0,0,height));
        newShape->normalBuffer.push_back(QVector3D(0,0,1));
        int cap_top_center = newShape->vertexBuffer.size()-1;
        
        for(unsigned int jj = 0; jj < numSlices; ++jj) {
            int jjPlus1 = (jj + 1) % numSlices;
            newShape->indexBuffer.push_back(cap_top_center);
            newShape->indexBuffer.push_back( numStacks*numSlices + jj );
            newShape->indexBuffer.push_back( numStacks*numSlices + jjPlus1 );
            
        }
        
        return newShape;
        
    }
	
	Shape* CreateSphere(
						  float radius, const QVector3D center,
						  unsigned int numSlices, unsigned int numStacks)
    {
        Shape* newShape = new Shape();
		
        float PI_Stacks = float(M_PI) / float(numStacks);
        float PI2_Slices = 2.0f * float(M_PI) / float(numSlices);
		
        //
        // Add the interior vertices
        //
        for (unsigned int ii = 1; ii < numStacks; ++ii) {
            float phi = float(ii) * PI_Stacks;
            float cosPhi = cosf(phi);
            float sinPhi = sinf(phi);
			
            for(unsigned int jj = 0; jj < numSlices; ++jj) {
                float theta = float(jj) * PI2_Slices;
				QVector3D direction = QVector3D(radius * cosf(theta) * sinPhi,
												 radius * sinf(theta) * sinPhi,
											  radius * cosPhi);
                QVector3D position = center + direction;
                newShape->vertexBuffer.push_back(position);
				direction.normalize();
				newShape->normalBuffer.push_back(direction);
            }
        }
		
        //
        // Add the pole vertices
        //

		newShape->vertexBuffer.push_back(center + QVector3D(0,0,radius));
		newShape->normalBuffer.push_back(QVector3D(0,0,1));
		unsigned int topVertexIndex = newShape->vertexBuffer.size() - 1;
		
        newShape->vertexBuffer.push_back(center + QVector3D(0,0,-radius));
		newShape->normalBuffer.push_back(QVector3D(0,0,-1));
		unsigned int bottomVertexIndex = newShape->vertexBuffer.size() - 1;
		
        //
        // Add the top and bottom triangles (all triangles involving the pole vertices)
        //
        for( unsigned int ii = 0; ii < numSlices; ii++) {
            unsigned int iiPlus1 = (ii + 1) % numSlices;
			
			newShape->indexBuffer.push_back(ii);
			newShape->indexBuffer.push_back(iiPlus1);
			newShape->indexBuffer.push_back(topVertexIndex);
			
			newShape->indexBuffer.push_back(iiPlus1 + (numStacks - 2) * numSlices);
			newShape->indexBuffer.push_back(ii + (numStacks - 2) * numSlices);
			newShape->indexBuffer.push_back(bottomVertexIndex);
			
        }
		
        //
        // Add all the interior triangles
        //
        for(unsigned int ii = 0; ii < numStacks - 2; ++ii) {
            for(unsigned int jj = 0; jj < numSlices; ++jj) {
                unsigned int jjPlus1 = (jj + 1) % numSlices;
				
				newShape->indexBuffer.push_back((ii + 1)*numSlices + jj);
				newShape->indexBuffer.push_back((ii + 0)*numSlices + jjPlus1);
				newShape->indexBuffer.push_back((ii + 0)*numSlices + jj);
				
				newShape->indexBuffer.push_back((ii + 1)*numSlices + jj);
				newShape->indexBuffer.push_back((ii + 1)*numSlices + jjPlus1);
				newShape->indexBuffer.push_back((ii + 0)*numSlices + jjPlus1);
            }
        }
		
		
		for (unsigned int k = 0; k < newShape->normalBuffer.size(); k++) {
			if (newShape->normalBuffer[k].lengthSquared() != 0)
			{newShape->normalBuffer[k].normalize();}
		}
		
        return newShape;
    }
    
    void addQuad(std::vector<unsigned int>* indexBuffer,int a,int b,int c,int d){
		indexBuffer->push_back(a);
		indexBuffer->push_back(b);
		indexBuffer->push_back(c);
		indexBuffer->push_back(a);
		indexBuffer->push_back(c);
		indexBuffer->push_back(d);
	}
	
	
	Shape* CreateFloor( float size, int subdiv )
    {
        Shape* newShape = new Shape();
		
        float floorSize = size / float(subdiv);
        int number = 0;
        
        for (int i = -subdiv; i <= subdiv; i++)
        {
            for (int j = -subdiv; j <= subdiv; j++)
            {
                newShape->vertexBuffer.push_back(QVector3D(i*floorSize + floorSize, 0, j*floorSize +  floorSize));
                newShape->texcoordBuffer.push_back(QVector2D(i*floorSize + floorSize, j*floorSize + floorSize));
                
                newShape->vertexBuffer.push_back(QVector3D(i*floorSize -floorSize, 0, j*floorSize +  floorSize));
                newShape->texcoordBuffer.push_back(QVector2D(i*floorSize -floorSize,j*floorSize + floorSize));
                
                newShape->vertexBuffer.push_back(QVector3D(i*floorSize + floorSize, 0, j*floorSize  -floorSize));
                newShape->texcoordBuffer.push_back(QVector2D(i*floorSize + floorSize,j*floorSize -floorSize));
                
                newShape->vertexBuffer.push_back(QVector3D(i*floorSize -floorSize, 0, j*floorSize  -floorSize));
                newShape->texcoordBuffer.push_back(QVector2D(i*floorSize -floorSize,j*floorSize -floorSize));
                
                for(int i = 0; i < 4; i++)
                {newShape->normalBuffer.push_back(QVector3D(0,1,0));}
                
                newShape->indexBuffer.push_back(number);
                newShape->indexBuffer.push_back(number+1);
                newShape->indexBuffer.push_back(number+2);
                
                newShape->indexBuffer.push_back(number+1);
                newShape->indexBuffer.push_back(number+3);
                newShape->indexBuffer.push_back(number+2);
                
                number+=4;
            }
        }
        
        /*
		newShape->vertexBuffer.push_back(QVector3D(floorSize, 0,  floorSize));
		newShape->texcoordBuffer.push_back(QVector2D(floorSize,floorSize));
		
		newShape->vertexBuffer.push_back(QVector3D(-floorSize, 0,  floorSize));
		newShape->texcoordBuffer.push_back(QVector2D(-floorSize,floorSize));
		
		newShape->vertexBuffer.push_back(QVector3D(floorSize, 0,  -floorSize));
		newShape->texcoordBuffer.push_back(QVector2D(floorSize,-floorSize));
		
		newShape->vertexBuffer.push_back(QVector3D(-floorSize, 0,  -floorSize));
		newShape->texcoordBuffer.push_back(QVector2D(-floorSize,-floorSize));
		
		for(int i = 0; i < 4; i++)
		{newShape->normalBuffer.push_back(QVector3D(0,1,0));}
		
		newShape->indexBuffer.push_back(0);
		newShape->indexBuffer.push_back(1);
		newShape->indexBuffer.push_back(2);
		
		newShape->indexBuffer.push_back(1);
		newShape->indexBuffer.push_back(3);
		newShape->indexBuffer.push_back(2);
         */

		return newShape;
    }
	

	Shape* CreateArrow(float tailSize, float thickness)
    {
        Shape* newShape = new Shape();
		
		// Lower face
		
		newShape->vertexBuffer.push_back(QVector3D(-0.5, 0,  0));
		newShape->vertexBuffer.push_back(QVector3D(0.5, 0,  0));
		newShape->vertexBuffer.push_back(QVector3D(0.5, 0, tailSize));
		newShape->vertexBuffer.push_back(QVector3D(-0.5, 0, tailSize));
		 
		newShape->vertexBuffer.push_back(QVector3D(-1., 0,  tailSize));
		newShape->vertexBuffer.push_back(QVector3D(1., 0,  tailSize));
		newShape->vertexBuffer.push_back(QVector3D(0, 0, tailSize + 1.));
		
		// Upper face
		
		newShape->vertexBuffer.push_back(QVector3D(-0.5, thickness,  0));
		newShape->vertexBuffer.push_back(QVector3D(0.5, thickness,  0));
		newShape->vertexBuffer.push_back(QVector3D(0.5, thickness, tailSize));
		newShape->vertexBuffer.push_back(QVector3D(-0.5, thickness, tailSize));
				
		newShape->vertexBuffer.push_back(QVector3D(-1., thickness,  tailSize));
		newShape->vertexBuffer.push_back(QVector3D(1., thickness,  tailSize));
		newShape->vertexBuffer.push_back(QVector3D(0, thickness, tailSize + 1.));
		
		for(int offset = 0; offset < 2; offset++)
		{
			addQuad(&(newShape->indexBuffer),offset*7,1+offset*7,2+offset*7,3+offset*7);
			
			newShape->indexBuffer.push_back(4+offset*7);
			newShape->indexBuffer.push_back(5+offset*7);
			newShape->indexBuffer.push_back(6+offset*7);
		}
		
		for (int i = 0; i < 4; i++)
		{
			addQuad(&(newShape->indexBuffer),i,(i+1)%4,(i+1)%4+7,i+7);
		}
		
		for (int i = 0; i < 3; i++)
		{
			addQuad(&(newShape->indexBuffer),i+4,(i+1)%3+4,(i+1)%3+11,i+11);
		}
	
		for(int i = 0; i < 7; i++)
		{newShape->normalBuffer.push_back(QVector3D(0,1,0));}
		for(int i = 7; i < 14; i++)
		{newShape->normalBuffer.push_back(QVector3D(0,1,0));}
		
		return newShape;
    }
	
	
	
}