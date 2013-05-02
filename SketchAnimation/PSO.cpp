#include "PSO.h"

#include <cstdlib>
#include <memory>

/* Definitions */
#define INERTIA 0.8 
#define C_SOC 1.9			/* Social component constant*/
#define C_COG 1.9			/* Cognitive component constant */
#define MAX_VEL 10.0 

Particle::Particle()
{

}
Particle::Particle(int space_dim)
{
	m_iDim = space_dim;

	position = new float[space_dim];
	velocity = new float[space_dim];
	personal_best = new float[space_dim];
}

Particle::~Particle()
{
	delete []position;
	delete []velocity;
	delete []personal_best;
};

// generate the particle using random method
void Particle::particle_init()
{
	for(int j=0; j<m_iDim; ++j) {
		float random_pso = ( ( (float)rand() / (float)RAND_MAX ) * 200) - 100;
		position[j] = random_pso; // from -100 to +100, as defined
		velocity[j] =0.0f;
		personal_best[j] = position[j];
	}

}

// generate the particle using given value
void Particle::particle_init(float* pos, float* vel, float* per_best)
{
	memcpy(position,pos,sizeof(float) * m_iDim);
	memcpy(velocity,vel,sizeof(float) * m_iDim);
	memcpy(personal_best,per_best,sizeof(float) * m_iDim);
}

Particle& Particle::operator=(const Particle& particle)
{
	if(this != &particle)
	{
		memcpy(position,particle.position,sizeof(float) * particle.m_iDim);
		memcpy(velocity,particle.velocity,sizeof(float) * particle.m_iDim);
		memcpy(personal_best,particle.personal_best,sizeof(float) * particle.m_iDim);
	}
	return *this;
}


PSO::PSO(void)
{
	m_iParticleNum = 0;
}


PSO::~PSO(void)
{
	if(m_iParticleNum != 0)
	{
		for(int i = 0; i < m_iParticleNum; i++)
			delete m_pParticles[i];

		delete []m_pParticles;
	}

	delete m_GlobalBestPosition;
}

// allocate the memory
void PSO::createParticles(int num,int dim)
{
	m_iParticleNum = num;

	m_pParticles = new Particle*[num];
	for(int i = 0; i < num; i++)
	{
		m_pParticles[i] = new Particle(dim);
	}

	m_GlobalBestPosition = new float[dim];
}

// initialize those particles
void PSO::initParticle(const Particle& particle, int index)
{
	*m_pParticles[index] = particle;
}

// update the particle
void PSO::updateParticles()
{
	int SPACE_DIM = m_pParticles[0]->m_iDim;

	//for(int i=0; i<m_iMaxIterations; ++i) 
	{
		for(int pos=0; pos<m_iParticleNum; ++pos) 
		{
			for(int direction=0; direction<SPACE_DIM; ++direction)
			{
				// update its position
				m_pParticles[pos]->position[direction] = m_pParticles[pos]->position[direction] + m_pParticles[pos]->velocity[direction];
				
				// Compute new velocity
				float r1 = (float)rand() / (float)RAND_MAX;
				float r2 = (float)rand() / (float)RAND_MAX;
				m_pParticles[pos]->velocity[direction] = INERTIA*m_pParticles[pos]->velocity[direction] + 
					r1 * C_SOC * (m_GlobalBestPosition[direction] - m_pParticles[pos]->position[direction]) +
					r2 * C_SOC * (m_pParticles[pos]->personal_best[direction] - m_pParticles[pos]->position[direction]);

				// Limit velocity
				if(m_pParticles[pos]->velocity[direction] > MAX_VEL)
					m_pParticles[pos]->velocity[direction] = MAX_VEL;
				else if(m_pParticles[pos]->velocity[direction] < -MAX_VEL)
					m_pParticles[pos]->velocity[direction] = -MAX_VEL;
			}
		}
	}

}


// set the value
void PSO::setPosition(int particle_idx, int dimension_idx, float pos)
{
	m_pParticles[particle_idx]->position[dimension_idx] = pos;
}

// obtain the value
float PSO::getPosition(int particle_idx, int dimension_idx)
{
	return m_pParticles[particle_idx]->position[dimension_idx];
}


// set the personal best position
void PSO::setPersonalBestPosition(int particle_idx, int dimension_idx, float pos)
{
	m_pParticles[particle_idx]->personal_best[dimension_idx] = pos;
}

// set velocity
void PSO::setVelocity(int particle_idx, int dimension_idx, float vel)
{
	m_pParticles[particle_idx]->velocity[dimension_idx] = vel;
}

// set the global best position
void PSO::setGlobalBestPosition(int dimension_idx, float best_pos)
{
	m_GlobalBestPosition[dimension_idx] = best_pos;
}

// obtain the global best position
float PSO::getGlobalBestPosition(int dimension_idx)
{
	return m_GlobalBestPosition[dimension_idx];
}

// initialize global best position
void PSO::initGlobalBestPosition(const float* position,int dim)
{
	memcpy(m_GlobalBestPosition,position,sizeof(float) * dim);
}
