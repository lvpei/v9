#pragma once

// define the particle
class Particle
{
public:
	float* position;
	float* velocity;
	float* personal_best;
	int m_iDim;

	Particle();
	Particle(int space_dim);
	~Particle();

	void particle_init();
	void particle_init(float* position, float* velocity, float* personal_best);

public:
	Particle& operator= (const Particle& );
} ;


// PSO algorithm
class PSO
{
public:
	PSO(void);
	virtual ~PSO(void);

	// allocate the memory
	void createParticles(int num,int dim);

	// initialize those particles
	void initParticle(const Particle& particle, int index);	

	// initialize those particles
	void initParticle(const float* position, const float* velocity, const float* personal_best, int index);	
	
	// initialize global best position
	void initGlobalBestPosition(const float* position,int dim);	

	// update the particle
	void updateParticles();									

	// set the iteration times
	void setIterationTimes(int times){m_iMaxIterations = times;}	

	// set & obtain position
	void setPosition(int particle_idx, int dimension_idx, float pos);
	float getPosition(int particle_idx, int dimension_idx);

	// set the personal best position
	void setPersonalBestPosition(int particle_idx, int dimension_idx, float pos);

	// set velocity
	void setVelocity(int particle_idx, int dimension_idx, float vel);

	// set & obtain global best position
	void setGlobalBestPosition(int dimension_idx, float best_pos);
	float getGlobalBestPosition(int dimension_idx);

private:
	
	// the data of particles
	Particle**	m_pParticles;			
	
	// the global best particles
	float*	m_GlobalBestPosition;

	// the number of particles 
	int			m_iParticleNum;

	// the maximum of iteration times
	int			m_iMaxIterations;	
};

