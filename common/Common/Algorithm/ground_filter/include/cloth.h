//#######################################################################################
//#  This source code is about a ground filtering algorithm for airborn LiDAR data      #
//#  based on physical process simulations, specifically cloth simulation.              #
//#                                     Copyright 2021                                  #
//#                  Changsha Xingshen Intelligent Technology Co., Ltd                  #
//#                               (http://www.xingshentech.com/)                        #
//#                                                                                     #
//#                                  Authored By Meng Deyuan                            #
//#                                                                                     #
//#######################################################################################


//This source code is about a ground filtering algorithm for airborn LiDAR data
//based on physical process simulations, specifically cloth simulation.
//
//This code is based on a Cloth Simulation Tutorial at the cg.alexandra.dk blog.
//Thanks to Jesper Mosegaard (clothTutorial@jespermosegaard.dk)
//
//When applying the cloth simulation to LIDAR point filtering. A lot of features
//have been added to the original source code, including
//* configuration file management
//* point cloud data read/write
//* point-to-point collsion detection
//* nearest point search structure from CGAL
//* addding a terrain class

//using discrete steps (drop and pull) to approximate the physical process
//Finding the max height value in nearest N points aroud every particles, as the lowest position where the particles can get.��ÿ�����ϵ���Χ�����ڽ���N���㣬�Ը߳�����ֵ��Ϊ���ܵ��������͵㡣

#ifndef _CLOTH_H_
#define _CLOTH_H_

//local
#include "particle.h"

//system
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <queue>


struct XY
{
    XY(int x1, int y1)
        : x(x1)
        , y(y1)
    {}
    int x;
    int y;
};
using namespace std;

class Cloth
{
private:

    // total number of particles is num_particles_width*num_particles_height
    int constraint_iterations;

    double time_step;

    std::vector<Particle> particles; // all particles that are part of this cloth
//	std::vector<Constraint> constraints; // alle constraints between particles as part of this cloth

    //parameters of slope postpocessing
    double smoothThreshold;
    double heightThreshold;

    //heightvalues
    std::vector<double> heightvals;

    //movable particle index
    std::vector<int> movableIndex;
    std::vector< std::vector<int> > particle_edges;

    //record
    inline void addConstraint(Particle *p1, Particle *p2)
    { /*constraints.push_back(Constraint(p1, p2));*/
        p1->neighborsList.push_back(p2);  //record the neighbor for each particle
        p2->neighborsList.push_back(p1);
    }

public:

    inline Particle& getParticle(int x, int y) { return particles[y*num_particles_width + x]; }
    inline const Particle& getParticle(int x, int y) const { return particles[y*num_particles_width + x]; }
    inline Particle& getParticleByIndex(int index) { return particles[index]; }
    inline const Particle getParticleByIndex(int index) const { return particles[index]; }

    int num_particles_width; // number of particles in "width" direction
    int num_particles_height; // number of particles in "height" direction
    Vec3 origin_pos;
    double step_x, step_y;

    inline int getSize() const { return num_particles_width * num_particles_height; }

    inline std::vector<double>& getHeightvals() { return heightvals; }

public:

    /* This is a important constructor for the entire system of particles and constraints */
    Cloth(	const Vec3& origin_pos,
            int num_particles_width,
            int num_particles_height,
            double step_x,
            double step_y,
            double smoothThreshold,
            double heightThreshold,
            int rigidness,
            double time_step);

    void setheightvals(const std::vector<double>& heightvals)
    {
        this->heightvals = heightvals;
    }

    /** This is an important methods where the time is progressed one time step for the entire cloth.
        This includes calling satisfyConstraint() for every constraint, and calling timeStep() for all particles
    **/
    double timeStep();

    /* used to add gravity (or any other arbitrary vector) to all particles */
    void addForce(const Vec3& direction);

    //detecting collision of cloth and terrain
    void terrainCollision();
};

#endif
