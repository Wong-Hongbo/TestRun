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


#include "cloth.h"


Cloth::Cloth(	const Vec3& _origin_pos,
                int _num_particles_width,
                int _num_particles_height,
                double _step_x,
                double _step_y,
                double _smoothThreshold,
                double _heightThreshold,
                int rigidness,
                double time_step)
    : constraint_iterations(rigidness)
    , time_step(time_step)
    , smoothThreshold(_smoothThreshold)
    , heightThreshold(_heightThreshold)
    , num_particles_width(_num_particles_width)
    , num_particles_height(_num_particles_height)
    , origin_pos(_origin_pos)
    , step_x(_step_x)
    , step_y(_step_y)
{
    //创建粒子，2D girdmap上每个栅格一个
    particles.resize(num_particles_width*num_particles_height);
    double time_step2 = time_step*time_step;
    for (int i = 0; i < num_particles_width; i++)
    {
        for (int j = 0; j < num_particles_height; j++)
        {
            Vec3 pos(	origin_pos.x + i * step_x,
                        origin_pos.y,
                        origin_pos.z + j * step_y);

            particles[j*num_particles_width + i] = Particle(pos, time_step2); // insert particle in column i at j'th row
            particles[j*num_particles_width + i].pos_x = i;
            particles[j*num_particles_width + i].pos_y = j;
        }
    }

    // 创建粒子的连接弹簧
    for (int x = 0; x<num_particles_width; x++)
    {
        for (int y = 0; y<num_particles_height; y++)
        {
            if (x < num_particles_width - 1)
            {
                addConstraint(&getParticle(x, y), &getParticle(x + 1, y));
            }

            if (y < num_particles_height - 1)
            {
                addConstraint(&getParticle(x, y), &getParticle(x, y + 1));
            }

            if (x < num_particles_width - 1 && y < num_particles_height - 1)
            {
                addConstraint(&getParticle(x, y), &getParticle(x + 1, y + 1));
                addConstraint(&getParticle(x + 1, y), &getParticle(x, y + 1));
            }
        }
    }
}


double Cloth::timeStep()
{
    int particleCount = static_cast<int>(particles.size());
// 外力作用运动
//#pragma omp parallel for
    for (int i = 0; i < particleCount; i++)
    {
        particles[i].timeStep();
    }
/*
Instead of interating over all the constraints several times, we 
compute the overall displacement of a particle accroding to the rigidness
*/
// 内力作用运动
//#pragma omp parallel for
    for (int i = 0; i < particleCount; i++)
    {
        particles[i].satisfyConstraintSelf(constraint_iterations);
    }

    double maxDiff = 0;
// 计算可移动粒子的最大位移
//#pragma omp parallel for
    for (int i = 0; i < particleCount; i++)
    {
        if (particles[i].isMovable())
        {
            double diff = std::abs(particles[i].old_pos.y - particles[i].pos.y);
            if (diff > maxDiff)
                maxDiff = diff;
        }
    }

    return maxDiff;
}

void Cloth::addForce(const Vec3& direction)
{
    int particleCount = static_cast<int>(particles.size());

    // add the forces to each particle
#pragma omp parallel for
    for (int i = 0; i < particleCount; i++)
    {
        particles[i].addForce(direction);
    }
}


//testing the collision
void Cloth::terrainCollision()
{
    assert(particles.size() == heightvals.size());
    int particleCount = static_cast<int>(particles.size());
    // 如果当前位置比heightval还要低，则置为heightval和不可移动
#pragma omp parallel for
    for (int i = 0; i < particleCount; i++)
    {
        Particle& particle = particles[i];
        if (particle.pos.y < heightvals[i]) // if the particle is inside the ball
        {
            particle.offsetPos(Vec3(0, heightvals[i] - particle.pos.y, 0));
            particle.makeUnmovable();
        }
    }
}
