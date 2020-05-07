// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <chrono>
#include <stdio.h>
#include <unistd.h>
#include <sys/resource.h>

#include "box2d/box2d.h"

class b2Benchmark {
public:
  b2Benchmark() : gravity(0.0f, -10.0f) {};
  
  virtual void InitBenchmark() {};
  virtual void InitWorld(b2World* world) = 0;
  virtual void StepWorld(b2World* world) {};

	b2Vec2 gravity;
	float timeStep = 1.0f / 60.0f;
	int32 velocityIterations = 8;
	int32 positionIterations = 3;
	int32 simulationSteps = 500;
	std::string name = "";
};

int main() {
  std::vector<b2Benchmark*> benchmarks;
  
  class : public b2Benchmark {
    virtual void InitBenchmark() override {
      name = "Falling squares";
      simulationSteps = 1300;
    }
    
    virtual void InitWorld(b2World* world) override {
      {
			  b2BodyDef bd;
			  b2Body* ground = world->CreateBody(&bd);

			  b2PolygonShape shape;
			  b2Vec2 position(0, -10);
			  shape.SetAsBox(100, 3, position, 0.0f);
			  ground->CreateFixture(&shape, 0.0f);
		  }

		  for (int32 i = 0; i < 15; ++i) {
		    float a = 0.5f + i / 15.0f * 2.5f;
        b2PolygonShape shape;
		    shape.SetAsBox(a, a);
			  
			  for (int32 j = i; j < 300; ++j) {
				  b2BodyDef bd;
				  bd.type = b2_dynamicBody;
				  bd.position.x = i * 7 - 30;
				  bd.position.y = 300 * 2 * a - j * 2 * a;
				  
				  b2Body* body = world->CreateBody(&bd);
			    body->CreateFixture(&shape, 5.0f);
			  }
		  }
    }
  } b1;
  benchmarks.push_back(&b1);
  
  class : public b2Benchmark {
    virtual void InitBenchmark() override {
      name = "Falling circles";
      simulationSteps = 1300;
    }
    
    virtual void InitWorld(b2World* world) override {
      {
			  b2BodyDef bd;
			  b2Body* ground = world->CreateBody(&bd);

			  b2PolygonShape shape;
			  b2Vec2 position(0, -10);
			  shape.SetAsBox(100, 3, position, 0.0f);
			  ground->CreateFixture(&shape, 0.0f);
		  }

		  for (int32 i = 0; i < 15; ++i) {
		    float a = 0.5f + i / 15.0f * 2.5f;
        b2CircleShape shape;
		    shape.m_radius = a / 2;
			  
			  for (int32 j = i; j < 300; ++j) {
				  b2BodyDef bd;
				  bd.type = b2_dynamicBody;
				  bd.position.x = i * 7 + j * 0.25f - 100;
				  bd.position.y = 300 * 2 * a - j * 2 * a;
				  
				  b2Body* body = world->CreateBody(&bd);
			    body->CreateFixture(&shape, 5.0f);
			  }
		  }
    }
  } b2;
  benchmarks.push_back(&b2);

  class : public b2Benchmark {
    virtual void InitBenchmark() override {
      name = "Tumbler";
      simulationSteps = 1500;
    }

    virtual void InitWorld(b2World* world) override {
      b2Body* ground = NULL;
		  
		  {
			  b2BodyDef bd;
			  ground = world->CreateBody(&bd);
		  }

		  {
			  b2BodyDef bd;
			  bd.type = b2_dynamicBody;
			  bd.allowSleep = false;
			  bd.position.Set(0.0f, 10.0f);
			  b2Body* body = world->CreateBody(&bd);

			  b2PolygonShape shape;
			  shape.SetAsBox(0.5f, 10.0f, b2Vec2( 10.0f, 0.0f), 0.0);
			  body->CreateFixture(&shape, 5.0f);
			  shape.SetAsBox(0.5f, 10.0f, b2Vec2(-10.0f, 0.0f), 0.0);
			  body->CreateFixture(&shape, 5.0f);
			  shape.SetAsBox(10.0f, 0.5f, b2Vec2(0.0f, 10.0f), 0.0);
			  body->CreateFixture(&shape, 5.0f);
			  shape.SetAsBox(10.0f, 0.5f, b2Vec2(0.0f, -10.0f), 0.0);
			  body->CreateFixture(&shape, 5.0f);

			  b2RevoluteJointDef jd;
			  jd.bodyA = ground;
			  jd.bodyB = body;
			  jd.localAnchorA.Set(0.0f, 10.0f);
			  jd.localAnchorB.Set(0.0f, 0.0f);
			  jd.referenceAngle = 0.0f;
			  jd.motorSpeed = 0.05f * b2_pi;
			  jd.maxMotorTorque = 1e8f;
			  jd.enableMotor = true;
			  world->CreateJoint(&jd);
		  }
    }
    
    int32 m_count = 0;
    
    virtual void StepWorld(b2World* world) override {
      const int32 e_count = 1000;
      
      if (m_count < e_count) {
			  b2BodyDef bd;
			  bd.type = b2_dynamicBody;
			  bd.position.Set(0.0f, 10.0f);
			  b2Body* body = world->CreateBody(&bd);

			  b2PolygonShape shape;
			  shape.SetAsBox(0.125f, 0.125f);
			  body->CreateFixture(&shape, 1.0f);

			  ++m_count;
		  }
    }
  } b3;
  benchmarks.push_back(&b3);

  class : public b2Benchmark {
    virtual void InitBenchmark() override {
      name = "Add pair";
      simulationSteps = 1000;
      gravity = b2Vec2(0.0f, 0.0f);
    }
    
    virtual void InitWorld(b2World* world) override {
		  {
			  b2CircleShape shape;
			  shape.m_p.SetZero();
			  shape.m_radius = 0.1f;

			  float minX = -9.0f;
			  float maxX = 9.0f;
			  float minY = 4.0f;
			  float maxY = 6.0f;
			  
			  for (int32 i = 0; i < 2000; ++i)
			  {
				  b2BodyDef bd;
				  bd.type = b2_dynamicBody;
				  bd.position = b2Vec2(minX + (maxX - minX) * i / 2000.0f, minY + (maxY - minY) * (i % 32) / 32.0f);
				  b2Body* body = world->CreateBody(&bd);
				  body->CreateFixture(&shape, 0.01f);
			  }
		  }
		  
		  {
			  b2PolygonShape shape;
			  shape.SetAsBox(1.5f, 1.5f);
			  b2BodyDef bd;
			  bd.type = b2_dynamicBody;
			  bd.position.Set(-40.0f,5.0f);
			  b2Body* body = world->CreateBody(&bd);
			  body->CreateFixture(&shape, 1.0f);
			  body->SetLinearVelocity(b2Vec2(150.0f, 0.0f));
		  }
    }
  } b4;
  benchmarks.push_back(&b4);
  
  for (auto benchmark : benchmarks) {
    benchmark->InitBenchmark();
    int32 iterations = 1;
    int64 totalTime = 0;
    
    for (int32 i = 0; i < iterations; ++i) {
	    // Construct a world object, which will hold and simulate the rigid bodies.
	    b2World world(benchmark->gravity);
	    world.SetContinuousPhysics(false);

      benchmark->InitWorld(&world);

      using std::chrono::high_resolution_clock;
	    auto start = high_resolution_clock::now();
	    
	    // This is our little game loop.
	    for (int32 i = 0; i < benchmark->simulationSteps; ++i) {
		    // Instruct the world to perform a single step of simulation.
		    // It is generally best to keep the time step and iterations fixed.
		    world.Step(benchmark->timeStep, benchmark->velocityIterations, benchmark->positionIterations);
		    benchmark->StepWorld(&world);
	    }
	    
	    auto finish = high_resolution_clock::now();
	    int64 time = (finish - start).count();
	    totalTime += time;
    }
    
    std::cout.precision(std::numeric_limits<float>::digits10);
    std::cout << std::left << std::setw(20) << benchmark->name << ": "
              << std::right << std::setw(15) << (totalTime / 1000000.0 / iterations) << " ms" << std::endl;
  }

/*
	int32 mem = -1;
	
#if __linux__
	long rss = 0L;
  FILE* fp = NULL;
  if ((fp = fopen( "/proc/self/statm", "r" )) == NULL ) {
    return -1;
  }
  
  if (fscanf(fp, "%*s%ld", &rss ) != 1) {
    fclose(fp);
    return -1;
  }
  
  fclose(fp);
  mem = ((size_t) rss * (size_t) sysconf(_SC_PAGESIZE));
#endif
  
  std::cout << "time: " << time << ", mem: " << mem << std::endl;
*/

	return 0;
}
