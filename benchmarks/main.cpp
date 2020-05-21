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
  benchmarks.insert(benchmarks.begin(), &b1);
  
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
  benchmarks.insert(benchmarks.begin(), &b2);

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
  benchmarks.insert(benchmarks.begin(), &b3);

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
  benchmarks.insert(benchmarks.begin(), &b4);
  
  class : public b2Benchmark {
    virtual void InitBenchmark() override {
      name = "mild n^2";
      simulationSteps = 100;
    }
    
    virtual void InitWorld(b2World* world) override {
		  {
			  b2BodyDef bd;
			  bd.position.Set(0.0f, 0.0f);
			  b2Body* body = world->CreateBody(&bd);

			  b2EdgeShape shape;
			  shape.Set(b2Vec2(50.0f, 0.0f), b2Vec2(-50.0f, 0.0f));

			  body->CreateFixture(&shape, 0.0f);
		  }

		  // Table
		  for (int32 i = 0; i < 200; ++i) {
			  b2BodyDef bd;
			  bd.type = b2_dynamicBody;
			  bd.position.Set(-5.0f, 1.0f);
			  b2Body* m_table2 = world->CreateBody(&bd);

			  b2PolygonShape top;
			  top.SetAsBox(3.0f, 0.5f, b2Vec2(0.0f, 3.5f), 0.0f);

			  b2PolygonShape leftLeg;
			  leftLeg.SetAsBox(0.5f, 2.0f, b2Vec2(-2.5f, 2.0f), 0.0f);

			  b2PolygonShape rightLeg;
			  rightLeg.SetAsBox(0.5f, 2.0f, b2Vec2(2.5f, 2.0f), 0.0f);

			  m_table2->CreateFixture(&top, 2.0f);
			  m_table2->CreateFixture(&leftLeg, 2.0f);
			  m_table2->CreateFixture(&rightLeg, 2.0f);
		  }

		  // Spaceship
		  for (int32 i = 0; i < 200; ++i) {
			  b2BodyDef bd;
			  bd.type = b2_dynamicBody;
			  bd.position.Set(15.0f, 1.0f);
			  b2Body* m_ship2 = world->CreateBody(&bd);

			  b2Vec2 vertices[3];

			  b2PolygonShape left;
			  vertices[0].Set(-2.0f, 0.0f);
			  vertices[1].Set(1.0f, 2.0f);
			  vertices[2].Set(0.0f, 4.0f);
			  left.Set(vertices, 3);

			  b2PolygonShape right;
			  vertices[0].Set(2.0f, 0.0f);
			  vertices[1].Set(-1.0f, 2.0f);
			  vertices[2].Set(0.0f, 4.0f);
			  right.Set(vertices, 3);

			  m_ship2->CreateFixture(&left, 2.0f);
			  m_ship2->CreateFixture(&right, 2.0f);
		  }
    }
  } b5;
  benchmarks.insert(benchmarks.begin(), &b5);

  class : public b2Benchmark {
    virtual void InitBenchmark() override {
      name = "n^2";
      simulationSteps = 100;
    }
    
    virtual void InitWorld(b2World* world) override {
		  {
        for (int32 j = 0; j < 750; ++j) {
          b2BodyDef bd;
          bd.type = b2_dynamicBody;
          bd.position = {j * 0.01f, j * 0.01f};
          b2Body* ground = world->CreateBody(&bd);
          b2CircleShape shape;
          shape.m_radius = 1;
          ground->CreateFixture(&shape, 1.0f);
        }
		  }
    }
  } b6;
  benchmarks.insert(benchmarks.begin(), &b6);

  class : public b2Benchmark {
    virtual void InitBenchmark() override {
      name = "Multi-fixture";
      simulationSteps = 500;
    }
    
    virtual void InitWorld(b2World* world) override {
		  {
			  b2BodyDef bd;
			  bd.position.Set(0.0f, 0.0f);
			  b2Body* body = world->CreateBody(&bd);

			  b2PolygonShape shape;
			  
			  shape.SetAsBox(35, 1, b2Vec2(0.0f, 0.0f), 0);
			  body->CreateFixture(&shape, 0.0f);
			  
			  shape.SetAsBox(1, 25, b2Vec2(-36.0f, 24.0f), 0);
			  body->CreateFixture(&shape, 0.0f);
			  
			  shape.SetAsBox(1, 25, b2Vec2(36.0f, 24.0f), 0);
			  body->CreateFixture(&shape, 0.0f);
		  }

		  // Table
		  for (int32 i = 0; i < 100; ++i) {
			  b2BodyDef bd;
			  bd.type = b2_dynamicBody;
			  bd.position.Set(-20.0f + (i % 6) * 7 + i / 10, 1.0f + (i / 6) * 5);
			  b2Body* m_table2 = world->CreateBody(&bd);

        const int32 c = 50 - i / 2;

        for (int32 z = 0; z < c; ++z) {
          b2PolygonShape top;
          float bs = 2.0f / c;
          float ps = 2.0f * z * bs + bs;
          
    			top.SetAsBox(bs, 0.5f, b2Vec2(ps - 2.0f, 3.5f), 0.0f);
	  
    			b2PolygonShape leftLeg;
			    leftLeg.SetAsBox(0.5f, bs, b2Vec2(-1.5f, ps), 0.0f);

			    b2PolygonShape rightLeg;
			    rightLeg.SetAsBox(0.5f, bs, b2Vec2(1.5f, ps), 0.0f);

			    m_table2->CreateFixture(&leftLeg, 2.0f / c);
			    m_table2->CreateFixture(&rightLeg, 2.0f / c);
          m_table2->CreateFixture(&top, 2.0f / c);
    		}
		  }
    }
  } b7;
  benchmarks.insert(benchmarks.begin(), &b7);
  
  class : public b2Benchmark {
    virtual void InitBenchmark() override {
      name = "Mostly static (single body)";
      simulationSteps = 400;
    }
    
    virtual void InitWorld(b2World* world) override {
		  {
			  float a = 0.5f;
			  b2BodyDef bd;
			  bd.position.y = -a;
			  b2Body* ground = world->CreateBody(&bd);

			  int32 N = 200;
			  int32 M = 200;
			  b2Vec2 position;
			  position.y = 0.0f;
			  for (int32 j = 0; j < M; ++j) {
				  position.x = -N * a;

				  for (int32 i = 0; i < N; ++i) {
				    if (b2Abs(j - i) > 3) {
    				  b2PolygonShape shape;
    					shape.SetAsBox(a, a, position, 0.0f);
    					ground->CreateFixture(&shape, 0.0f);
				    } else if (i == j) {
				      bd.type = b2_dynamicBody;
				      bd.position = position;
				      b2Body* body = world->CreateBody(&bd);
				      b2CircleShape shape;
				      shape.m_radius = a * 2;
				      body->CreateFixture(&shape, 1.0f);
				    }
				    
					  position.x += 2.0f * a;
				  }
				  
				  position.y -= 2.0f * a;
			  }
		  }
    }
  } b8;
  benchmarks.insert(benchmarks.begin(), &b8);
  
  class : public b2Benchmark {
    virtual void InitBenchmark() override {
      name = "Mostly static (multi body)";
      simulationSteps = 500;
    }
    
    virtual void InitWorld(b2World* world) override {
		  {
			  float a = 0.5f;
			  
			  int32 N = 200;
			  int32 M = 200;
			  b2Vec2 position;
			  position.y = 0.0f;
			  for (int32 j = 0; j < M; ++j) {
				  position.x = -N * a;

				  for (int32 i = 0; i < N; ++i) {
				    if (b2Abs(j - i) > 3) {
    				  b2BodyDef bd;
			        bd.position = position;      
			        b2Body* ground = world->CreateBody(&bd);
              b2PolygonShape shape;
    					shape.SetAsBox(a, a);
    					ground->CreateFixture(&shape, 0.0f);
				    } else if (i == j) {
				      b2BodyDef bd;
			        bd.type = b2_dynamicBody;
				      bd.position = position;
				      b2Body* body = world->CreateBody(&bd);
				      b2CircleShape shape;
				      shape.m_radius = a * 2;
				      body->CreateFixture(&shape, 1.0f);
				    }
				    
					  position.x += 2.0f * a;
				  }
				  
				  position.y -= 2.0f * a;
			  }
		  }
    }
  } b9;
  benchmarks.insert(benchmarks.begin(), &b9);
  
  class : public b2Benchmark {
    virtual void InitBenchmark() override {
      name = "Diagonal";
      simulationSteps = 1000;
    }
    
    virtual void InitWorld(b2World* world) override {
		  {
			  float a = 0.5f;
			  
			  int32 N = 50;
			  int32 M = 25;
			  b2Vec2 position;
			  position.y = 0.0f;
			  for (int32 j = 0; j < M; ++j) {
				  position.x = -N * a;

				  for (int32 i = 0; i < N; ++i) {
				    b2BodyDef bd;
			      bd.position = position;
				    b2Body* body = world->CreateBody(&bd);
				    
				    b2PolygonShape shape;
    				shape.SetAsBox(a, (3 * j + 1) * a, position, b2_pi / 4);
    				body->CreateFixture(&shape, 0.0f);
				    
					  position.x += 8.0f * a;
				  }
				  
				  position.y -= 8.0f * a;
			  }
		  }
		  
		  {
			  for (int32 i = 0; i < 3000; ++i) {
				  b2BodyDef bd;
				  b2Vec2 pos;
				  pos.x = (i / 15) * 2 - 75;
				  pos.y = (i % 15) * 2 + 50;
			    bd.position = pos;
				  bd.type = b2_dynamicBody;
				  b2Body* body = world->CreateBody(&bd);
				  
				  b2CircleShape shape;
				  shape.m_radius = 0.5f;
    			body->CreateFixture(&shape, 1.0f);
			  }
		  }
    }
  } b10;
  benchmarks.insert(benchmarks.begin(), &b10);
  
  class : public b2Benchmark {
    virtual void InitBenchmark() override {
      name = "Mixed static/dynamic";
      simulationSteps = 400;
    }
    
    virtual void InitWorld(b2World* world) override {
		  int32 N = 150;
		  int32 M = 150;
		  b2Vec2 cntr = {M / 2.0f, N / 2.0f};
		  float a = 0.5f;
			  
		  {
			  b2BodyDef bd;
			  b2Body* ground = world->CreateBody(&bd);

			  for (int32 j = 0; j < M; ++j) {
				  for (int32 i = 0; i < N; ++i) {
				    b2Vec2 pos;
				    pos.x = i;
				    pos.y = j;
				    
				    if (b2Dot(pos - cntr, pos - cntr) > 67 * 67) {
				      b2CircleShape shape;
      				shape.m_radius = a;
      				shape.m_p = pos;
      				ground->CreateFixture(&shape, 0.0f);
				    }
    			}
			  }
		  }
		  
		  {
			  for (int32 i = 0; i < 6000; ++i) {
          b2BodyDef bd;
			    bd.type = b2_dynamicBody;
			    b2Vec2 pos;
			    float s = i / 6000.0f;
			    pos.x = cosf(s * 30.0f) * (s * 50 + 10);
			    pos.y = sinf(s * 30.0f) * (s * 50 + 10);
				  bd.position = pos + cntr;
				  b2Body* body = world->CreateBody(&bd);
				  body->SetLinearVelocity(pos);
				  b2CircleShape shape;
				  shape.m_radius = a;
				  body->CreateFixture(&shape, 0.5f);
			  }
		  }
    }
  } b11;
  benchmarks.insert(benchmarks.begin(), &b11);

  class : public b2Benchmark {
    virtual void InitBenchmark() override {
      name = "Slow explosion";
      simulationSteps = 1000;
    }
    
    virtual void InitWorld(b2World* world) override {
		  world->SetGravity(b2Vec2(0.0f,0.0f));
		  
		  {
			  for (int32 i = 0; i < 6000; ++i) {
          b2BodyDef bd;
			    bd.type = b2_dynamicBody;
			    b2Vec2 pos;
			    float s = i / 600.0f;
			    pos.x = cosf(s * 30.0f) * (s * 30 + 5);
			    pos.y = sinf(s * 30.0f) * (s * 30 + 5);
				  bd.position = pos;
				  b2Body* body = world->CreateBody(&bd);
				  body->SetLinearVelocity(0.2f * pos);
				  b2CircleShape shape;
				  shape.m_radius = 0.5f;
				  body->CreateFixture(&shape, 0.5f);
			  }
		  }
    }
  } b13;
  benchmarks.insert(benchmarks.begin(), &b13);

  for (auto benchmark : benchmarks) {
    benchmark->InitBenchmark();
    int32 iterations = 1;
    int64 totalTime = 0;
    int64 maxTime = 0;
    
    for (int32 i = 0; i < iterations; ++i) {
	    // Construct a world object, which will hold and simulate the rigid bodies.
	    b2World world(benchmark->gravity);
	    world.SetContinuousPhysics(false);

      benchmark->InitWorld(&world);

      using std::chrono::high_resolution_clock;
	    int64 maxTime0 = 0;

	    for (int32 i = 0; i < benchmark->simulationSteps; ++i) {
		    auto start = high_resolution_clock::now();

		    world.Step(benchmark->timeStep, benchmark->velocityIterations, benchmark->positionIterations);
		    benchmark->StepWorld(&world);

	      auto finish = high_resolution_clock::now();
  	    int64 time = (finish - start).count();
  	    totalTime += time;
  	    maxTime0 = b2Max(maxTime0, time);
	    }
	    
	    maxTime = (maxTime == 0)? maxTime0 : b2Min(maxTime0, maxTime);
    }
    
    std::cout << std::left << std::setw(30) << benchmark->name << "  " << std::right
              << std::setw(8) << "total: "
              << std::setw(12) << (int32) (totalTime / 1000000.0 / iterations) << " ms"
              << std::setw(8) << " max:  "
              << std::setw(12) << (((int32) (maxTime / 10000.0)) / 100.0) << " ms" <<  std::endl;
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
