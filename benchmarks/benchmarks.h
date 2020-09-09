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

#include <string>
#include <algorithm>
#include <vector>

#include "box2d/box2d.h"

class b2Benchmark {
public:
  b2Benchmark() : gravity(0.0f, -10.0f) {};
  
  virtual void InitBenchmark() {};
  virtual void InitWorld(b2World* world) { InitWorld(world, defaultSize); };
  virtual void InitWorld(b2World* world, int32 size) = 0;
  virtual void StepWorld(b2World* world) {
    world->Step(timeStep, velocityIterations, positionIterations);
  };

	b2Vec2 gravity;
	float timeStep = 1.0f / 60.0f;
	int32 velocityIterations = 8;
	int32 positionIterations = 3;
	int32 simulationSteps = 500;
	int32 defaultSize = 0;
	int32 startSize = 0;
	int32 endSize = 0;
	int32 sizeInc = 0;
	std::string name = "";
};

class b2Benchmarks {
public:
  std::vector<b2Benchmark*> benchmarks;
  
  ~b2Benchmarks() {
    for (auto benchmark : benchmarks) {
      delete benchmark;
    }
  }
  
  b2Benchmarks() {
    class b1 : public b2Benchmark {
      virtual void InitBenchmark() override {
        name = "Falling squares";
        simulationSteps = 1300;
        defaultSize = 300;
        startSize = 10;
        endSize = 300;
        sizeInc = 10;
      }
      
      virtual void InitWorld(b2World* world, int32 size) override {
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
			    
			    for (int32 j = i; j < size; ++j) {
				    b2BodyDef bd;
				    bd.type = b2_dynamicBody;
				    bd.position.x = i * 7 - 30;
				    bd.position.y = 2 * a * (size - j);
				    
				    b2Body* body = world->CreateBody(&bd);
			      body->CreateFixture(&shape, 5.0f);
			    }
		    }
      }
    };
    benchmarks.insert(benchmarks.begin(), new b1());
    
    class b2 : public b2Benchmark {
      virtual void InitBenchmark() override {
        name = "Falling circles";
        simulationSteps = 1300;
        defaultSize = 300;
        startSize = 10;
        endSize = 300;
        sizeInc = 10;
      }
      
      virtual void InitWorld(b2World* world, int32 size) override {
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
			    
			    for (int32 j = i; j < size; ++j) {
				    b2BodyDef bd;
				    bd.type = b2_dynamicBody;
				    bd.position.x = i * 7 + j * 0.25f - 100;
				    bd.position.y = 2 * a * (size - j);
				    
				    b2Body* body = world->CreateBody(&bd);
			      body->CreateFixture(&shape, 5.0f);
			    }
		    }
      }
    };
    benchmarks.insert(benchmarks.begin(), new b2());

    class b3 : public b2Benchmark {
      int32 m_count = 0;
      int32 e_count;
      
      virtual void InitBenchmark() override {
        name = "Tumbler";
        simulationSteps = 1500;
        defaultSize = 1000;
        startSize = 50;
        endSize = 1000;
        sizeInc = 50;
      }

      virtual void InitWorld(b2World* world, int32 size) override {
        e_count = size;
        b2Body* ground = NULL;
		    
		    {
			    b2BodyDef bd;
			    ground = world->CreateBody(&bd);
		    }

		    {
			    b2BodyDef bd;
			    bd.type = b2_dynamicBody;
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
      
      virtual void StepWorld(b2World* world) override {
        b2Benchmark::StepWorld(world);
        
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
    };
    benchmarks.insert(benchmarks.begin(), new b3());

    class b4 : public b2Benchmark {
      public: 
      virtual void InitBenchmark() override {
        name = "Add pair";
        simulationSteps = 1000;
        defaultSize = 2000;
        startSize = 100;
        endSize = 2500;
        sizeInc = 100;
        
        gravity = b2Vec2(0.0f, 0.0f);
      }
      
      virtual void InitWorld(b2World* world, int32 size) override {
		    {
			    b2CircleShape shape;
			    shape.m_p.SetZero();
			    shape.m_radius = 0.1f;

			    float minX = -9.0f;
			    float maxX = 9.0f;
			    float minY = 4.0f;
			    float maxY = 6.0f;
			    
			    for (int32 i = 0; i < size; ++i)
			    {
				    b2BodyDef bd;
				    bd.type = b2_dynamicBody;
				    bd.position = b2Vec2(minX + (maxX - minX) * i / (float) size, minY + (maxY - minY) * (i % 32) / 32.0f);
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
    };
    benchmarks.insert(benchmarks.begin(), new b4());
    
    class b5 : public b2Benchmark {
      virtual void InitBenchmark() override {
        name = "mild n^2";
        simulationSteps = 100;
        defaultSize = 200;
        startSize = 10;
        endSize = 200;
        sizeInc = 10;
      }
      
      virtual void InitWorld(b2World* world, int32 size) override {
		    {
			    b2BodyDef bd;
			    bd.position.Set(0.0f, 0.0f);
			    b2Body* body = world->CreateBody(&bd);

			    b2EdgeShape shape;
			    shape.SetTwoSided(b2Vec2(50.0f, 0.0f), b2Vec2(-50.0f, 0.0f));

			    body->CreateFixture(&shape, 0.0f);
		    }

		    // Table
		    for (int32 i = 0; i < size; ++i) {
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
		    for (int32 i = 0; i < size; ++i) {
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
    };
    benchmarks.insert(benchmarks.begin(), new b5());

    class b6 : public b2Benchmark {
      virtual void InitBenchmark() override {
        name = "n^2";
        simulationSteps = 100;
        defaultSize = 750;
        startSize = 25;
        endSize = 750;
        sizeInc = 25;
      }
      
      virtual void InitWorld(b2World* world, int32 size) override {
		    {
          for (int32 j = 0; j < size; ++j) {
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
    };
    benchmarks.insert(benchmarks.begin(), new b6());

    class b7 : public b2Benchmark {
      virtual void InitBenchmark() override {
        name = "Multi-fixture";
        simulationSteps = 500;
        defaultSize = 100;
        startSize = 5;
        endSize = 100;
        sizeInc = 5;
      }
      
      virtual void InitWorld(b2World* world, int32 size) override {
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
		    for (int32 i = 0; i < size; ++i) {
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
    };
    benchmarks.insert(benchmarks.begin(), new b7());
    
    class b8 : public b2Benchmark {
      virtual void InitBenchmark() override {
        name = "Mostly static (single body)";
        simulationSteps = 400;
        defaultSize = 200;
        startSize = 10;
        endSize = 200;
        sizeInc = 5;
      }
      
      virtual void InitWorld(b2World* world, int32 size) override {
		    {
			    float a = 0.5f;
			    b2BodyDef bd;
			    bd.position.y = -a;
			    b2Body* ground = world->CreateBody(&bd);

			    int32 N = size;
			    int32 M = size;
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
    };
    benchmarks.insert(benchmarks.begin(), new b8());
    
    class b9 : public b2Benchmark {
      virtual void InitBenchmark() override {
        name = "Mostly static (multi body)";
        simulationSteps = 500;
        defaultSize = 200;
        startSize = 10;
        endSize = 200;
        sizeInc = 5;
      }
      
      virtual void InitWorld(b2World* world, int32 size) override {
		    {
			    float a = 0.5f;
			    
			    int32 N = size;
			    int32 M = size;
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
    };
    benchmarks.insert(benchmarks.begin(), new b9());
    
    class b10 : public b2Benchmark {
      virtual void InitBenchmark() override {
        name = "Diagonal";
        simulationSteps = 1000;
        defaultSize = 50;
        startSize = 2;
        endSize = 50;
        sizeInc = 2;
      }
      
      virtual void InitWorld(b2World* world, int32 size) override {
		    {
			    float a = 0.5f;
			    
			    int32 N = size;
			    int32 M = size / 2;
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
    };
    benchmarks.insert(benchmarks.begin(), new b10());
    
    class b11 : public b2Benchmark {
      virtual void InitBenchmark() override {
        name = "Mixed static-dynamic";
        simulationSteps = 400;
        defaultSize = 6000;
        startSize = 100;
        endSize = 6000;
        sizeInc = 100;
      }
      
      virtual void InitWorld(b2World* world, int32 size) override {
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
			    for (int32 i = 0; i < size; ++i) {
            b2BodyDef bd;
			      bd.type = b2_dynamicBody;
			      b2Vec2 pos;
			      float s = i / (float) size;
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
    };
    benchmarks.insert(benchmarks.begin(), new b11());

    class b12 : public b2Benchmark {
      int32 maxDepth;
      
      virtual void InitBenchmark() override {
        name = "Big mobile";
        simulationSteps = 1000;
        defaultSize = 11;
        startSize = 1;
        endSize = 11;
        sizeInc = 1;
      }

      virtual void InitWorld(b2World* world, int32 size) override {
        maxDepth = size;
        b2Body* ground;

		    // Create ground body.
		    {
			    b2BodyDef bodyDef;
			    bodyDef.position.Set(0.0f, 20.0f);
			    ground = world->CreateBody(&bodyDef);
		    }

		    const float a = 0.25f;
		    b2Vec2 h(0.0f, a);

		    b2Body* root = AddNode(world, ground, b2Vec2_zero, 0, 200.0f, a);

		    b2RevoluteJointDef jointDef;
		    jointDef.bodyA = ground;
		    jointDef.bodyB = root;
		    jointDef.localAnchorA.SetZero();
		    jointDef.localAnchorB = h;
		    world->CreateJoint(&jointDef);
	    }

	    b2Body* AddNode(b2World* world, b2Body* parent, const b2Vec2& localAnchor, int32 depth, float offset, float a) {
		    const float density = 20.0f;

		    b2Vec2 h(0.0f, a);

		    b2Vec2 p = parent->GetPosition() + localAnchor - h;

		    b2BodyDef bodyDef;
		    bodyDef.type = b2_dynamicBody;
		    bodyDef.position = p;
		    b2Body* body = world->CreateBody(&bodyDef);

		    b2PolygonShape shape;
		    shape.SetAsBox(0.25f * a, a);
		    body->CreateFixture(&shape, density + p.x * 0.02f);

		    if (depth == maxDepth) {
			    return body;
		    }

		    shape.SetAsBox(offset, 0.25f * a, b2Vec2(0, -a), 0.0f);
		    body->CreateFixture(&shape, density);

		    b2Vec2 a1 = b2Vec2(offset, -a);
		    b2Vec2 a2 = b2Vec2(-offset, -a);
		    b2Body* body1 = AddNode(world, body, a1, depth + 1, 0.5f * offset, a);
		    b2Body* body2 = AddNode(world, body, a2, depth + 1, 0.5f * offset, a);

		    b2RevoluteJointDef jointDef;
		    jointDef.bodyA = body;
		    jointDef.localAnchorB = h;

		    jointDef.localAnchorA = a1;
		    jointDef.bodyB = body1;
		    world->CreateJoint(&jointDef);

		    jointDef.localAnchorA = a2;
		    jointDef.bodyB = body2;
		    world->CreateJoint(&jointDef);

		    return body;
	    }
    };
    benchmarks.insert(benchmarks.begin(), new b12());

    class b13 : public b2Benchmark {
      virtual void InitBenchmark() override {
        name = "Slow explosion";
        simulationSteps = 1000;
        defaultSize = 6000;
        startSize = 100;
        endSize = 6000;
        sizeInc = 100;
      }
      
      virtual void InitWorld(b2World* world, int32 size) override {
		    world->SetGravity(b2Vec2(0.0f,0.0f));
		    
		    {
			    for (int32 i = 0; i < size; ++i) {
            b2BodyDef bd;
			      bd.type = b2_dynamicBody;
			      b2Vec2 pos;
			      float s = i * 10 / (float) size;
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
    };
    benchmarks.insert(benchmarks.begin(), new b13());
  }
};
