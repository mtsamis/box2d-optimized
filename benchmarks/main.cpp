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
#include <chrono>
#include <stdio.h>
#include <unistd.h>
#include <sys/resource.h>

#include "box2d/box2d.h"

void InitWorld(b2World* world) {
  const int32 e_count = 200;
  
  {
    float a = 0.5f;
	  b2BodyDef bd;
	  bd.position.y = -a;
	  b2Body* ground = world->CreateBody(&bd);
	  
	  int32 N = 200;
	  int32 M = 10;
	  b2Vec2 position;
	  position.y = 0.0f;
	  for (int32 j = 0; j < M; ++j) {
		  position.x = -N * a;
		  for (int32 i = 0; i < N; ++i) {
			  b2PolygonShape shape;
			  shape.SetAsBox(a, a, position, 0.0f);
			  ground->CreateFixture(&shape, 0.0f);
			  position.x += 2.0f * a;
		  }
		  position.y -= 2.0f * a;
	  }
	}
	
	{
	  float a = 0.5f;
		b2PolygonShape shape;
		shape.SetAsBox(a, a);
    b2FixtureDef ds;
	  ds.shape = &shape;
	  ds.density = 5;
	   
		b2Vec2 x(-7.0f, 0.75f);
		b2Vec2 y;
		b2Vec2 deltaX(0.5625f, 1.25f);
		b2Vec2 deltaY(1.125f, 0.0f);
		
		for (int32 i = 0; i < e_count; ++i) {
			y = x;

			for (int32 j = i; j < e_count; ++j) {
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position = y;
				bd.position.y = 190 - bd.position.y;

				b2Body* body = world->CreateBody(&bd);
				body->CreateFixture(&ds);
				y += deltaY;
			}
			
			x += deltaX;
		}
	}
}

int main() {
	// Define the gravity vector.
	b2Vec2 gravity(0.0f, -10.0f);

	// Construct a world object, which will hold and simulate the rigid bodies.
	b2World world(gravity);
  world.SetContinuousPhysics(false);

  InitWorld(&world);

	// Prepare for simulation. Typically we use a time step of 1/60 of a
	// second (60Hz) and 10 iterations. This provides a high quality simulation
	// in most game scenarios.
	float timeStep = 1.0f / 60.0f;
	int32 velocityIterations = 8;
	int32 positionIterations = 3;

  using std::chrono::high_resolution_clock;
	auto start = high_resolution_clock::now();
	
	// This is our little game loop.
	for (int32 i = 0; i < 400; ++i) {
		// Instruct the world to perform a single step of simulation.
		// It is generally best to keep the time step and iterations fixed.
		world.Step(timeStep, velocityIterations, positionIterations);
	}
	
	auto finish = high_resolution_clock::now();
	int64 time = (finish - start).count();
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

	return 0;
}
