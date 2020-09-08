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

#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <algorithm>
#include <chrono>

#include "box2d/box2d.h"
#include "benchmarks.h"

#define BENCH_ID "opt"

int main() {
  class : public b2Benchmarks {
  public:
    void run() {
      using std::chrono::high_resolution_clock;

      for (auto benchmark : benchmarks) {
        benchmark->InitBenchmark();
        std::cout << benchmark->name << std::endl;
            
        std::ofstream outfile;
        std::string name = benchmark->name + "-" + BENCH_ID + ".csv";
        std::replace(name.begin(), name.end(), ' ', '-');
        std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c){ return std::tolower(c); });
        
        outfile.open(name, std::ios::out | std::ios::trunc);
        
	      outfile << "size, time" << std::endl;
        
        for (int32 i = benchmark->startSize; i <= benchmark->endSize; i += benchmark->sizeInc) {
          b2World world(benchmark->gravity);
          world.SetContinuousPhysics(false);

          benchmark->InitWorld(&world, i);

	        int64 totalTime = 0;
            
	        for (int32 i = 0; i < benchmark->simulationSteps; ++i) {
            auto start = high_resolution_clock::now();

		        world.Step(benchmark->timeStep, benchmark->velocityIterations, benchmark->positionIterations);
		        benchmark->StepWorld(&world);

	          auto finish = high_resolution_clock::now();
        	  int64 time = (finish - start).count();
        	  totalTime += time;
        	}
	        
	        // fixture count | time
	        outfile << world.GetProxyCount() << ", " << (totalTime / 1000000) << std::endl;
	        std::cout << world.GetProxyCount() << ", " << (totalTime / 1000000.0f) << std::endl;
        }
        
        outfile.close();
      }
    }
  } b;
  
  b.run();
  
	return 0;
}
