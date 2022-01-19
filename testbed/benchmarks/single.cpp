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
#include <algorithm>
#include <chrono>

#include "box2d/box2d.h"
#include "benchmarks.h"

int main() {
  using std::chrono::high_resolution_clock;

  for (int i = 0; i < benchmark_count; i++) {
    auto benchmark = benchmarks[i];
    int32 iterations = 1;
    int64 totalTime = 0;
    int64 maxTime = 0;
    
    for (int32 i = 0; i < iterations; ++i) {
      // Construct a world object, which will hold and simulate the rigid bodies.
      b2World world(benchmark->gravity);
      world.SetContinuousPhysics(false);

      benchmark->InitWorld(&world);

      int64 maxTime0 = 0;

      for (int32 i = 0; i < benchmark->simulationSteps; ++i) {
        auto start = high_resolution_clock::now();

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
  
  return 0;
}
