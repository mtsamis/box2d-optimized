// MIT License

// Copyright (c) 2022 Manolis Tsamis

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

#include "test.h"
#include "benchmarks/benchmarks.h"

template<int I>
class b2BenchmarkTest : public Test
{
public:
  b2BenchmarkTest() {
    auto benchmark = benchmarks[I];
    
    m_world->SetGravity(benchmark->gravity);
    benchmark->InitWorld(m_world);
  }

  void Step(Settings& settings) override {
    auto benchmark = benchmarks[I];

    benchmark->BeforeWorldStep(m_world);
    Test::Step(settings);
    benchmark->AfterWorldStep(m_world);
  }

  static const char* GetName() {
    return strdup(benchmarks[I]->name.c_str());
  }

  static Test* Create() { return new b2BenchmarkTest<I>(); }
};

#define REGISTER_BENCHMARK(IDX) static int testIndex##IDX = RegisterTest("Benchmark Suite", b2BenchmarkTest<IDX>::GetName(), b2BenchmarkTest<IDX>::Create);

REGISTER_BENCHMARK(0);
REGISTER_BENCHMARK(1);
REGISTER_BENCHMARK(2);
REGISTER_BENCHMARK(3);
REGISTER_BENCHMARK(4);
REGISTER_BENCHMARK(5);
REGISTER_BENCHMARK(6);
REGISTER_BENCHMARK(7);
REGISTER_BENCHMARK(8);
REGISTER_BENCHMARK(9);
REGISTER_BENCHMARK(10);
REGISTER_BENCHMARK(11);
REGISTER_BENCHMARK(12);
REGISTER_BENCHMARK(13);