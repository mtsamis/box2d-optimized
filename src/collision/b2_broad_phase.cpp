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

#include "box2d/b2_broad_phase.h"

b2BroadPhase::b2BroadPhase() {
	m_proxyCount = 0;
}

b2BroadPhase::~b2BroadPhase() {

}

void b2BroadPhase::Add(b2Fixture* fixture) {
	// TODO
	int32 proxyId = m_tree.CreateProxy(fixture->GetAABB(), fixture);
	++m_proxyCount;
}

bool b2BroadPhase::Remove(b2Fixture* fixture) {
	// TODO
	return false;
}

void b2BroadPhase::UpdateAll()
{
	m_tree.UpdateAll();
}

void b2BroadPhase::ShiftOrigin(const b2Vec2& newOrigin)
{
	m_tree.ShiftOrigin(newOrigin);
}

