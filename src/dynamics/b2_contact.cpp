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

#include "box2d/b2_contact.h"
#include "box2d/b2_block_allocator.h"
#include "box2d/b2_body.h"
#include "box2d/b2_collision.h"
#include "box2d/b2_fixture.h"
#include "box2d/b2_shape.h"
#include "box2d/b2_time_of_impact.h"
#include "box2d/b2_world.h"

#define b2CreateCollisionFuncWrapper(FUNC_NAME, TYPE_A, TYPE_B)                        \
void FUNC_NAME##Wrapper (b2Manifold* manifold,                                         \
                        const b2Shape* shapeA, const b2Transform& xfA,                 \
                        const b2Shape* shapeB, const b2Transform& xfB) {               \
	FUNC_NAME(manifold, (const TYPE_A*) (shapeA), xfA, (const TYPE_B*) (shapeB), xfB);   \
}

// Create b2EvaluateFunction wrapper functions for the collision functions
b2CreateCollisionFuncWrapper(b2CollideCircles, b2CircleShape, b2CircleShape)
b2CreateCollisionFuncWrapper(b2CollidePolygonAndCircle, b2PolygonShape, b2CircleShape)
b2CreateCollisionFuncWrapper(b2CollidePolygons, b2PolygonShape, b2PolygonShape)
b2CreateCollisionFuncWrapper(b2CollideEdgeAndCircle, b2EdgeShape, b2CircleShape)
b2CreateCollisionFuncWrapper(b2CollideEdgeAndPolygon, b2EdgeShape, b2PolygonShape)

b2EvaluateFunction* b2Contact::functions[b2Shape::e_typeCount][b2Shape::e_typeCount];

bool b2Contact::InitializeRegisters() {
	functions[b2Shape::e_circle][b2Shape::e_circle] = &b2CollideCirclesWrapper;
	functions[b2Shape::e_polygon][b2Shape::e_circle] = &b2CollidePolygonAndCircleWrapper;
	functions[b2Shape::e_polygon][b2Shape::e_polygon] = &b2CollidePolygonsWrapper;
	functions[b2Shape::e_edge][b2Shape::e_circle] = &b2CollideEdgeAndCircleWrapper;
	functions[b2Shape::e_edge][b2Shape::e_polygon] = &b2CollideEdgeAndPolygonWrapper;

	return true;
}

b2Contact* b2Contact::Create(b2Fixture* fixtureA, b2Fixture* fixtureB, b2BlockAllocator* allocator) {	
	// lambda to initialize once
	static bool initialize = []() { return InitializeRegisters(); } ();

	b2Shape::Type type1 = fixtureA->GetType();
	b2Shape::Type type2 = fixtureB->GetType();

	b2Assert(0 <= type1 && type1 < b2Shape::e_typeCount);
	b2Assert(0 <= type2 && type2 < b2Shape::e_typeCount);
	
	if (functions[type1][type2] != nullptr) {
		void* mem = allocator->Allocate(sizeof(b2Contact));
		return new (mem) b2Contact(fixtureA, fixtureB, functions[type1][type2]);
	} else if (functions[type2][type1] != nullptr) {
		void* mem = allocator->Allocate(sizeof(b2Contact));
		return new (mem) b2Contact(fixtureB, fixtureA, functions[type2][type1]);
	} else {
		return nullptr;
	}
}

void b2Contact::Destroy(b2Contact* contact, b2BlockAllocator* allocator) {
	b2Fixture* fixtureA = contact->m_fixtureA;
	b2Fixture* fixtureB = contact->m_fixtureB;

	if (contact->m_manifold.pointCount > 0 &&
		fixtureA->IsSensor() == false &&
		fixtureB->IsSensor() == false)
	{
		fixtureA->GetBody()->SetAwake(true);
		fixtureB->GetBody()->SetAwake(true);
	}

	allocator->Free(contact, sizeof(b2Contact));
}

b2Contact::b2Contact(b2Fixture* fA, b2Fixture* fB, b2EvaluateFunction* evaluateFunction) {
	m_flags = e_enabledFlag;

	m_fixtureA = fA;
	m_fixtureB = fB;
	m_evaluateFunction = evaluateFunction;

	m_manifold.pointCount = 0;

	m_prev = nullptr;
	m_next = nullptr;

	m_nodeA.contact = nullptr;
	m_nodeA.prev = nullptr;
	m_nodeA.next = nullptr;
	m_nodeA.other = nullptr;

	m_nodeB.contact = nullptr;
	m_nodeB.prev = nullptr;
	m_nodeB.next = nullptr;
	m_nodeB.other = nullptr;

	m_toiCount = 0;

	m_friction = b2MixFriction(m_fixtureA->m_friction, m_fixtureB->m_friction);
	m_restitution = b2MixRestitution(m_fixtureA->m_restitution, m_fixtureB->m_restitution);

	m_tangentSpeed = 0.0f;
}

// Update the contact manifold and touching status.
// Note: do not assume the fixture AABBs are overlapping or are valid.
void b2Contact::Update(b2ContactListener* listener) {
	b2Manifold oldManifold = m_manifold;

	// Re-enable this contact.
	m_flags |= e_enabledFlag;

	bool touching = false;
	bool wasTouching = IsTouching();

	bool sensorA = m_fixtureA->IsSensor();
	bool sensorB = m_fixtureB->IsSensor();
	bool sensor = sensorA || sensorB;

	b2Body* bodyA = m_fixtureA->GetBody();
	b2Body* bodyB = m_fixtureB->GetBody();
	const b2Transform& xfA = bodyA->GetTransform();
	const b2Transform& xfB = bodyB->GetTransform();

	// Is this contact a sensor?
	if (sensor) {
		const b2Shape* shapeA = m_fixtureA->GetShape();
		const b2Shape* shapeB = m_fixtureB->GetShape();
		touching = b2TestOverlap(shapeA, shapeB, xfA, xfB);

		// Sensors don't generate manifolds.
		m_manifold.pointCount = 0;
	} else {
		Evaluate(&m_manifold, xfA, xfB);
		touching = m_manifold.pointCount > 0;

		// Match old contact ids to new contact ids and copy the
		// stored impulses to warm start the solver.
		for (int32 i = 0; i < m_manifold.pointCount; ++i) {
			b2ManifoldPoint* mp2 = m_manifold.points + i;
			mp2->normalImpulse = 0.0f;
			mp2->tangentImpulse = 0.0f;
			b2ContactID id2 = mp2->id;

			for (int32 j = 0; j < oldManifold.pointCount; ++j) {
				b2ManifoldPoint* mp1 = oldManifold.points + j;

				if (mp1->id.key == id2.key) {
					mp2->normalImpulse = mp1->normalImpulse;
					mp2->tangentImpulse = mp1->tangentImpulse;
					break;
				}
			}
		}

		if (touching != wasTouching) {
			bodyA->SetAwake(true);
			bodyB->SetAwake(true);
		}
	}

	if (touching) {
		m_flags |= e_touchingFlag;
	} else {
		m_flags &= ~e_touchingFlag;
	}
	
	if (listener) {
		if (wasTouching == false && touching == true) {
			listener->BeginContact(this);
		}

		if (wasTouching == true && touching == false) {
			listener->EndContact(this);
		}

		if (sensor == false && touching == true) {
			listener->PreSolve(this, &oldManifold);
		}
	}
}
