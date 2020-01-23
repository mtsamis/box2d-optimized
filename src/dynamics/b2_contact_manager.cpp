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

#include "box2d/b2_body.h"
#include "box2d/b2_contact.h"
#include "box2d/b2_contact_manager.h"
#include "box2d/b2_fixture.h"
#include "box2d/b2_world_callbacks.h"

b2ContactFilter b2_defaultFilter;
b2ContactListener b2_defaultListener;

b2ContactManager::b2ContactManager()
{
	m_contactList = nullptr;
	m_contactCount = 0;
	m_contactFilter = &b2_defaultFilter;
	m_contactListener = &b2_defaultListener;
	m_allocator = nullptr;
}

void b2ContactManager::Destroy(b2Contact* c)
{
	b2Fixture* fixtureA = c->GetFixtureA();
	b2Fixture* fixtureB = c->GetFixtureB();
	b2Body* bodyA = fixtureA->GetBody();
	b2Body* bodyB = fixtureB->GetBody();

	if (m_contactListener && c->IsTouching()) {
		m_contactListener->EndContact(c);
	}

	// Remove from the world.
	if (c->m_prev) {
		c->m_prev->m_next = c->m_next;
	}

	if (c->m_next) {
		c->m_next->m_prev = c->m_prev;
	}

	if (c == m_contactList) {
		m_contactList = c->m_next;
	}

	// Remove from body 1
	if (c->m_nodeA.prev) {
		c->m_nodeA.prev->next = c->m_nodeA.next;
	}

	if (c->m_nodeA.next) {
		c->m_nodeA.next->prev = c->m_nodeA.prev;
	}

	if (&c->m_nodeA == bodyA->m_contactList) {
		bodyA->m_contactList = c->m_nodeA.next;
	}

	// Remove from body 2
	if (c->m_nodeB.prev) {
		c->m_nodeB.prev->next = c->m_nodeB.next;
	}

	if (c->m_nodeB.next) {
		c->m_nodeB.next->prev = c->m_nodeB.prev;
	}

	if (&c->m_nodeB == bodyB->m_contactList) {
		bodyB->m_contactList = c->m_nodeB.next;
	}

	// Call the factory.
	b2Contact::Destroy(c, m_allocator);
	--m_contactCount;
}

#include<iostream>
// This is the top level collision call for the time step. Here
// all the narrow phase collision is processed for the world
// contact list.
void b2ContactManager::Collide() {
	// Destroy old contacts that did not persist
	while (m_destroyList) {
		//TODO since this list is detached we can delete the nodes without removing them from the list 
		b2Contact* cNuke = m_destroyList;
		m_destroyList = m_destroyList->GetNext();
		Destroy(cNuke);
	}
	
	// Update awake contacts.
	b2Contact* c = m_contactList;
	while (c) {
		b2Fixture* fixtureA = c->GetFixtureA();
		b2Fixture* fixtureB = c->GetFixtureB();
		b2Body* bodyA = fixtureA->GetBody();
		b2Body* bodyB = fixtureB->GetBody();
		 
		// Is this contact flagged for filtering?
		if (c->m_flags & b2Contact::e_filterFlag) {
			// Should these bodies collide?
			if (bodyB->ShouldCollide(bodyA) == false) {
				b2Contact* cNuke = c;
				c = cNuke->GetNext();
				Destroy(cNuke);
				continue;
			}

			// Check user filtering.
			if (m_contactFilter && m_contactFilter->ShouldCollide(fixtureA, fixtureB) == false) {
				b2Contact* cNuke = c;
				c = cNuke->GetNext();
				Destroy(cNuke);
				continue;
			}
			
			// Clear the filtering flag.
			c->m_flags &= ~b2Contact::e_filterFlag;
		}

		bool activeA = bodyA->IsAwake() && bodyA->m_type != b2_staticBody;
		bool activeB = bodyB->IsAwake() && bodyB->m_type != b2_staticBody;

		// At least one body must be awake and it must be dynamic or kinematic.
		if (activeA == false && activeB == false) {
			c = c->GetNext();
			continue;
		}

		/* contacts that made it to the contact list have persisted
		//int32 proxyIdA = fixtureA->m_proxies[indexA].proxyId;
		//int32 proxyIdB = fixtureB->m_proxies[indexB].proxyId;
		bool overlap = b2TestOverlap(fixtureA->GetAABB(), fixtureB->GetAABB());

		// Here we destroy contacts that cease to overlap in the broad-phase.
		if (overlap == false)
		{
			b2Contact* cNuke = c;
			c = cNuke->GetNext();
			Destroy(cNuke);
			continue;
		}
		//*/
		
		// The contact persists.
		c->Update(m_contactListener);
		c = c->GetNext();
	}
}

void b2ContactManager::FindNewContacts() {
	m_destroyList = m_contactList;
	
	m_broadPhase.UpdateAll();
	m_broadPhase.QueryAll(this);
	
	// Detach from the list of contacts that persisted
	if (m_destroyList) {
		if (m_destroyList->m_prev) {
			m_destroyList->m_prev->m_next = nullptr;
			m_destroyList->m_prev = nullptr;
		} else {
			m_contactList = nullptr;
		}
	}
}

void b2ContactManager::QueryCallback(b2Fixture* fixtureA, b2Fixture* fixtureB)
{
	b2Body* bodyA = fixtureA->GetBody();
	b2Body* bodyB = fixtureB->GetBody();
	
	// Are the fixtures on the same body?
	if (bodyA == bodyB) {
		return;
	}
	
	// Does a contact already exist?
	b2ContactEdge* edge = bodyB->GetContactList();
	while (edge) {
		if (edge->other == bodyA) {
			b2Fixture* fA = edge->contact->GetFixtureA();
			b2Fixture* fB = edge->contact->GetFixtureB();
			
			if ((fA == fixtureA && fB == fixtureB) || (fA == fixtureB && fB == fixtureA)) {
				// A contact already exists.
				
				b2Contact* cmov = edge->contact;
				
				if (cmov == m_destroyList) {
					m_destroyList = m_destroyList->m_next;
				}
				
				if (cmov->m_prev) {
					cmov->m_prev->m_next = cmov->m_next;
				}

				if (cmov->m_next) {
					cmov->m_next->m_prev = cmov->m_prev;
				}
				
				if (cmov == m_contactList)
				{
					m_contactList = cmov->m_next;
				}

				cmov->m_prev = nullptr;
				cmov->m_next = m_contactList;
				if (m_contactList != nullptr)
				{
					m_contactList->m_prev = cmov;
				}
				m_contactList = cmov;
				
				return;
			}
		}

		edge = edge->next;
	}

	// Does a joint override collision? Is at least one body dynamic?
	if (bodyB->ShouldCollide(bodyA) == false) {
		return;
	}

	// Check user filtering.
	if (m_contactFilter && m_contactFilter->ShouldCollide(fixtureA, fixtureB) == false) {
		return;
	}

	// Call the factory.
	b2Contact* c = b2Contact::Create(fixtureA, fixtureB, m_allocator);
	if (c == nullptr) {
		return;
	}

	// Contact creation may swap fixtures.
	fixtureA = c->GetFixtureA();
	fixtureB = c->GetFixtureB();
	bodyA = fixtureA->GetBody();
	bodyB = fixtureB->GetBody();

	// Insert into the world.
	c->m_prev = nullptr;
	c->m_next = m_contactList;
	if (m_contactList != nullptr)
	{
		m_contactList->m_prev = c;
	}
	m_contactList = c;

	// Connect to island graph.

	// Connect to body A
	c->m_nodeA.contact = c;
	c->m_nodeA.other = bodyB;

	c->m_nodeA.prev = nullptr;
	c->m_nodeA.next = bodyA->m_contactList;
	if (bodyA->m_contactList != nullptr)
	{
		bodyA->m_contactList->prev = &c->m_nodeA;
	}
	bodyA->m_contactList = &c->m_nodeA;

	// Connect to body B
	c->m_nodeB.contact = c;
	c->m_nodeB.other = bodyA;

	c->m_nodeB.prev = nullptr;
	c->m_nodeB.next = bodyB->m_contactList;
	if (bodyB->m_contactList != nullptr)
	{
		bodyB->m_contactList->prev = &c->m_nodeB;
	}
	bodyB->m_contactList = &c->m_nodeB;

	// Wake up the bodies
	if (fixtureA->IsSensor() == false && fixtureB->IsSensor() == false)
	{
		bodyA->SetAwake(true);
		bodyB->SetAwake(true);
	}

	++m_contactCount;
}
