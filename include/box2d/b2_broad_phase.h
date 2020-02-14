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

#ifndef B2_BROAD_PHASE_H
#define B2_BROAD_PHASE_H

#include "b2_collision.h"
#include "b2_fixture.h"
#include "b2_growable_stack.h"

struct b2TreeNode {
	b2TreeNode* left;
	b2TreeNode* right;
	void* userData;
	b2AABB aabb;
	
	bool IsLeaf() const { return left == nullptr; }
};

struct b2MapNode {
	uint32 key;
	int32 value;

	b2MapNode* next;
};

class b2IntHashTable {
protected:
	friend class b2BroadPhase;

	b2IntHashTable(int32 initialCapacity);
	
	~b2IntHashTable();
	
	void Grow();
	b2MapNode** Get(b2Fixture* fixture);
	
	b2MapNode** m_mapNodes;
	int32 m_mapCapacity;
};

class b2BroadPhase {
public:
	b2BroadPhase() : b2BroadPhase(32, 0.05f) {}
	
	/// Constructing the tree initializes the node pool.
	b2BroadPhase(int32 initialCapacity, float qualityFactor);

	/// Destroy the tree, freeing the node pool.
	~b2BroadPhase();
	
	bool Add(b2Fixture* fixture);
	bool Update(b2Fixture* fixture);
	bool Remove(b2Fixture* fixture);
	
	void UpdateAll();
	
	template <typename Visitor>
	void UpdateAll(Visitor visitor);
	
	void RemoveAll();
	
	template <typename UnaryPredicate>
	void RemoveAll(UnaryPredicate predicate);
	
	/// Query an AABB for overlapping proxies. The callback class
	/// is called for each proxy that overlaps the supplied AABB.
	template <typename T>
	void Query(T* callback, const b2AABB& aabb) const;
	
	template <typename T>
	void QueryId(T* callback, b2TreeNode* leaf) const;
	
	template <typename T>
	void QueryAll(T* callback) const;
	
	template <typename T, typename UnaryPredicate>
	void QueryAll(T* callback, UnaryPredicate predicate) const;
	
	template <typename T>
	void detect_impl(T* callback, const b2AABB& aabb, b2TreeNode* root) const;
	
	template <typename T>
	void detect_impl_id(T* callback, const b2TreeNode* leaf, b2TreeNode* root) const;
	
	/// Ray-cast against the proxies in the tree. This relies on the callback
	/// to perform a exact ray-cast in the case were the proxy contains a shape.
	/// The callback also performs the any collision filtering. This has performance
	/// roughly equal to k * log(n), where k is the number of collisions and n is the
	/// number of proxies in the tree.
	/// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
	/// @param callback a callback class that is called for each proxy that is hit by the ray.
	template <typename T>
	void RayCast(T* callback, const b2RayCastInput& input) const;
	
	/// Get the height of the embedded tree.
	int32 GetTreeHeight() const;
	
	/// Get the quality metric of the embedded tree.
	float GetTreeQuality() const;
	
	int32 GetCount() const;
	
	/// Shift the world origin. Useful for large worlds.
	/// The shift formula is: position -= newOrigin
	/// @param newOrigin the new origin with respect to the old origin
	void ShiftOrigin(const b2Vec2& newOrigin);
	
	void MarkRebuild();
	
private:
	int32 ComputeHeight(b2TreeNode* node) const;
	
	bool RebuildTree();
	b2TreeNode* RebuildTree(b2TreeNode* parent, int32 start, int32 end);
	
	template <typename Visitor>
	void RefreshTree(Visitor visitor, b2TreeNode* node);
	
	float m_currentQuality;
	float m_lastRebuildQuality;
	float m_qualityFactor;
	
	b2TreeNode** m_links;
	b2TreeNode* m_nodes;
	
	bool m_rebuildLinks;
	b2TreeNode* m_root;
	
	b2IntHashTable m_map;
	
	int32 m_capacity;
	int32 m_count;
};

inline void b2BroadPhase::MarkRebuild() {
	m_currentQuality = 1;
	m_lastRebuildQuality = 0;
}

inline float b2BroadPhase::GetTreeQuality() const {
	return (m_currentQuality == 0)? 0 : m_lastRebuildQuality / m_currentQuality;
}

inline int32 b2BroadPhase::GetCount() const {
	return m_count;
}

inline void b2BroadPhase::RemoveAll() {
	// TODO efficient
	RemoveAll([](b2Fixture* fixture) { return true; });
}

template <typename UnaryPredicate>
void b2BroadPhase::RemoveAll(UnaryPredicate predicate) {
	MarkRebuild();
	
	for (int32 i = 0; i < m_count; i++) {
		b2Fixture* f = ((b2Fixture*) m_nodes[i].userData);
		
		if (predicate(f)) {
			// TODO efficient traversal of the map
			Remove(f);
		}
	}
}

inline void b2BroadPhase::UpdateAll() {
	UpdateAll([](b2Fixture* fixture) { return true; });
}
	
template <typename Visitor>
inline void b2BroadPhase::UpdateAll(Visitor visitor) {
	if (m_count == 0) {
		return;
	}

	if (RebuildTree()) {
		m_currentQuality = 0;
		RefreshTree(visitor, m_root);	
		
		m_lastRebuildQuality = m_currentQuality;
	} else {
		m_currentQuality = 0;
		RefreshTree(visitor, m_root);
	}
}

template <typename Visitor>
inline void b2BroadPhase::RefreshTree(Visitor visitor, b2TreeNode* node) {
	if (node->IsLeaf()) {
		b2Fixture* f = ((b2Fixture*) node->userData);
		
		if (visitor(f)) {
			node->aabb = f->GetAABB();
		}
			
		return;
	}
	
	RefreshTree(visitor, node->left);
	RefreshTree(visitor, node->right);
	
	node->aabb = node->left->aabb;
	node->aabb.Combine(node->right->aabb);
	
	m_currentQuality += node->aabb.upperBound.x - node->aabb.lowerBound.x + node->aabb.upperBound.y - node->aabb.lowerBound.y;
}

template <typename T>
inline void b2BroadPhase::QueryAll(T* callback) const {
	QueryAll(callback, [](b2Fixture* fixture) { return true; });
}

template <typename T, typename UnaryPredicate>
inline void b2BroadPhase::QueryAll(T* callback, UnaryPredicate predicate) const {
	for (int32 i = 0; i < m_count; i++) {
		b2Fixture* f = ((b2Fixture*) m_nodes[i].userData);
		
		if (predicate(f)) {
			QueryId(callback, &m_nodes[i]);
		}
	}
}
	
template <typename T>
inline void b2BroadPhase::QueryId(T* callback, b2TreeNode* leaf) const {
	if (m_count > 0) {
		detect_impl_id(callback, leaf, m_root);
	}
}

template <typename T>
void b2BroadPhase::detect_impl_id(T* callback, const b2TreeNode* leaf, b2TreeNode* root) const {
	if (root->IsLeaf()) {
		callback->QueryCallback((b2Fixture*) leaf->userData, (b2Fixture*) root->userData);
	} else {
		if (b2TestOverlap(leaf->aabb, root->left->aabb)) detect_impl_id(callback, leaf, root->left);
		if (b2TestOverlap(leaf->aabb, root->right->aabb)) detect_impl_id(callback, leaf, root->right);
	}
}

template <typename T>
void b2BroadPhase::detect_impl(T* callback, const b2AABB& aabb, b2TreeNode* root) const {
	if (root->IsLeaf()) {
		callback->QueryCallback((b2Fixture*) root->userData);
	} else {
		if (b2TestOverlap(aabb, root->left->aabb)) detect_impl(callback, aabb, root->left);
		if (b2TestOverlap(aabb, root->right->aabb)) detect_impl(callback, aabb, root->right);
	}
}

template <typename T>
inline void b2BroadPhase::Query(T* callback, const b2AABB& aabb) const {
	if (m_count > 0) {
		detect_impl(callback, aabb, m_root);
	}
}

template <typename T>
inline void b2BroadPhase::RayCast(T* callback, const b2RayCastInput& input) const
{
	b2Vec2 p1 = input.p1;
	b2Vec2 p2 = input.p2;
	b2Vec2 r = p2 - p1;
	b2Assert(r.LengthSquared() > 0.0f);
	r.Normalize();

	// v is perpendicular to the segment.
	b2Vec2 v = b2Cross(1.0f, r);
	b2Vec2 abs_v = b2Abs(v);

	// Separating axis for segment (Gino, p80).
	// |dot(v, p1 - c)| > dot(|v|, h)

	float maxFraction = input.maxFraction;

	// Build a bounding box for the segment.
	b2AABB segmentAABB;
	{
		b2Vec2 t = p1 + maxFraction * (p2 - p1);
		segmentAABB.lowerBound = b2Min(p1, t);
		segmentAABB.upperBound = b2Max(p1, t);
	}

	b2GrowableStack<b2TreeNode*, 256> stack;
	stack.Push(m_root);

	while (stack.GetCount() > 0) {
		const b2TreeNode* node = stack.Pop();

		if (b2TestOverlap(node->aabb, segmentAABB) == false) {
			continue;
		}

		// Separating axis for segment (Gino, p80).
		// |dot(v, p1 - c)| > dot(|v|, h)
		b2Vec2 c = node->aabb.GetCenter();
		b2Vec2 h = node->aabb.GetExtents();
		float separation = b2Abs(b2Dot(v, p1 - c)) - b2Dot(abs_v, h);
		if (separation > 0.0f) {
			continue;
		}

		if (node->IsLeaf()) {
			b2RayCastInput subInput;
			subInput.p1 = input.p1;
			subInput.p2 = input.p2;
			subInput.maxFraction = maxFraction;

			float value = callback->RayCastCallback(subInput, (b2Fixture*) node->userData);

			if (value == 0.0f) {
				// The client has terminated the ray cast.
				return;
			}

			if (value > 0.0f) {
				// Update segment bounding box.
				maxFraction = value;
				b2Vec2 t = p1 + maxFraction * (p2 - p1);
				segmentAABB.lowerBound = b2Min(p1, t);
				segmentAABB.upperBound = b2Max(p1, t);
			}
		} else {
			stack.Push(node->left);
			stack.Push(node->right);
		}
	}
}

#endif
