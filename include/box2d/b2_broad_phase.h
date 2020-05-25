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
	b2TreeNode* parent;
	b2Fixture* fixture;
	b2AABB aabb;
	
	bool IsLeaf() const {
	  return left == nullptr;
	}
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
	b2BroadPhase() : b2BroadPhase(32) {}
	
	/// Constructing the tree initializes the node pool.
	b2BroadPhase(int32 initialCapacity);

	/// Destroy the tree, freeing the node pool.
	~b2BroadPhase();

	bool Add(b2Fixture* fixture);

	bool Update(b2Fixture* fixture);
	bool UpdateNoRebuild(b2Fixture* fixture);

	bool Remove(b2Fixture* fixture);

	void UpdateAll();
	
	template <typename UnaryPredicate>
	void UpdateAll(UnaryPredicate predicate);
	
	void RemoveAll();
	
	template <typename UnaryPredicate>
	void RemoveAll(UnaryPredicate predicate);
	
	/// Query an AABB for overlapping proxies. The callback class
	/// is called for each proxy that overlaps the supplied AABB.
	template <typename T>
	void Query(T* callback, const b2AABB& aabb);
	
	template <typename T>
	void QueryAll(T* callback);
	
	template <typename T, typename UnaryPredicate>
	void QueryAll(T* callback, UnaryPredicate predicate);
	
	template <typename T>
	void UpdateAndQuery(T* callback);

	/// Ray-cast against the proxies in the tree. This relies on the callback
	/// to perform a exact ray-cast in the case were the proxy contains a shape.
	/// The callback also performs the any collision filtering. This has performance
	/// roughly equal to k * log(n), where k is the number of collisions and n is the
	/// number of proxies in the tree.
	/// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
	/// @param callback a callback class that is called for each proxy that is hit by the ray.
	template <typename T>
	void RayCast(T* callback, const b2RayCastInput& input);
	
	/// Get the height of the embedded tree.
	int32 GetTreeHeight() const;

	int32 GetCount() const;

	/// Shift the world origin. Useful for large worlds.
	/// The shift formula is: position -= newOrigin
	/// @param newOrigin the new origin with respect to the old origin
	void ShiftOrigin(const b2Vec2& newOrigin);

private:
  void EnsureBuiltTree();
	int32 ComputeHeight(b2TreeNode* node) const;
	b2Vec2 GetCenter2(const b2AABB& aabb);

	void Build();
  b2TreeNode* Build(int32 start, int32 end);

	template <typename T>
	b2TreeNode* BuildAndQuery(T* callback, int32 start, int32 end, b2TreeNode** collision, int32 ccount);

	template <typename T>
	void QueryAABB(T* callback, const b2AABB& aabb, b2TreeNode* root) const;

  template <typename T>
  void QueryAll(T* callback, const b2TreeNode* root, b2TreeNode** collision, int ccount) const;

	b2TreeNode** m_links;
	b2TreeNode* m_nodes;
	b2TreeNode* m_treeAllocator;

	b2TreeNode* m_root;
	b2TreeNode m_treeMergeNode;

	b2TreeNode* m_rootDynamic;
	b2TreeNode* m_rootStatic;

	b2IntHashTable m_map;

	int32 m_capacity;
	int32 m_count;
	bool m_needsRebuild;
};

inline void b2BroadPhase::EnsureBuiltTree() {
  if (m_needsRebuild) {
    Build();
  }
}

inline int32 b2BroadPhase::GetCount() const {
	return m_count;
}

inline b2Vec2 b2BroadPhase::GetCenter2(const b2AABB& aabb) {
	b2Vec2 ret(aabb.lowerBound.x + aabb.upperBound.x, aabb.lowerBound.y + aabb.upperBound.y);
	return ret;
}

template <typename T>
void b2BroadPhase::UpdateAndQuery(T* callback) {
	b2Assert(callback);

	// (m_count == 0) -> root = rootStatic = rootDynamic = nullptr
  int32 staticGroup = 0;

	for (int32 i = 0; i < m_count; i++) {
		m_links[i] = &m_nodes[i];
		b2Fixture* f = m_links[i]->fixture;

		if (f->GetBody()->GetType() == b2_staticBody) {
		  b2Swap(m_links[i], m_links[staticGroup]);
		  staticGroup++;
		} else {
		  f->UpdateAABB();
			m_links[i]->aabb = f->GetAABB();
		}
	}

  if (staticGroup <= m_count / 8) {
    staticGroup = 0;
    m_rootStatic = nullptr;
  } else if (m_rootStatic == nullptr) {
    m_treeAllocator = m_nodes + m_capacity;
	  m_rootStatic = Build(0, staticGroup);
  }

	b2TreeNode* temp[m_count];
  m_treeAllocator = m_nodes + m_capacity + staticGroup;
	m_rootDynamic = BuildAndQuery(callback, staticGroup, m_count, temp, 0);

  if (m_rootStatic != nullptr) {
    int32 staticCollisionCount = 0;

    for (int32 i = staticGroup; i < m_count; i++) {
      if (b2TestOverlap(m_rootStatic->aabb, m_links[i]->aabb)) {
        temp[staticCollisionCount++] = m_links[i];
      }
    }

    QueryAll(callback, m_rootStatic, temp, staticCollisionCount);
  }

  if (m_rootDynamic == nullptr) {
    m_root = m_rootStatic;
  } else if (m_rootStatic == nullptr) {
    m_root = m_rootDynamic;
  } else {
    m_root = &m_treeMergeNode;
    m_root->aabb.Combine(m_rootStatic->aabb, m_rootDynamic->aabb);
    m_root->left = m_rootStatic;
    m_root->right = m_rootDynamic;
    m_rootStatic->parent = m_root;
    m_rootDynamic->parent = m_root;
  }

  m_root->parent = nullptr;
	m_needsRebuild = false;
}

template <typename T>
b2TreeNode* b2BroadPhase::BuildAndQuery(T* callback, int32 start, int32 end, b2TreeNode** collision, int32 ccount) {
	int count = end - start;

  if (count == 0) {
    return nullptr;
  } else if (count == 1) {
    for (int32 i = 0; i < ccount; ++i) {
		  callback->QueryCallback(m_links[start]->fixture, collision[i]->fixture);
		}

		return m_links[start];
	} else if (count == 2) {
	  // Although the count == 1 case is enough for the algorithm to work correctly this
	  // extra code for the common count == 2 case helps improve performance
	  b2TreeNode* n1 = m_links[start];
	  b2TreeNode* n2 = m_links[start + 1];

	  for (int32 i = 0; i < ccount; ++i) {
	    if (b2TestOverlap(n1->aabb, collision[i]->aabb)) {
        callback->QueryCallback(n1->fixture, collision[i]->fixture);
	    }

	    if (b2TestOverlap(n2->aabb, collision[i]->aabb)) {
    	  callback->QueryCallback(n2->fixture, collision[i]->fixture);
	    }
    }
    
		if (b2TestOverlap(n1->aabb, n2->aabb)) {
      callback->QueryCallback(n1->fixture, n2->fixture);
    }

	  b2TreeNode* cur = m_treeAllocator++;
	  cur->aabb.Combine(n1->aabb, n2->aabb);
    cur->left = n1;
	  cur->right = n2;
    n1->parent = cur;
    n2->parent = cur;

		return cur;
	} else {
		b2Vec2 c = GetCenter2(m_links[start]->aabb);
		float minx = c.x, maxx = minx;
		float miny = c.y, maxy = miny;

		for (int32 i = start + 1; i < end; i++) {
			c = GetCenter2(m_links[i]->aabb);

			minx = b2Min(minx, c.x);
			miny = b2Min(miny, c.y);
			maxx = b2Max(maxx, c.x);
			maxy = b2Max(maxy, c.y);
		}

    int32 group0 = start;
    b2AABB leftAABB, rightAABB;

    if (b2Abs(maxx - minx) < b2_epsilon && b2Abs(maxy - miny) < b2_epsilon) {
      // If all the centers coincide then we have a degenerate O(n^2) collision case
      // This is quite bad for the physics pipeline, but we can help with this special handling
		  for (int32 i = start; i < end; i++) {
		    for (int32 j = i + 1; j < end; ++j) {
			    callback->QueryCallback(m_links[i]->fixture, m_links[j]->fixture);
		    }

		    for (int32 i = 0; i < ccount; ++i) {
			    callback->QueryCallback(m_links[i]->fixture, collision[i]->fixture);
		    }
		  }
      
		  // TODO create a valid subtree here
		  return m_links[start];
    } else {
      bool splitX = (maxx - minx) > (maxy - miny);
		  float mid = splitX? (minx + maxx) / 2 : (miny + maxy) / 2;

		  leftAABB.lowerBound = {b2_maxFloat, b2_maxFloat};
		  leftAABB.upperBound = {-b2_maxFloat, -b2_maxFloat};
		  rightAABB = leftAABB;

      for (int32 i = start; i < end; i++) {
			  b2AABB aabb = m_links[i]->aabb;
			  float nodeMid = splitX? (aabb.lowerBound.x + aabb.upperBound.x) : (aabb.lowerBound.y + aabb.upperBound.y);

			  if (nodeMid < mid) {
				  b2Swap(m_links[i], m_links[group0++]);
				  leftAABB.Combine(aabb);
			  } else {
				  rightAABB.Combine(aabb);
			  }
		  }

		  // prevent a degenrate tree with O(n) height
		  // while this will degrade tree quality but linear height is even worse
		  // Because the detection algorithm may require O(n*height) stack memory
		  // in the worst case this prevents some potential stack overflow errors
		  int32 lim = b2Max(count / 16, 1);
		  if (group0 < start + lim || group0 > end - lim) {
			  if (group0 < start + lim) {
			    group0 = start + lim;
			  } else {
  			  group0 = end - lim;
			  }

		    leftAABB = m_links[start]->aabb;
		    rightAABB = m_links[end - 1]->aabb;

		    for (int32 i = start + 1; i < group0; i++) {
			    leftAABB.Combine(m_links[i]->aabb);
			  }

			  for (int32 i = group0; i < end - 1; i++) {
			    rightAABB.Combine(m_links[i]->aabb);
			  }
		  }
    }

    int32 leftCapacity = b2Min(ccount + count, m_count);

    // ccount + count > 0 because count > 0
    // m_count > 0
    b2Assert(leftCapacity > 0);

		b2TreeNode* left[leftCapacity];
	  b2TreeNode** right = collision;
	  int32 leftCount = 0;
	  int32 rightCount = 0;

    for (int32 i = 0; i < ccount; ++i) {
		  if (b2TestOverlap(leftAABB, collision[i]->aabb)) {
		   left[leftCount++] = collision[i];
		  }

		  if (b2TestOverlap(rightAABB, collision[i]->aabb)) {
		   right[rightCount++] = collision[i];
		  }
	  }

    if (b2TestOverlap(leftAABB, rightAABB)) {
	    if (group0 - start < end - group0) {
		    for (int32 i = start; i < group0; ++i) {
		      if (b2TestOverlap(rightAABB, m_links[i]->aabb)) {
		        right[rightCount++] = m_links[i];
		      }
		    }
	    } else {
		    for (int32 i = group0; i < end; ++i) {
			    if (b2TestOverlap(leftAABB, m_links[i]->aabb)) {
			      left[leftCount++] = m_links[i];
			    }
		    }
	    }
    }

	  b2TreeNode* cur = m_treeAllocator++;
	  cur->aabb.Combine(leftAABB, rightAABB);
	  cur->left = BuildAndQuery(callback, start, group0, left, leftCount);
	  cur->right = BuildAndQuery(callback, group0, end, right, rightCount);
    cur->left->parent = cur;
    cur->right->parent = cur;

	  return cur;
	}
}

inline void b2BroadPhase::RemoveAll() {
	// TODO efficient
	RemoveAll([](b2Fixture* fixture) { return true; });
}

template <typename UnaryPredicate>
void b2BroadPhase::RemoveAll(UnaryPredicate predicate) {
	for (int32 i = 0; i < m_count; i++) {
		b2Fixture* f = m_nodes[i].fixture;
		
		if (predicate(f)) {
			// TODO efficient traversal of the map
			Remove(f);
		}
	}

  m_needsRebuild = true;
}

inline void b2BroadPhase::UpdateAll() {
	UpdateAll([](b2Fixture* fixture) { return true; });
}

template <typename UnaryPredicate>
inline void b2BroadPhase::UpdateAll(UnaryPredicate predicate) {
	for (int32 i = 0; i < m_count; i++) {
		b2Fixture* f = m_nodes[i].fixture;
		
		if (predicate(f)) {
			f->UpdateAABB();
			m_nodes[i].aabb = f->GetAABB();
		}
	}

  m_needsRebuild = true;
}

template <typename T>
inline void b2BroadPhase::QueryAll(T* callback) {
	QueryAll(callback, [](b2Fixture* fixture) { return true; });
}

template <typename T, typename UnaryPredicate>
inline void b2BroadPhase::QueryAll(T* callback, UnaryPredicate predicate) {
	if (m_count == 0) {
		return;
	}

	EnsureBuiltTree();

	b2TreeNode* temp[m_count];
	int ccount = 0;
	
	for (int32 i = 0; i < m_count; i++) {
		b2Fixture* f = m_nodes[i].fixture;
		
		if (predicate(f)) {
			temp[ccount] = &m_nodes[i];
			++ccount;
		}
	}
	
	QueryAll(callback, m_root, temp, ccount);
}

template <typename T>
inline void b2BroadPhase::QueryAll(T* callback, const b2TreeNode* root, b2TreeNode** collision, int ccount) const {
	if (root->IsLeaf()) {
    for (int32 i = 0; i < ccount; ++i) {
			callback->QueryCallback(root->fixture, collision[i]->fixture);
		}
		
		return;
  }
  
  int32 leftCount = ccount;
  
  for (int32 i = 0; i < leftCount; ++i) {
		if (!b2TestOverlap(root->left->aabb, collision[i]->aabb)) {
		  --leftCount;
		  b2Swap(collision[i], collision[leftCount]);
		  --i;
		}
	}
	
	QueryAll(callback, root->left, collision, leftCount);
	
  int32 rightCount = ccount;
  
  for (int32 i = 0; i < rightCount; ++i) {
		if (!b2TestOverlap(root->right->aabb, collision[i]->aabb)) {
		  --rightCount;
		  b2Swap(collision[i], collision[rightCount]);
		  --i;
		}
	}
	
	QueryAll(callback, root->right, collision, rightCount);
}

template <typename T>
inline void b2BroadPhase::Query(T* callback, const b2AABB& aabb) {
	if (m_count > 0) {
		EnsureBuiltTree();
		QueryAABB(callback, aabb, m_root);
	}
}

template <typename T>
void b2BroadPhase::QueryAABB(T* callback, const b2AABB& aabb, b2TreeNode* root) const {
	if (root->IsLeaf()) {
		callback->QueryCallback(root->fixture);
	} else {
		if (b2TestOverlap(aabb, root->left->aabb)) {
			QueryAABB(callback, aabb, root->left);
		}

		if (b2TestOverlap(aabb, root->right->aabb)) {
			QueryAABB(callback, aabb, root->right);
		}
	}
}

template <typename T>
inline void b2BroadPhase::RayCast(T* callback, const b2RayCastInput& input) {
	EnsureBuiltTree();

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

			float value = callback->RayCastCallback(subInput, node->fixture);

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
