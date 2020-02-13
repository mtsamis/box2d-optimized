// TODO License

#include "box2d/b2_geometry.h"
#include "box2d/b2_edge_shape.h"

void CreateLinks(b2Body* b, b2FixtureDef* fd, const b2Vec2* vertices, int32 count, bool closed) {
	b2Assert(closed? count >= 3 : count >= 2);

	const b2Shape* oldShape = fd->shape;

	for (int32 i = 1; i < count; ++i) {
		// If the code crashes here, it means your vertices are too close together.
		b2Assert(b2DistanceSquared(vertices[i - 1], vertices[i]) > b2_linearSlop * b2_linearSlop);
		
		b2EdgeShape shape;
		shape.Set(vertices[i - 1], vertices[i]);
		
		if (i > 1) {
			shape.m_hasVertex0 = true;
			shape.m_vertex0 = vertices[i - 2];
		} else if (closed) {
			shape.m_hasVertex0 = true;
			shape.m_vertex0 = vertices[count - 1];
		}

		if (i < count - 1) {
			shape.m_hasVertex3 = true;
			shape.m_vertex3 = vertices[i + 1];
		} else if (closed) {
			shape.m_hasVertex3 = true;
			shape.m_vertex3 = vertices[0];
		}

		fd->shape = &shape;
		b->CreateFixture(fd);
	}
	
	if (closed) {
		b2EdgeShape shape;
		shape.Set(vertices[count - 1], vertices[0]);
		shape.m_hasVertex0 = true;
		shape.m_vertex0 = vertices[count - 2];
		shape.m_hasVertex3 = true;
		shape.m_vertex3 = vertices[1];

		fd->shape = &shape;
		b->CreateFixture(fd);
	}

	fd->shape = oldShape;
}

void b2CreateLoop(b2Body* b, b2FixtureDef* fd, const b2Vec2* vertices, int32 count) {
	CreateLinks(b, fd, vertices, count, true);
}

void b2CreateChain(b2Body* b, b2FixtureDef* fd, const b2Vec2* vertices, int32 count) {
	CreateLinks(b, fd, vertices, count, false);
}

void b2CreateLoop(b2Body* b, const b2Vec2* vertices, int32 count) {
	b2FixtureDef fd;
	CreateLinks(b, &fd, vertices, count, true);
}

void b2CreateChain(b2Body* b, const b2Vec2* vertices, int32 count) {
	b2FixtureDef fd;
	CreateLinks(b, &fd, vertices, count, false);
}
