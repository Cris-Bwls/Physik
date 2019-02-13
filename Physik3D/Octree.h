#pragma once

#include <vector>
#include <glm/ext.hpp>

using namespace glm;
using std::vector;

// forward declare so we can make a pointer in OctObject
template <class T>
class Octree;

struct OctCube
{
	vec3 min;
	vec3 max;
};

template <class T>
struct OctObject
{
	T data;
	OctCube volume;
	Octree<T>* parent;
};

template <class T>
class Octree
{
public:
	Octree(int density, OctCube const& bounds)
		: m_density(density), m_bounds(bounds)
	{
		m_divided = false;

		// this tree isn't dividable (divisible?) if it gets small enough
		// - stops an eventual stack overflow when trees get tiny
		const float dividableLimit = 0.2f;
		m_dividable = bounds.max.x - bounds.min.x >= dividableLimit &&
			bounds.max.y - bounds.min.y >= dividableLimit;
	}

	Octree(int density, vec3 const& min, vec3 const& max)
		: Octree(density, { min, max })
	{
	}

	~Octree()
	{
		if (m_divided)
			for (int i = 0; i < 8; ++i)
				delete m_children[i];

		for (int i = 0; i < m_objects.size(); ++i)
			delete m_objects[i];
		m_objects.clear();
	}

	/***
	 * @brief Puts an object into the tree, passing it into child trees if
	 *			possible
	 *
	 * @param object Object to put into the tree
	 * @param vol Bounding box of this object
	 */
	void insert(T object, OctCube const& vol)
	{
		if ((m_divided || m_objects.size() >= m_density)
			&& m_dividable)
		{
			if (!m_divided)
				split();

			// pass it to all relevant segments
			// (not just one, so large objects get added to every tree they
			//		touch)
			for (int i = 0; i < 8; ++i)
			{
				Octree<T>* child = m_children[i];
				if (child->intersects(vol))
					child->insert(object, vol);
			}
		}
		else
		{
			// not divided, so the object can go in this node

			// make an OctObject for this object
			auto obj = new OctObject<T>();
			obj->data = object;
			obj->volume = vol;
			obj->parent = this;

			// stick it into the array
			m_objects.push_back(obj);
		}
	}

	/***
	 * @brief Removes all objects and child trees from this tree
	 */
	void clear()
	{
		if (m_divided)
			for (int i = 0; i < 8; ++i)
				delete m_children[i];
		for (int i = 0; i < m_objects.size(); ++i)
			delete m_objects[i];
		m_objects.clear();

		m_divided = false;
	}

	/***
	 * @brief Checks of a bounding box intersects this tree
	 *
	 * @param vol Bounding box to check for intersection
	 */
	bool intersects(OctCube const& vol)
	{
		// if any of these are true, we're NOT colliding
		if (vol.min.y > m_bounds.max.y || vol.max.y < m_bounds.min.y ||
			vol.min.x > m_bounds.max.x || vol.max.x < m_bounds.min.x ||
			vol.min.z > m_bounds.max.z || vol.max.z < m_bounds.min.z)
			return false;

		return true;
	}

	/***
	 * @brief Gets all objects in all trees that intersect with a box
	 *
	 * @param range Bounding box to check for intersection
	 * @return Dynamic Array containing all objects within these trees
	 */
	vector<T> getInRange(OctCube const& range)
	{
		vector<T> result;
		getInRange(range, &result);
		return result;
	}

	/***
	 * @brief Gets all objects in all trees that intersect with a set of bounds
	 *
	 * @return Dynamic Array containing all objects within these trees
	 */
	vector<T> getInRange(vec3 const& min, vec3 const& max)
	{
		return getInRange({ min, max });
	}

	bool isDivided() { return m_divided; }
	Octree* getChild(int i) { return m_children[i]; }
	OctCube getBounds() { return m_bounds; }

private:
	// how many objects can be in this tree before it splits
	int m_density;
	// bounding box of this tree
	OctCube m_bounds;

	vector<OctObject<T>*> m_objects;

	// has this tree split
	bool m_divided;
	// is this tree large enough to split
	bool m_dividable;
	Octree* m_children[8];

	/***
	 * @brief Splits this tree into 8 equal-sized child trees
	 */
	void split()
	{
		// get the size of each box (just half as big as this)
		float depth = (m_bounds.max.z - m_bounds.min.z) / 2.0f;
		float width = (m_bounds.max.x - m_bounds.min.x) / 2.0f;
		float height = (m_bounds.max.y - m_bounds.min.y) / 2.0f;

		// index of child we're creating
		int childIndex = 0;
		for (int x = 0; x < 2; ++x)
		{
			for (int y = 0; y < 2; ++y)
			{
				for (int z = 0; z < 2; ++z)
				{
					OctCube bounds;

					bounds.min.x = m_bounds.min.x + width * x;
					bounds.min.y = m_bounds.min.y + height * y;
					bounds.min.z = m_bounds.min.z + depth * z;
							  
					bounds.max.x = bounds.min.x + width;
					bounds.max.y = bounds.min.y + height;
					bounds.max.z = bounds.min.z + depth;

					m_children[childIndex] = new Octree<T>(m_density, bounds);

					childIndex++;
				}
			}
		}
		m_divided = true;

		// move objects into children
		for (int i = 0; i < m_objects.getCount(); ++i)
		{
			// check each child tree to see if this object intersects
			for (int j = 0; j < 8; ++j)
			{
				Octree<T>* child = m_children[j];
				// and insert it if it does
				if (child->intersects(m_objects[i]->volume))
					child->insert(m_objects[i]->data, m_objects[i]->volume);
			}
			// we can delete this since we're creating new objects in insert
			delete m_objects[i];
		}
		// all the objects have (hopefully) been passed onto our children
		m_objects.clear();
	}

	/***
	 * @brief Recursively adds items into a list from child trees that
	 *			intersect with a box
	 *
	 * @param cube Box to check for intersections with
	 * @param list Pointer to a dynamic array to add objects to
	 */
	void getInRange(OctCube const& cube, vector<T>* list)
	{
		// if this doesn't intersect with this cube, none of our children would
		//	either, so we can leave
		if (!intersects(cube))
			return;

		if (m_divided)
		{
			// call this on all our children
			for (int i = 0; i < 8; ++i)
				m_children[i]->getInRange(cube, list);
		}
		else
		{
			// only add objects which aren't already in the list
			for (int i = 0; i < m_objects.count(); ++i)
			{
				bool wasInList = false;
				for (int j = 0; j < list->count(); ++j)
				{
					if (m_objects[i]->data == (*list)[j])
					{
						wasInList = true;
						break;
					}
				}
				if (!wasInList)
					list->add(m_objects[i]->data);
			}
		}
	}
};
