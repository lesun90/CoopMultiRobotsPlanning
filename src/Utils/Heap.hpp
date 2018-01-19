#ifndef MP_HEAP_HPP_
#define MP_HEAP_HPP_

#include "Utils/Map.hpp"
#include "Utils/Constants.hpp"
#include <vector>

namespace MP
{
    template <typename Key, typename LessFnData>
    class Heap
    {
    public:
        Heap(void)
	{
	}

	virtual ~Heap(void)
	{
	}

	typedef bool (*LessFn) (const Key , const Key , LessFnData);

	LessFn     m_lessFn;
	LessFnData m_lessFnData;

	bool IsEmpty(void) const
	{
	    return m_heap.empty();
	}

	int GetNrKeys(void) const
	{
	    return m_heap.size();
	}

	bool HasKey(const Key  key) const
	{
	    return m_map.find(key) != m_map.end();
	}

	int GetKeyPosition(const Key  key) const
	{
	    auto cur = m_map.find(key);
	    if(cur == m_map.end())
		return Constants::ID_UNDEFINED;
	    else
		return cur->second;
	}

  void Insert(const Key  key)
  {
    const int pos =  m_heap.size();
    m_heap.push_back(key);
    m_map.insert(std::make_pair(key, pos));
    PrecolateUp(pos);
  }

 	void Update(const Key  key)
	{
	    UpdateAtPosition(m_map.find(key)->second);
	}

 	void UpdateAtPosition(const int pos)
	{
	    PrecolateUp(pos);
	    PrecolateDown(pos);
	}

 	Key GetTop(void) const
	{
	    return m_heap[0];
	}

	Key GetKeyAtPosition(const int pos) const
	{
	    return m_heap[pos];
	}

	Key RemoveTop(void)
	{
	    const Key key = m_heap[0];
	    RemoveAtPosition(0);
	    return key;
	}

	void Remove(const Key  key)
	{
	    RemoveAtPosition(m_map.find(key)->second);
	}

	void RemoveAtPosition(const int pos)
	{
	    const Key key  = m_heap[pos];
	    const int size = m_heap.size();

	    m_heap[pos] = m_heap[size - 1];
	    m_map.find(m_heap[pos])->second = pos;
	    m_heap.pop_back();
	    m_map.erase(key);
	    if(pos < size - 1)
		UpdateAtPosition(pos);
	}

	void Clear(void)
	{
	    m_heap.clear();
	    m_map.clear();
	}

	void Sort(Key * const keys, const int size)
	{
	    Clear();
	    for(int i = 0; i < size; i++)
		m_heap.push_back(keys[i]);
	    for(int i = (size / 2) - 1; i >= 0; i--)
		PrecolateDown(i);
	    for(int i = 0; i < size; i++)
		keys[i] = RemoveTop();
	}

	void Print(FILE *out)
	{
	    for(int i = 0; i < m_heap.size(); ++i)
		printf("heap %d: %d\n", i, m_heap[i]);

	}

	bool Test(void)
	{
	    for(int i = 1; i < m_heap.size(); ++i)
	    {
		int child = i;
		int parent= (child - 1) >> 1;
		if(m_lessFn(m_heap[parent], m_heap[child], m_lessFnData) == false)
		{
		    printf("heap property violated %d %d with keys %d %d\n",
			   child, parent, m_heap[child], m_heap[parent]);
		    return false;
		}
	    }

	    return true;

	}


	//protected:
	void PrecolateDown(const int pos)
	{
	    const int size   = m_heap.size();
	    const Key key    = m_heap[pos];
	    int       schild = (pos + 1) << 1;
	    int       parent = pos;

	    for(;schild < size; parent = schild, schild = ((schild + 1) << 1))
	    {
		if(m_lessFn(m_heap[schild - 1], m_heap[schild], m_lessFnData))
		    --schild;
		m_heap[parent] = m_heap[schild];
		m_map.find(m_heap[parent])->second = parent;
	    }
	    if(schild == size)
	    {
		m_heap[parent] = m_heap[schild - 1];
		m_map.find(m_heap[parent])->second = parent;
		parent         = schild -1;
	    }
	    if(parent != pos)
	    {
		m_heap[parent] = key;
		m_map.find(key)->second = parent;
		PrecolateUp(parent);
	    }
	}

	void PrecolateUp(const int pos)
	{
	    const Key key    = m_heap[pos];
	    int       child  = pos;
	    int       parent = (child - 1) >> 1;

	    for(; child > 0 && m_lessFn(key, m_heap[parent], m_lessFnData);
		child = parent, parent = (child - 1) >> 1)
	    {
		m_heap[child] = m_heap[parent];
		m_map.find(m_heap[child])->second = child;
	    }
	    if(child != pos)
	    {
		m_heap[child] = key;
		m_map.find(key)->second = child;
	    }
	}



    protected:
	UseMap(Key, int)  m_map;
	std::vector<Key>  m_heap;

    };
}

#endif
