#ifndef __PRIORITIZED_QUEUE_H__
#define __PRIORITIZED_QUEUE_H__
#include <map>

// This implementation of queue interface makes quick access to ordered by priority elements of the queue
template<typename PriorityT, typename ValueT>
struct PrioritizedQueue
{
	bool empty() const
	{
		return m_data.empty();
	}

	const std::pair<const PriorityT, ValueT>& front()
	{
		return *m_data.begin();
	}

	void pop()
	{
		if (!m_data.empty())
		{
			m_data.erase(m_data.begin());
		}
	}

	void push(const PriorityT& priority, const ValueT& value)
	{
		//m_data[priority] = value;
		auto range = m_data.equal_range(priority);
		for (auto element = range.first; element != range.second; ++element)
		{
			if (element->second == value)
			{
				//Element is already in queue
				return;
			}
		}
		m_data.insert(std::make_pair(priority, value));
	}

	size_t size() const { return m_data.size(); }
private:
	std::multimap<PriorityT, ValueT> m_data;// Queue data are saved in a balanced by key RB-tree
};
#endif //__PRIORITIZED_QUEUE_H__