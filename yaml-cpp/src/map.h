#pragma once

#ifndef MAP_H_62B23520_7C8E_11DE_8A39_0800200C9A66
#define MAP_H_62B23520_7C8E_11DE_8A39_0800200C9A66


#include "content.h"
#include <map>
#include <memory>

namespace YAML
{
	class Node;

	class Map: public Content
	{
	private:
		typedef std::map <Node *, Node *, ltnode> node_map;

	public:
		Map();
		virtual ~Map();

		void Clear();

		virtual bool GetBegin(std::map <Node *, Node *, ltnode>::const_iterator& it) const;
		virtual bool GetEnd(std::map <Node *, Node *, ltnode>::const_iterator& it) const;
		virtual std::size_t GetSize() const;

		virtual void Insert(std::auto_ptr<Node> pKey, std::auto_ptr<Node> pValue);
		virtual void EmitEvents(AliasManager& am, EventHandler& eventHandler, const Mark& mark, const std::string& tag, anchor_t anchor) const;

		virtual bool IsMap() const { return true; }

		// ordering
		virtual int Compare(Content *pContent);
		virtual int Compare(Scalar *) { return 1; }
		virtual int Compare(Sequence *) { return 1; }
		virtual int Compare(Map *pMap);

	private:
		node_map m_data;
	};
}

#endif // MAP_H_62B23520_7C8E_11DE_8A39_0800200C9A66
