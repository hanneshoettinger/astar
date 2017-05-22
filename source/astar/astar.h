
#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>
#include <stdio.h>
//#include <conio.h>
#include <assert.h>

#include <algorithm>
#include <set>
#include <vector>
#include <cfloat>

using namespace std;

// disable warning
#if defined(WIN32) && defined(_WINDOWS)
#pragma warning( disable : 4786 )
#endif

template <class T> class AStarState;

// AStar search class as template function with UserState specific input
template <class UserState> class AStarSearch
{
public: // data
	enum
	{
		SEARCH_STATE_NOT_INITIALISED,
		SEARCH_STATE_SEARCHING,
		SEARCH_STATE_SUCCEEDED,
		SEARCH_STATE_FAILED,
		SEARCH_STATE_OUT_OF_MEMORY,
		SEARCH_STATE_INVALID
	};

	public:

	class Node
	{
		public:

			Node *parent; // parent of successor nodes
			Node *child; // to view the search in reverse
			
			float g; // cost 
			float h; // heuristic estimate of distance to goal
			float f; // sum of costs

			Node() :
				parent( 0 ),
				child( 0 ),
				g( 0.0f ),
				h( 0.0f ),
				f( 0.0f )
			{			
			}

			UserState m_UserState;
	};

	class HeapCompare_f 
	{
		public:

			bool operator() ( const Node *x, const Node *y ) const
			{
				return x->f > y->f;
			}
	};


public:
	// constructor 
	AStarSearch() :
		m_State( SEARCH_STATE_NOT_INITIALISED ),
		m_CurrentSolutionNode( NULL ),
		m_AllocateNodeCount(0),
		m_CancelRequest( false )
	{
	}

	AStarSearch( int MaxNodes ) :
		m_State( SEARCH_STATE_NOT_INITIALISED ),
		m_CurrentSolutionNode( NULL ),
		m_AllocateNodeCount(0),
		m_CancelRequest( false )
	{
	}
	void CancelSearch()
	{
		m_CancelRequest = true;
	}

	// Set Start and goal states
	void SetStartAndGoalStates( UserState &Start, UserState &Goal )
	{
		m_CancelRequest = false;

		m_Start = AllocateNode();
		m_Goal = AllocateNode();

		assert((m_Start != NULL && m_Goal != NULL));
		
		m_Start->m_UserState = Start;
		m_Goal->m_UserState = Goal;

		m_State = SEARCH_STATE_SEARCHING;
		
		// init
		m_Start->g = 0; 
		m_Start->h = m_Start->m_UserState.GoalDistanceEstimate( m_Goal->m_UserState );
		m_Start->f = m_Start->g + m_Start->h;
		m_Start->parent = 0;

		m_OpenList.push_back( m_Start ); // start on openlist

		push_heap( m_OpenList.begin(), m_OpenList.end(), HeapCompare_f() );

		// Initialise counter for search steps
		m_Steps = 0;
	}

	// Search
	unsigned int SearchStep()
	{
		assert( (m_State > SEARCH_STATE_NOT_INITIALISED) &&
				(m_State < SEARCH_STATE_INVALID) );

		// if search succeeded...
		if( (m_State == SEARCH_STATE_SUCCEEDED) ||
			(m_State == SEARCH_STATE_FAILED) 
		  )
		{
			return m_State; 
		}

		// stop if openlist is empty
		if( m_OpenList.empty() || m_CancelRequest )
		{
			FreeAllNodes();
			m_State = SEARCH_STATE_FAILED;
			return m_State;
		}
		
		// Incremement step count
		m_Steps ++;

		// Pop the best node
		Node *n = m_OpenList.front(); 
		pop_heap( m_OpenList.begin(), m_OpenList.end(), HeapCompare_f() );
		m_OpenList.pop_back();

		// Check for the goal
		if( n->m_UserState.IsGoal( m_Goal->m_UserState ) )
		{
			m_Goal->parent = n->parent;
			m_Goal->g = n->g;
			
			// goal was passed in as the start state
			if( false == n->m_UserState.IsSameState( m_Start->m_UserState ) )
			{
				FreeNode( n );
				Node *nodeChild = m_Goal;
				Node *nodeParent = m_Goal->parent;

				do
				{
					if (m_Goal->parent == nullptr)
					{
						m_State = SEARCH_STATE_FAILED;
						return m_State;
					}
					try {
						nodeParent->child = nodeChild;
					}
					catch (const std::overflow_error& e) {
						m_State = SEARCH_STATE_FAILED;
						return m_State;
					}

					nodeChild = nodeParent;
					nodeParent = nodeParent->parent;
				
				} 
				while( nodeChild != m_Start ); // Start is always the first node by definition

			}

			// delete nodes
			FreeUnusedNodes();

			m_State = SEARCH_STATE_SUCCEEDED;

			return m_State;
		}
		else // not goal
		{
			m_Successors.clear(); // empty vector of successor nodes to n

			bool ret = n->m_UserState.GetSuccessors( this, n->parent ? &n->parent->m_UserState : NULL ); 

			if( !ret )
			{
			    typename vector< Node * >::iterator successor;

				// free the nodes that may previously have been added 
				for( successor = m_Successors.begin(); successor != m_Successors.end(); successor ++ )
				{
					FreeNode( (*successor) );
				}

				m_Successors.clear(); // empty vector of successor nodes to n

				// free up everything else we allocated
				FreeAllNodes();

				m_State = SEARCH_STATE_OUT_OF_MEMORY;
				return m_State;
			}
			
			// each successor to the current node ...
			for( typename vector< Node * >::iterator successor = m_Successors.begin(); successor != m_Successors.end(); successor ++ )
			{

				// calculate cost
				float newg = n->g + n->m_UserState.GetCost( (*successor)->m_UserState );

				// find whether the node is on the open or closed lists and check g

				typename vector< Node * >::iterator openlist_result;

				for( openlist_result = m_OpenList.begin(); openlist_result != m_OpenList.end(); openlist_result ++ )
				{
					if( (*openlist_result)->m_UserState.IsSameState( (*successor)->m_UserState ) )
					{
						break;					
					}
				}

				if( openlist_result != m_OpenList.end() )
				{

					if( (*openlist_result)->g <= newg )
					{
						FreeNode( (*successor) );
						continue;
					}
				}

				typename vector< Node * >::iterator closedlist_result;

				for( closedlist_result = m_ClosedList.begin(); closedlist_result != m_ClosedList.end(); closedlist_result ++ )
				{
					if( (*closedlist_result)->m_UserState.IsSameState( (*successor)->m_UserState ) )
					{
						break;					
					}
				}

				if( closedlist_result != m_ClosedList.end() )
				{

					if( (*closedlist_result)->g <= newg )
					{
						FreeNode( (*successor) );

						continue;
					}
				}

				(*successor)->parent = n;
				(*successor)->g = newg;
				(*successor)->h = (*successor)->m_UserState.GoalDistanceEstimate( m_Goal->m_UserState );
				(*successor)->f = (*successor)->g + (*successor)->h;

				// Remove successor from closed if it was on it

				if( closedlist_result != m_ClosedList.end() )
				{
					FreeNode(  (*closedlist_result) ); 
					m_ClosedList.erase( closedlist_result );
					
				}

				// Update old version of this node
				if( openlist_result != m_OpenList.end() )
				{	   

					FreeNode( (*openlist_result) ); 
			   		m_OpenList.erase( openlist_result );

					make_heap( m_OpenList.begin(), m_OpenList.end(), HeapCompare_f() );
			
				}

				m_OpenList.push_back( (*successor) );

				// sort back element into heap
				push_heap( m_OpenList.begin(), m_OpenList.end(), HeapCompare_f() );

			}

			m_ClosedList.push_back( n );

		} // end else (not goal so expand)

 		return m_State; // Succeeded bool is false at this point. 

	}

	bool AddSuccessor( UserState &State )
	{
		Node *node = AllocateNode();

		if( node )
		{
			node->m_UserState = State;

			m_Successors.push_back( node );

			return true;
		}

		return false;
	}

	// Free the solution nodes
	void FreeSolutionNodes()
	{
		Node *n = m_Start;

		if( m_Start->child )
		{
			do
			{
				Node *del = n;
				n = n->child;
				FreeNode( del );

				del = NULL;

			} while( n != m_Goal );

			FreeNode( n ); // Delete the goal

		}
		else
		{
			FreeNode( m_Start );
			FreeNode( m_Goal );
		}

	}

	// Get start node
	UserState *GetSolutionStart()
	{
		m_CurrentSolutionNode = m_Start;
		if( m_Start )
		{
			return &m_Start->m_UserState;
		}
		else
		{
			return NULL;
		}
	}
	
	// Get next node
	UserState *GetSolutionNext()
	{
		if( m_CurrentSolutionNode )
		{
			if( m_CurrentSolutionNode->child )
			{
				Node *child = m_CurrentSolutionNode->child;

				m_CurrentSolutionNode = m_CurrentSolutionNode->child;

				return &child->m_UserState;
			}
		}

		return NULL;
	}
	
	// Get end node
	UserState *GetSolutionEnd()
	{
		m_CurrentSolutionNode = m_Goal;
		if( m_Goal )
		{
			return &m_Goal->m_UserState;
		}
		else
		{
			return NULL;
		}
	}
	
	// Step solution iterator backwards
	UserState *GetSolutionPrev()
	{
		if( m_CurrentSolutionNode )
		{
			if( m_CurrentSolutionNode->parent )
			{

				Node *parent = m_CurrentSolutionNode->parent;

				m_CurrentSolutionNode = m_CurrentSolutionNode->parent;

				return &parent->m_UserState;
			}
		}

		return NULL;
	}

	// Get final cost of solution
	float GetSolutionCost()
	{
		if( m_Goal && m_State == SEARCH_STATE_SUCCEEDED )
		{
			return m_Goal->g;
		}
		else
		{
			return FLT_MAX;
		}
	}
	// Get the number of steps

	int GetStepCount() { return m_Steps; }

private: // methods

	void FreeAllNodes()
	{
		// iterate open list and delete all nodes
		typename vector< Node * >::iterator iterOpen = m_OpenList.begin();

		while( iterOpen != m_OpenList.end() )
		{
			Node *n = (*iterOpen);
			FreeNode( n );

			iterOpen ++;
		}

		m_OpenList.clear();

		// iterate closed list and delete unused nodes
		typename vector< Node * >::iterator iterClosed;

		for( iterClosed = m_ClosedList.begin(); iterClosed != m_ClosedList.end(); iterClosed ++ )
		{
			Node *n = (*iterClosed);
			FreeNode( n );
		}

		m_ClosedList.clear();

		// delete the goal

		FreeNode(m_Goal);
	}

	void FreeUnusedNodes()
	{
		// iterate open list and delete unused nodes
		typename vector< Node * >::iterator iterOpen = m_OpenList.begin();

		while( iterOpen != m_OpenList.end() )
		{
			Node *n = (*iterOpen);

			if( !n->child )
			{
				FreeNode( n );

				n = NULL;
			}

			iterOpen ++;
		}

		m_OpenList.clear();

		// iterate closed list and delete unused nodes
		typename vector< Node * >::iterator iterClosed;

		for( iterClosed = m_ClosedList.begin(); iterClosed != m_ClosedList.end(); iterClosed ++ )
		{
			Node *n = (*iterClosed);

			if( !n->child )
			{
				FreeNode( n );
				n = NULL;

			}
		}

		m_ClosedList.clear();

	}

	// Node memory management
	Node *AllocateNode()
	{

		Node *p = new Node;
		return p;
	}

	void FreeNode( Node *node )
	{

		m_AllocateNodeCount --;

		delete node;
	}

private: // data

	vector< Node *> m_OpenList;
	vector< Node * > m_ClosedList; 
	vector< Node * > m_Successors;

	unsigned int m_State;
	int m_Steps;

	Node *m_Start;
	Node *m_Goal;

	Node *m_CurrentSolutionNode;
	int m_AllocateNodeCount;
	
	bool m_CancelRequest;

};

template <class T> class AStarState
{
public:
	virtual ~AStarState() {}
	virtual float GoalDistanceEstimate( T &nodeGoal ) = 0; // Heuristic function -> estimated cost to the goal node
	virtual bool IsGoal( T &nodeGoal ) = 0;
	virtual bool GetSuccessors( AStarSearch<T> *astarsearch, T *parent_node ) = 0; 
	virtual float GetCost( T &successor ) = 0; // cost of travelling from this node to the successor node
	virtual bool IsSameState( T &rhs ) = 0; 
};

#endif

   
