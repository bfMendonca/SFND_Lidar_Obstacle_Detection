/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	int depth;

	Node(std::vector<float> arr, int setId, int _depth = 0 )
	:	point(arr), id(setId), left(NULL), right(NULL), depth( _depth )
	{}

	void insert( std::vector<float> p, int setId ) {

		Node** n;

		if( point[ depth % 2 ] < p[ depth % 2 ] ) {
			n = &left;
		}else {
			n = &right;
		}

		if( *n == NULL ) {
			*n = new Node( p, setId, depth+1 );
		}else {
			(*n)->insert( p, setId );
		}
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

		//Ok so, actually, we are at the "root", let's just insert.
		if( root == NULL ) {
			root = new Node( point, id );
		}else {
			//Not the root, let's find out where to place
			root->insert( point, id );
		}

	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




