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

		if( p[ depth % 2 ] < point[ depth % 2 ] ) {
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

	bool isInside( const std::vector<float> & c, float l,  const std::vector<float> & p ) {
		//Check if p is within of the box of center c and l size
		return ( 	  p[0] >= ( c[0] - l ) ) && 
					( p[0] <= ( c[0] + l ) ) && 
					( p[1] >= ( c[1] - l ) ) && 
					( p[1] <= ( c[1] + l ) );

	}

	float distance( const std::vector< float > &p1, const std::vector< float > &p2 ) {
		//Euclidian distance, just another helper function
		return sqrt( pow( p1[0] - p2[0], 2 ) + pow( p1[1] - p2[1], 2 ) );
	}

	void explore( Node *node, std::vector<float> target,  float distanceTol, std::vector<int> &ids ) {
		//Verify if node is valid. NULL -> we hit a "leaf", with nothing else to explore.
		if( node == NULL )
			return; 

		//Verify if the node point is within distance
		if( isInside( target, distanceTol, node->point ) ) {
			if( distance( node->point, target ) <= distanceTol ) {
				ids.push_back( node->id );	
			}
		}
		
		size_t coord = node->depth % 2;

		if( ( target[coord] ) < ( node->point[coord] + distanceTol ) ) {
			explore( node->left, target, distanceTol, ids );
		}

		if( ( target[coord] ) > ( node->point[coord] - distanceTol ) ) {
			explore( node->right, target, distanceTol, ids );	
		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;		
		explore( root, target, distanceTol, ids );

		return ids;
	}
	

};




