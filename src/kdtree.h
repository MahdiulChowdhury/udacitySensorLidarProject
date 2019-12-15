/* \author Aaron Brown */
// Quiz on implementing kd tree
#ifndef KDTREE_H 
#define KDTREE_H
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template <typename PointT>
struct KdTree_euclidean
{
	Node* root;


	KdTree_euclidean()
	: root(NULL)
	{}
	
	void inserthelper(Node *&node, uint level, PointT point, int id)
	{
	    uint index = level%3;
		if(node == NULL)
		{
		 std::vector<float> v_point(point.data, point.data+3);
		 node = new Node(v_point,id);
		}
		else if(point.data[index] < node->point[index])
		{
		inserthelper(node->left,level+1,point,id);
		}
		else
		{
		inserthelper(node->right,level+1,point,id);
		}
	}
	void insert_cloud(typename pcl::PointCloud<PointT>::Ptr cloud)
	{
		for(uint index = 0; index < cloud->points.size(); index++)
		{
		   inserthelper(root,0,cloud->points[index],index);
		}

	}
	void helpersearch(Node *&node,uint depth,std::vector<int> *ids,PointT target, float distanceTol)
	{
		uint id = depth%3;
		if(node!=NULL)
		{
			if(((node->point[0]<target.data[0]+distanceTol)&&(node->point[0]>target.data[0]-distanceTol))&&
					((node->point[1]<target.data[1]+distanceTol)&&(node->point[1]>target.data[1]-distanceTol))&&
						((node->point[2]<target.data[2]+distanceTol)&&(node->point[2]>target.data[2]-distanceTol)))
			{
				
				uint dis=sqrt((node->point[0]-target.data[0])*(node->point[0]-target.data[0])+
						(node->point[1]-target.data[1])*(node->point[1]-target.data[1])+
						(node->point[2]-target.data[2])*(node->point[2]-target.data[2]));

				
				if(dis<distanceTol)
				{
					ids->push_back(node->id);
				}
			}

			if(target.data[id]-distanceTol<node->point[id])
			{
				helpersearch(node->left,depth+1,ids,target,distanceTol);

			}
			if(target.data[id]+distanceTol>node->point[id])
			{
				helpersearch(node->right,depth+1,ids,target,distanceTol);

			}

		}
	}
	
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		uint depth =0;
		uint maxdistance=0;

		helpersearch(root,depth,&ids,target,distanceTol);
		return ids;
	}


};

#endif 