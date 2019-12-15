// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "kdtree.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    //create the filtering object: downsample the dataset using a leaf size of .2m 
    pcl::VoxelGrid<PointT> vg; 
    //typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    

    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);

    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes,filterRes);
    vg.filter(*cloudRegion);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudRegion);
    region.filter(*cloudRegion);

    std::vector<int>indices; 

    pcl::CropBox<PointT> roof (true);
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f(2.6,1.7,-.4,1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point:indices)
    {
        inliers->indices.push_back(point);
    }
    pcl::ExtractIndices<PointT>extract; 
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter (*cloudRegion);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

    for (int index : inliers -> indices)
    {
        planeCloud -> points.push_back(cloud->points[index]);
    }
    pcl::ExtractIndices<PointT> extract; 
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative(true);
    extract.filter (*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::SACSegmentation<PointT> seg; 
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
   pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment (*inliers, *coefficients);

    //auto inliers_set = SegmentPlane_RANSAC(cloud, maxIterations, distanceThreshold);
    //cout<<"type==" << typeid(inliers_set).name()<<endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> inliers_set = SegmentPlane_RANSAC(cloud, maxIterations,distanceThreshold) ;
    //cout << "size of the pair == "<< sizeof(inliers_set) << endl; 
    

    for (int index = 0; index<sizeof(inliers_set); index++)
	{
		inliers->indices.push_back(index);
		//cout<<"Index==" << index << endl; 
	}

	/*for (int index = 0; index<cloud->points.size(); index++)
	{
		
	}*/

    if (inliers->indices.empty())
    {
        std::cout << "Could not estimate a planar model for the given dataset" << std::endl; 
    }
	//pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane_RANSAC(cloud, 50, 0.2);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    //KdTree* tree = new KdTree;
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec; 
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    //ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for (pcl::PointIndices getIndices: clusterIndices)
    {
        typename pcl::PointCloud <PointT>:: Ptr cloudCluster (new pcl::PointCloud <PointT>);

        for (int index: getIndices.indices)
        {
            cloudCluster->points.push_back(cloud->points[index]);
        }
        cloudCluster->width = cloudCluster -> points.size(); 
        cloudCluster->height = 1; 

        clusters.push_back(cloudCluster);
    }
    //std::vector<std::vector<int>> cluster_indices = cluster_helper(cloud, tree,clusterTolerance ,minSize,maxSize);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
/*


Proximity(point,cluster):
    mark point as processed
    add point to cluster
    nearby points = tree(point)
    Iterate through each nearby point
        If point has not been processed
            Proximity(cluster)

*/

template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(typename pcl::PointCloud<PointT>::Ptr cloud,std::vector<int> &cluster,std::vector<bool> &processed_flag,int idx,typename KdTree_euclidean<PointT>::KdTree_euclidean* tree,float distanceTol, int maxSize)
{
    if ((processed_flag[idx] == false) && (cluster.size()<maxSize))
    {
        processed_flag[idx] = true; 
        cluster.push_back(idx);
        std::vector<int> nearby = tree->search(cloud->points[idx], distanceTol);
        for (int index: nearby)
        {
            if(processed_flag[index] == false)
            {
                Proximity(cloud, cluster,processed_flag,index, tree,distanceTol,maxSize);
            }
        }
    }
}

/*
EuclideanCluster():
    list of clusters 
    Iterate through each point
        If point has not been processed
            Create cluster
            Proximity(point, cluster)
            cluster add clusters
    return clusters*/

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::cluster_helper(typename pcl::PointCloud<PointT>::Ptr cloud, typename KdTree_euclidean<PointT>::KdTree_euclidean* tree, float distanceTol, int minSize, int maxSize)
{
    std::vector<std::vector<int>> clusters; 
        std::vector<bool> processed_flag(cloud->points.size(),false);

    for (int index = 0; index < cloud->points.size(); index++)
    {
        if (processed_flag[index]==false)
        {
            std::vector<int> cluster; 
            Proximity(cloud,cluster,processed_flag,index,tree,distanceTol,maxSize);
            if ((cluster.size()>=minSize) && (cluster.size() <= maxSize))
            {
                clusters.push_back(cluster);
            }
        }
    }

    return clusters; 
}




template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering_euclidean(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    //create cluster object...
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    //create tree object to use kD tree algorithm
    typename KdTree_euclidean<PointT>::KdTree_euclidean *tree =new KdTree_euclidean<PointT>;
    tree->insert_cloud(cloud); 

    //Perform euclidean clustering to group detected cloud points 
    std::vector<std::vector<int>> cluster_indices = cluster_helper(cloud,tree,clusterTolerance,minSize,maxSize);

    for (std::vector<std::vector<int>>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
      {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->begin (); pit != it->end (); ++pit)
          cloud_cluster->points.push_back (cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
      }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "euclideanClustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;

}



template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane_RANSAC(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold) 
{

	auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	PointT point1;
	PointT point2;
	PointT point3;
	float valu_one,val_two,val_three,val_four,distance,length;

	while (maxIterations--)
	{
		std::unordered_set<int> inliers; 
		while (inliers.size()<3)
		{
			inliers.insert(rand()%(cloud->points.size()));
		}

		 

		auto itr = inliers.begin();
		int point_index_one = *itr;
		++itr; 
		int point_index_two = *itr;
		++itr; 
		int point_index_three = *itr;
		point1 = cloud->points[point_index_one]; 
		point2 = cloud->points[point_index_two];
		point3 = cloud->points[point_index_three];


		valu_one = (((point2.y-point1.y)*(point3.z-point1.z))-((point2.z-point1.z)*(point3.y-point1.y)));
		val_two = (((point2.z-point1.z)*(point3.x-point1.x))-((point2.x-point1.x)*(point3.z-point1.z)));
		val_three = (((point2.x-point1.x)*(point3.y-point1.y))-((point2.y-point1.y)*(point3.x-point1.x)));
		val_four = -(valu_one*point1.x+val_two*point1.y+val_three*point1.z);
		length = sqrt(valu_one*valu_one+val_two*val_two+val_three*val_three);

		for (int index = 0; index<cloud->points.size(); index++)
		{
			if(index != point_index_one ||index != point_index_two || index != point_index_three)
			{
				distance = (fabs(valu_one*cloud->points[index].x+val_two*cloud->points[index].y+val_three*cloud->points[index].z+val_four)/length);
			}

			
		}

		if (inliers.size()>inliersResult.size())
		{
			inliersResult = inliers;
		}

	}

	if (inliersResult.size () == 0)
	{
	  std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	}

	typename pcl::PointCloud<PointT>::Ptr cloud_Inliers (new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloud_Outiers (new pcl::PointCloud<PointT>());

	for (int index = 0; index<cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
		{
			cloud_Inliers -> points.push_back(point);
		}
		else 
		{
			cloud_Outiers->points.push_back(point);
		}
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_Outiers, cloud_Inliers);

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;

}

