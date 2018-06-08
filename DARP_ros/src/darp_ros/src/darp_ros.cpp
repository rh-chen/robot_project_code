#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/matx.hpp>
#include <sys/stat.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>

#include <boost/thread/thread.hpp>
#include <boost/make_shared.hpp>

#include <stdio.h>
#include <math.h>

#include <hash_set>
#include <set>
#include <map>


namespace darp_ros {

using namespace __gnu_cxx; 
using namespace std;
using namespace cv;

boost::shared_ptr<boost::thread> darp_thread;

cv::Mat environment_grid_;
cv::Mat binary_grid_;
cv::Mat label_2d_;

class DARP
{
	public:
	
	int rows,cols;
	int nr;
	int obs;

	cv::Mat robotBinary;
	cv::Mat A;
	cv::Mat GridEnv;
	cv::Mat BinrayRobotRegions;
	
	DARP(int rows_,int cols_,cv::Mat& src_)
	{
		rows = rows_;
		cols = cols_;
		nr = 0;
		obs = 0;
		
		deepCopyMatrix(src_,GridEnv);
		
		printf("GridEnv.rows:%d\n",GridEnv.rows);
		printf("GridEnv.cols:%d\n",GridEnv.cols);
		printf("GridEnv_old");
		for(int i = 0;i < GridEnv.rows;i++)
			for(int j = 0;j < GridEnv.cols;j++)
			{
				std::cout << GridEnv.at<int>(i,j) << " ";
				if(j == GridEnv.cols-1)
					printf("\n");
			}

		robotBinary.create(rows,cols,CV_32SC1);
		memset(robotBinary.data,0,rows*cols*4);
		A.create(rows,cols,CV_32SC1);
		memset(A.data,0,rows*cols*4);
		BinrayRobotRegions.create(rows,cols,CV_32SC1);
		memset(BinrayRobotRegions.data,0,rows*cols*4);

		defineRobotsObstacles();

		assign();

		calculateRobotBinaryArrays();
	}

	void assign()
	{
		int indMin = 0;
		for (int i = 0; i < rows; i++) {
      for (int j = 0; j < cols; j++){
        if (GridEnv.at<int>(i,j)==-1) {
          A.at<int>(i,j) = indMin;
        } else if (GridEnv.at<int>(i,j)==-2) {A.at<int>(i,j) = nr;}
      }
  	}
	}
	void deepCopyMatrix(cv::Mat& data_,cv::Mat& dst_)
	{data_.copyTo(dst_);}

	void defineRobotsObstacles(){
    for(int i = 0;i < rows;i++){
      for (int j = 0;j < cols;j++){
        if (GridEnv.at<int>(i,j) == 2) {
          robotBinary.at<int>(i,j) = 1;//true
          GridEnv.at<int>(i,j) = nr;
          A.at<int>(i,j) = nr;
          nr++;
        }
        else if (GridEnv.at<int>(i,j) == 1) {
          obs++;
          GridEnv.at<int>(i,j) = -2;
        } else {GridEnv.at<int>(i,j) = -1;}
    }
		}

		printf("robotBinary\n");
		for(int i = 0;i < rows;i++){
      for (int j = 0;j < cols;j++){
        std::cout << robotBinary.at<int>(i,j) << " ";
				if(j == cols-1)
					printf("\n");
			}
		}
		printf("GridEnv_new\n");
		for(int i = 0;i < rows;i++){
      for (int j = 0;j < cols;j++){
        std::cout << GridEnv.at<int>(i,j) << " ";
				if(j == cols-1)
					printf("\n");
			}
		}
	}
	
	int getNr(){return nr;}
	int getNumOB(){return obs;}
	
	void calculateRobotBinaryArrays()
	{
    for (int i = 0;i < rows;i++){
    	for (int j = 0;j < cols;j++) {
        if (A.at<int>(i,j) < nr){
						BinrayRobotRegions.at<int>(i,j) = 1;//true
        }
    	}
		}

		printf("BinrayRobotRegions\n");
		for (int i = 0;i < rows;i++){
    	for (int j = 0;j < cols;j++) {
				std::cout << BinrayRobotRegions.at<int>(i,j) << " ";
				if(j == cols-1)
					printf("\n");
    	}
		}
	}

};


class ConnectComponent
{
	public:
	ConnectComponent(int rows_,int cols_)
	{
		printf("construct ConnectComponent object......\n");
		next_label = 1;
		rows = rows_;
		cols = cols_;
		MAX_LABELS = rows_*cols_;
	}

	int MAX_LABELS;
	int next_label;
	cv::Mat BinaryRobot,BinaryNonRobot;
	int rows,cols;


	int getMaxLabel() {return next_label;}

	void compactLabeling(cv::Mat& data_,cv::Mat& label_2d_,int rows,int cols,bool zeroAsBg)
	{
		vector<int> image;
		transform2Dto1D(data_,image,rows,cols);

		printf("transform 2 Dimension to 1 Dimension......\n");
		for(int i = 0;i < image.size();i++)
			std::cout << image[i] << " ";
		std::cout << std::endl;
		
//std::cout << __FILE__ << __LINE__ << std::endl;
		vector<int> label(MAX_LABELS,0);
		labeling(image,label,rows,cols,zeroAsBg);
//std::cout << __FILE__ << __LINE__ << std::endl;		
		printf("labeling......\n");
		for(int i = 0;i < label.size();i++)
			std::cout << label[i] << " ";
		std::cout << std::endl;

		printf("next_label:%d\n",next_label);
		vector<int> stat_(next_label+1,0);

		for(int i = 0;i < image.size();i++)
			stat_[label[i]]++;

		stat_[0] = 0;
		int temp = 1;
		for(int i = 1;i < stat_.size();i++)
			if(stat_[i] != 0) stat_[i] = temp++;

		printf("stat_......\n");
		for(int i = 0;i < stat_.size();i++)
			std::cout << stat_[i] << " ";
		std::cout << std::endl;

		next_label = temp-1;
		int locIDX = 0;
		for(int i = 0;i < rows;i++)
			for(int j = 0;j < cols;j++)
			{
				label[locIDX] = stat_[label[locIDX]];
				label_2d_.at<int>(i,j) = label[locIDX];
				
				locIDX++;
			} 
	}

	void transform2Dto1D(cv::Mat& data_,vector<int>& vec_,int rows,int cols)
	{
		int k = 0;
		for(int i = 0;i < rows;i++)
			for(int j = 0;j < cols;j++)
			{
				vec_.push_back(data_.at<int>(i,j));
			}
	}

	void labeling(vector<int>& data_,vector<int>& vec_,int rows,int cols,bool zeroAsBg)
	{
		vector<int> parent(MAX_LABELS);
		vector<int> labels(MAX_LABELS);

		int next_region = 1;
		for(int y = 0;y < rows;y++){
			for(int x = 0;x < cols;x++){
				if(data_[y*cols+x] == 0 && zeroAsBg) continue;

				int k = 0;
				bool connected = false;

				if(x > 0 && data_[y*cols+x-1] == data_[y*cols+x])
				{
					k = vec_[y*cols+x-1];
					//printf("k1:%d\n",k);
					connected = true;
				}
//std::cout << __FILE__ << __LINE__ << std::endl;
				if(y > 0 && data_[(y-1)*cols+x] == data_[y*cols+x] && (!connected || data_[(y-1)*cols+x] < k))
				{
					k = vec_[(y-1)*cols+x];
					//printf("k2:%d\n",k);
					connected = true;
				}

				if(!connected)
				{
					k = next_region;
					//printf("k3:%d\n",k);
					next_region++;
				}

				vec_[y*cols+x] = k;

				if(x > 0 && data_[y*cols+x-1] == data_[y*cols+x] && vec_[y*cols+x-1] != k)
				{
					//std::cout << __FILE__ << __LINE__ << std::endl;
					uf_union(k,vec_[y*cols+x-1],parent);
					//std::cout << __FILE__ << __LINE__ << std::endl;
				}

				if(y > 0 && data_[(y-1)*cols+x] == data_[y*cols+x] && vec_[(y-1)*cols+x] != k)
				{
					uf_union(k,vec_[(y-1)*cols+x],parent);
				}

			}
		}
		next_label = 1;
		//std::cout << __FILE__ << __LINE__ << std::endl;
		for(int i = 0;i < rows*cols;i++){
			if(data_[i] != 0 || !zeroAsBg){
				vec_[i] = uf_find(vec_[i],parent,labels);
				if(!zeroAsBg) vec_[i]--;
			}
		}
		//std::cout << __FILE__ << __LINE__ << std::endl;
		next_label--;
		if(!zeroAsBg) next_label--;
	}

 	void uf_union( int x, int y, vector<int>& parent)
  {
			printf("start_x:%d\n",x);
			printf("start_y:%d\n",y);
//std::cout << __FILE__ << __LINE__ << std::endl;
      while ( parent[x]>0 ){
          x = parent[x];
					printf("x:%d\n",x);
			}
//std::cout << __FILE__ << __LINE__ << std::endl;
      while ( parent[y]>0 ){
          y = parent[y];
					printf("y:%d\n",y);
			}
//std::cout << __FILE__ << __LINE__ << std::endl;
      if ( x != y ) {
          if (x<y)
              parent[x] = y;
          else parent[y] = x;
      }
//std::cout << __FILE__ << __LINE__ << std::endl;
  }

  int uf_find( int x, vector<int>& parent, vector<int>& label)
  {
      while ( parent[x]>0 )
          x = parent[x];
      if ( label[x] == 0 )
          label[x] = next_label++;
      return label[x];
  }

};

class Edge{
	public:

	int from,to,cost;
	Edge(){};
	Edge(int f,int t,int c){from = f;to = t;cost = c;}
};

class Kruskal{
	public:

	int MAX_NODES;
	vector<hash_set<int> > nodes;
	map<int,Edge> allEdges;
	vector<Edge> allNewEdges;
	int node_count;
	
	Kruskal(int rows,int cols){
		MAX_NODES = rows*cols;
		std::cout << "MAX_NODES:" << MAX_NODES << std::endl;
		nodes.resize(MAX_NODES);
		std::cout << "nodes_size:" << nodes.size() << std::endl;
		node_count = 0;
	}

	void initializeGraph(cv::Mat& data,bool connect)
	{
		int rows = data.rows;
		int cols = data.cols;

		for(int i = 0;i < rows;i++){
			for(int j = 0;j < cols;j++){
				if(data.at<int>(i,j)){
					if(i > 0 && data.at<int>(i-1,j)){
						AddToAllEdges(i*cols+j,(i-1)*cols+j,1);
					}
					if(i < rows-1 && data.at<int>(i+1,j))
					{
						AddToAllEdges(i*cols+j,(i+1)*cols+j,1);
					}
					if(j > 0 && data.at<int>(i,j-1))
					{
						AddToAllEdges(i*cols+j,i*cols+j-1,1);
					}
					if(j < cols-1 && data.at<int>(i,j+1))
					{
						AddToAllEdges(i*cols+j,i*cols+j+1,1);
					}

					if(!connect){
						if(i > 0 && j > 0 && data.at<int>(i-1,j-1))
						{
							AddToAllEdges(i*cols+j,(i-1)*cols+j-1,1);
						}
						if(i < rows-1 && j < cols-1 && data.at<int>(i+1,j+1))
						{
							AddToAllEdges(i*cols+j,(i+1)*cols+j+1,1);
						}
						if(i > rows-1 && j > 0 && data.at<int>(i+1,j-1))
						{
							AddToAllEdges(i*cols+j,(i+1)*cols+j-1,1);
						}
						if(i > 0 && j < cols-1 && data.at<int>(i-1,j+1))
						{
							AddToAllEdges(i*cols+j,(i-1)*cols+j+1,1);
						}
					}

				}
			}
		}

		for(int i = 0;i < MAX_NODES;i++){
			if(nodes[i].size() > 0 ){
				std::cout << "every_nodes_size:" << nodes[i].size() << std::endl;
				node_count++;
			}
		}
		std::cout << "node_count:" << node_count << std::endl;
		std::cout << "allEdges_size:" << allEdges.size() << std::endl;
	}

	void AddToAllEdges(int from,int to,int cost)
	{
		static uint64 key = 0;
		key++;

		Edge e(from,to,cost);
		allEdges.insert(pair<int,Edge>(key,e));
		
		if(nodes[from].size() == 0){
			nodes[from].resize(2*MAX_NODES);
			nodes[from].insert(from);
		}
		if(nodes[to].size() == 0){
			nodes[to].resize(2*MAX_NODES);
			nodes[to].insert(to);
		}
	}

	void performKruskal()
	{
		int min_span_tree_edge_count = 0;
		int edge_size = allEdges.size();
		int vex_size = node_count;

		vector<int> parent;
		for(int i = 0;i < vex_size;i++)
			parent.push_back(0);
	
		for(int i = 0;i < edge_size;i++){			
			Edge curEdge(allEdges[i].from,allEdges[i].to,allEdges[i].cost);

				if(nodesAreInDifferentSets(parent,curEdge.from,curEdge.to)){
					min_span_tree_edge_count++;
					allNewEdges.push_back(curEdge);
				}else{
					std::cout << "nodes are in the same set......" << std::endl;
				}
			}
		std::cout << "min_span_tree_edge_count:" << min_span_tree_edge_count <<std::endl; 
	}

	int find_root(vector<int>& parent,int node)
	{
		while(parent[node])
			node = parent[node];
		
		return node;
	}
	bool nodesAreInDifferentSets(vector<int>& parent,int a ,int b){

		int n = find_root(parent,a);
		int m = find_root(parent,b);

		if(n!=m){
			parent[n] = m;
			return true;
		}
		else
			return false;
	}

};
class DarpRosNodelet : public nodelet::Nodelet {


	int compare_edge(Edge& e1,Edge& e2)
	{
		int cost1 = e1.cost;
		int cost2 = e2.cost;

		int from1 = e1.from;
		int from2 = e2.from;

		int to1 = e1.to;
		int to2 = e2.to;

		if(cost1 < cost2)
			return -1;
		else if(cost1 == cost2 && from1 == from2 && to1 == to2)
			return 0;
		else if(cost1 == cost2)
			return -1;
		else if(cost1 > cost2)
			return 1;
		else
			return 0;
	}

	bool equal_edge(Edge& e1,Edge& e2)
	{
		return (e1.cost == e2.cost && e1.from == e2.from && e1.to == e2.to);
	}

	void calculateMSTs(cv::Mat& region_,vector<Edge>& vec_,int nr_)
	{
		int rows = region_.rows;
		int cols = region_.cols;

    for (int r = 0;r < nr_;r++){
		  Kruskal k(rows,cols);
		  k.initializeGraph(region_,true);
		  k.performKruskal();
		  /*MSTs.add(k.getAllNewEdges());*/
		}
	}

	void getGraphics(cv::Mat& map_,cv::Mat& data_)
	{
		for(int i = 0;i < map_.rows;i++)
			for(int j = 0;j < map_.cols;j++)
			{
				if(map_.at<unsigned char>(i,j) == 128)
					data_.at<int>(i,j) = 1;
				else if(map_.at<unsigned char>(i,j) == 255)
					data_.at<int>(i,j) = 2;
				else
					data_.at<int>(i,j) = 0;
			}
	}

	void makeGridBinary(cv::Mat& data_,cv::Mat& binary_)
	{
		for(int i = 0;i < data_.rows;i++)
			for(int j = 0;j < data_.cols;j++)
			{
				if(data_.at<int>(i,j) != 1)
					binary_.at<int>(i,j) = 1;
				else
					binary_.at<int>(i,j) = 0;
			}
	}

	void call_back() {
		NODELET_INFO_STREAM("start Darp algorithm......\n");

		ConnectComponent cc(environment_grid_.rows,environment_grid_.cols);
		NODELET_INFO_STREAM("cc.rows:" << cc.rows);
		NODELET_INFO_STREAM("cc.cols:" << cc.cols);
		NODELET_INFO_STREAM("cc.MAX_LABELS:" << cc.MAX_LABELS);

		binary_grid_.create(cc.rows,cc.cols,CV_32SC1);
		makeGridBinary(environment_grid_,binary_grid_);

		NODELET_INFO_STREAM("binary_grid_:\n" << binary_grid_);

		label_2d_.create(cc.rows,cc.cols,CV_32SC1);
		cc.compactLabeling(binary_grid_,label_2d_,binary_grid_.rows,binary_grid_.cols,true);

		NODELET_INFO_STREAM("label_2d_:\n" << label_2d_);

		if(cc.getMaxLabel() > 1)
		{NODELET_INFO_STREAM("The environment grid MUST not have unreachable and/or closed shape regions\n\n");}

		DARP p(environment_grid_.rows,environment_grid_.cols,environment_grid_);

		NODELET_INFO_STREAM("nr:" << p.getNr());
		NODELET_INFO_STREAM("obs:" << p.getNumOB());
		if(p.getNr() < 1)
		{NODELET_INFO_STREAM("Please define at least one robot \n\n");};

		vector<Edge> vec_edge;
		calculateMSTs(p.BinrayRobotRegions,vec_edge,p.nr);
		
	}

	void onInit() {
		ros::NodeHandle pn("~");

		//pn.param<string>("map_path_string", map_path_, "/home/wzm/ine_detection_and_rotation_ros/src/line_detection_and_rotation/maps/test.bmp");	
		//NODELET_INFO_STREAM("map_path_:" << map_path_);

		std::cout << __FILE__ << __LINE__ << std::endl;
		
		int rows = 7;
		int cols = 7;

		unsigned char map_data[] = {
		0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,
		0,0,0,128,0,0,0,
		0,0,0,128,0,255,0,
		0,0,0,128,0,0,0,
		0,0,0,0,0,0,0,
		0,0,0,0,0,0,0};

		cv::Mat map_(rows,cols,CV_8UC1,map_data);

		if(map_.empty())
		{
			std::cout << "map open fail.............." << std::endl;
			return;
		}
		else{
			NODELET_INFO_STREAM("map_width:" << map_.cols);
			NODELET_INFO_STREAM("map_height:" << map_.rows);
			
			environment_grid_.create(map_.rows,map_.cols,CV_32SC1);
			getGraphics(map_,environment_grid_);

			NODELET_INFO_STREAM("environment_grid_:\n" << environment_grid_);

			darp_thread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&DarpRosNodelet::call_back, this)));
		}
	}

	~DarpRosNodelet() {		
	}

};

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(darp_ros::DarpRosNodelet, nodelet::Nodelet);
