#include <deque>
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
#include <algorithm> 

#include <tf/tf.h>
#include "scale_map/GetCoveragePathScaleMap.h"


namespace scale_map{

using namespace __gnu_cxx; 
using namespace std;
using namespace cv;

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
	vector<cv::Point> RobotsInit;
	
	DARP(int rows_,int cols_,cv::Mat& src_)
	{
		rows = rows_;
		cols = cols_;
		nr = 0;
		obs = 0;
		
		deepCopyMatrix(src_,GridEnv);
		
		//printf("GridEnv.rows:%d\n",GridEnv.rows);
		//printf("GridEnv.cols:%d\n",GridEnv.cols);
		/*printf("GridEnv_old");
		for(int i = 0;i < GridEnv.rows;i++)
			for(int j = 0;j < GridEnv.cols;j++)
			{
				std::cout << GridEnv.at<int>(i,j) << " ";
				if(j == GridEnv.cols-1)
					printf("\n");
			}
		*/
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
					RobotsInit.push_back(cv::Point(j,i));
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

		/*printf("robotBinary\n");
		for(int i = 0;i < rows;i++){
      for (int j = 0;j < cols;j++){
        std::cout << robotBinary.at<int>(i,j) << " ";
				if(j == cols-1)
					printf("\n");
			}
		}*/

		/*printf("GridEnv_new\n");
		for(int i = 0;i < rows;i++){
      for (int j = 0;j < cols;j++){
        std::cout << GridEnv.at<int>(i,j) << " ";
				if(j == cols-1)
					printf("\n");
			}
		}
		*/
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

		/*printf("BinrayRobotRegions\n");
		for (int i = 0;i < rows;i++){
    	for (int j = 0;j < cols;j++) {
				std::cout << BinrayRobotRegions.at<int>(i,j) << " ";
				if(j == cols-1)
					printf("\n");
    	}
		}
		*/
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

		/*printf("transform 2 Dimension to 1 Dimension......\n");
		for(int i = 0;i < image.size();i++)
			std::cout << image[i] << " ";
		std::cout << std::endl;
		*/

//std::cout << __FILE__ << __LINE__ << std::endl;
		vector<int> label(MAX_LABELS,0);
		labeling(image,label,rows,cols,zeroAsBg);
//std::cout << __FILE__ << __LINE__ << std::endl;		
		/*printf("labeling......\n");
		for(int i = 0;i < label.size();i++)
			std::cout << label[i] << " ";
		std::cout << std::endl;
		*/
		printf("next_label:%d\n",next_label);
		vector<int> stat_(next_label+1,0);

		for(int i = 0;i < image.size();i++)
			stat_[label[i]]++;

		stat_[0] = 0;
		int temp = 1;
		for(int i = 1;i < stat_.size();i++)
			if(stat_[i] != 0) stat_[i] = temp++;

		/*printf("stat_......\n");
		for(int i = 0;i < stat_.size();i++)
			std::cout << stat_[i] << " ";
		std::cout << std::endl;
		*/

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
			//printf("start_x:%d\n",x);
			//printf("start_y:%d\n",y);
//std::cout << __FILE__ << __LINE__ << std::endl;
      while ( parent[x]>0 ){
          x = parent[x];
					//printf("x:%d\n",x);
			}
//std::cout << __FILE__ << __LINE__ << std::endl;
      while ( parent[y]>0 ){
          y = parent[y];
					//printf("y:%d\n",y);
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

typedef struct EdgeStr{
	//public:
	int from,to,cost;
	EdgeStr(){};
	EdgeStr(int f,int t,int c){from = f;to = t;cost = c;}
}Edge;

class Kruskal{
	public:

	int MAX_NODES;
	vector<hash_set<int> > nodes;
	map<int,Edge> allEdges;
	vector<Edge> allNewEdges;
	int node_count;
	int min_span_tree_edge_count;
	
	Kruskal(int rows,int cols){
		MAX_NODES = rows*cols;
		std::cout << "Kruskal_MAX_NODES:" << MAX_NODES << std::endl;
		nodes.resize(MAX_NODES);
		std::cout << "Kruskal_nodes_size:" << nodes.size() << std::endl;
		node_count = 0;
		min_span_tree_edge_count = 0;
	}

	void initializeGraph(cv::Mat& data,bool connect)
	{
		int rows = data.rows;
		int cols = data.cols;
		int key = -1;
		for(int i = rows-1;i >= 0;i--){
			for(int j = cols-1;j >=0;j--){
				if(data.at<int>(i,j)){

					if(j < cols-1 && data.at<int>(i,j+1))
					{
						key++;
						AddToAllEdges(i*cols+j,i*cols+j+1,1,key);
					}
					if(i < rows-1 && data.at<int>(i+1,j))
					{
						key++;
						AddToAllEdges(i*cols+j,(i+1)*cols+j,1,key);
					}
					if(j > 0 && data.at<int>(i,j-1))
					{
						key++;
						AddToAllEdges(i*cols+j,i*cols+j-1,1,key);
					}
					if(i > 0 && data.at<int>(i-1,j)){
						key++;
						AddToAllEdges(i*cols+j,(i-1)*cols+j,1,key);
					}

					if(!connect){
						if(i > 0 && j > 0 && data.at<int>(i-1,j-1))
						{
							AddToAllEdges(i*cols+j,(i-1)*cols+j-1,1,key);
						}
						if(i < rows-1 && j < cols-1 && data.at<int>(i+1,j+1))
						{
							AddToAllEdges(i*cols+j,(i+1)*cols+j+1,1,key);
						}
						if(i > rows-1 && j > 0 && data.at<int>(i+1,j-1))
						{
							AddToAllEdges(i*cols+j,(i+1)*cols+j-1,1,key);
						}
						if(i > 0 && j < cols-1 && data.at<int>(i-1,j+1))
						{
							AddToAllEdges(i*cols+j,(i-1)*cols+j+1,1,key);
						}
					}

				}
			}
		}

		/*for(int i = 0;i < MAX_NODES;i++){
			if(nodes[i].size() > 0 ){
				std::cout << "Kruskal_every_nodes_size:" << nodes[i].size() << std::endl;
				node_count++;
			}
		}
		*/
		//std::cout << "Kruskal_node_count:" << node_count << std::endl;
		std::cout << "Kruskal_allEdges_size:" << allEdges.size() << std::endl;
	}

	void AddToAllEdges(int from,int to,int cost,int key)
	{
		//static uint64 key = 0;
		//key++;

		Edge e(from,to,cost);

		//std::cout << "e.from:" << e.from << "," << "to:" << e.to << std::endl;
		allEdges.insert(pair<int,Edge>(key,e));
		
		if(nodes[from].size() == 0){
			//nodes[from].resize(2*MAX_NODES);
			nodes[from].insert(from);
		}
		if(nodes[to].size() == 0){
			//nodes[to].resize(2*MAX_NODES);
			nodes[to].insert(to);
		}
		//key++;
	}

	void performKruskal()
	{
		int edge_size = allEdges.size();
		int vex_size = MAX_NODES;
		std::cout << "edge_size:" << edge_size << std::endl;
		//std::cout << __FILE__ << __LINE__ << std::endl;
		vector<int> parent;
		for(int i = 0;i < vex_size;i++)
			parent.push_back(-1);
	
		//std::cout << __FILE__ << __LINE__ << std::endl;
		for(int i = 0;i < edge_size;i++){			
			Edge curEdge(allEdges[i].from,allEdges[i].to,allEdges[i].cost);
			//std::cout << "curEdge.from:" << curEdge.from << "," << "to:" << curEdge.to << std::endl;
		//std::cout << __FILE__ << __LINE__ << std::endl;
				if(nodesAreInDifferentSets(parent,curEdge.from,curEdge.to)){
					min_span_tree_edge_count++;

		//std::cout << __FILE__ << __LINE__ << std::endl;
					allNewEdges.push_back(curEdge);
		//std::cout << __FILE__ << __LINE__ << std::endl;
				}else{
					;//std::cout << "nodes are in the same set......" << std::endl;
				}

		//std::cout << __FILE__ << __LINE__ << std::endl;
			}

		//std::cout << __FILE__ << __LINE__ << std::endl;
		std::cout << "min_span_tree_edge_count:" << min_span_tree_edge_count <<std::endl; 
	}

	int find_root(vector<int>& parent,int node)
	{
		//std::cout << "node:" << node << std::endl;
		//std::cout << __FILE__ << __LINE__ << std::endl;
		int s;
		for(s = node;parent[s] >= 0;s = parent[s]);
		
		//std::cout << __FILE__ << __LINE__ << std::endl;
		while(s != node)
		{

		//std::cout << __FILE__ << __LINE__ << std::endl;
			int tmp = parent[node];
			parent[node] = s;
			node = tmp;

		//std::cout << __FILE__ << __LINE__ << std::endl;
		}
		
		//std::cout << __FILE__ << __LINE__ << std::endl;
		return s;
		/*while(parent[node])
			node = parent[node];
		
		return node;*/	
	}
	bool nodesAreInDifferentSets(vector<int>& parent,int a ,int b){

		//std::cout << __FILE__ << __LINE__ << std::endl;
		int n = find_root(parent,a);

		//std::cout << __FILE__ << __LINE__ << std::endl;
		int m = find_root(parent,b);

		//std::cout << __FILE__ << __LINE__ << std::endl;
		if(n!=m){
			parent[n] = m;
			return true;
		}
		else
			return false;
			
		//std::cout << __FILE__ << __LINE__ << std::endl;
	}
	void printFinalEdges()
	{
		for(int i = 0;i < min_span_tree_edge_count;i++)
		{
			std::cout << "Node:(" << allNewEdges[i].from << "," <<allNewEdges[i].to << ") with cost:" << allNewEdges[i].cost << std::endl; 
		}
	}
};

typedef struct path_node
{
	int pre_i;
	int pre_j;
	int i;
	int j;
}PathNode;

class edgeCmp{
	public:
	bool operator()(Edge const &e_a,Edge const &e_b)const
	{
		if((e_a.from*e_a.to) == (e_b.from*e_b.to)){
			return (e_a.from < e_b.from?true:false);
		}
		else{
			return ((e_a.from*e_a.to) < (e_b.from*e_b.to)?true:false);
		}
	}
};

class CalculateTrajectories
{
	public:
	int MAX_NODES;
	int rows,cols;
	int MSTedges;
	int node_count;
	vector<Edge> MSTvector;
	//map<int,Edge> allEdges;
	map<Edge,int,edgeCmp> allEdges;
	vector<hash_set<int> > nodes;
	vector<PathNode> PathSequence;
	//vector<PathNode> FinalPathSequence;

	CalculateTrajectories(int r,int c,vector<Edge>& MST)
	{
		MAX_NODES = 4*r*c;
		rows = r;
		cols = c;
		node_count = 0;
		MSTedges = MST.size();
		nodes.resize(MAX_NODES);
		
		/*for(int i = 0;i < MST.size();i++)
			MSTvector.push_back(MST[i]);*/
		MSTvector.assign(MST.begin(),MST.end());
	}

	void initializeGraph(cv::Mat& data,bool connect)
	{
		//std::cout << __FILE__ << __LINE__ << std::endl;
		for(int i = 0;i < 2*rows;i++){
			for(int j = 0;j < 2*cols;j++){
				if(data.at<int>(i,j)){
					if(i > 0 && data.at<int>(i-1,j)){
						AddToAllEdges(i*2*cols+j,(i-1)*2*cols+j,1);
					}
					if(i < 2*rows-1 && data.at<int>(i+1,j))
					{
						AddToAllEdges(i*2*cols+j,(i+1)*2*cols+j,1);
					}
					if(j > 0 && data.at<int>(i,j-1))
					{
						AddToAllEdges(i*2*cols+j,i*2*cols+j-1,1);
					}
					if(j < 2*cols-1 && data.at<int>(i,j+1))
					{
						AddToAllEdges(i*2*cols+j,i*2*cols+j+1,1);
					}

					if(!connect){
						if(i > 0 && j > 0 && data.at<int>(i-1,j-1))
						{
							AddToAllEdges(i*2*cols+j,(i-1)*2*cols+j-1,1);
						}
						if(i < 2*rows-1 && j < 2*cols-1 && data.at<int>(i+1,j+1))
						{
							AddToAllEdges(i*2*cols+j,(i+1)*2*cols+j+1,1);
						}
						if(i > 2*rows-1 && j > 0 && data.at<int>(i+1,j-1))
						{
							AddToAllEdges(i*2*cols+j,(i+1)*2*cols+j-1,1);
						}
						if(i > 0 && j < 2*cols-1 && data.at<int>(i-1,j+1))
						{
							AddToAllEdges(i*2*cols+j,(i-1)*2*cols+j+1,1);
						}
					}

				}
			}
		}
		//std::cout << __FILE__ << __LINE__ << std::endl;
		/*for(int i = 0;i < MAX_NODES;i++){
			if(nodes[i].size() > 0 ){
				std::cout << "CalculateTrajectories_every_nodes_size:" << nodes[i].size() << std::endl;
				node_count++;
			}
		}
		std::cout << "CalculateTrajectories_node_count:" << node_count << std::endl;*/
		std::cout << "CalculateTrajectories_allEdges_size:" << allEdges.size() << std::endl;
	}
	
	void AddToAllEdges(int from,int to,int cost)
	{
		static uint64 key = 0;
		//key++;
//std::cout << __FILE__ << __LINE__ << std::endl;
		Edge e(from,to,cost);
		allEdges.insert(pair<Edge,int>(e,key));
//std::cout << __FILE__ << __LINE__ << std::endl;		
		//if(nodes[from].size() == 0)
			//nodes[from].resize(8*MAX_NODES);
//std::cout << __FILE__ << __LINE__ << std::endl;		
		nodes[from].insert(to);
//std::cout << __FILE__ << __LINE__ << std::endl;
		//if(nodes[to].size() == 0)
			//nodes[to].resize(8*MAX_NODES);
//std::cout << __FILE__ << __LINE__ << std::endl;		
		nodes[to].insert(from);
//std::cout << __FILE__ << __LINE__ << std::endl;
		key++;
	}
	void RemoveTheAppropriateEdges()
	{
		//printf("start RemoveTheAppropriateEdges!!!\n");
		int alpha,maxN,minN;
		Edge eToRemove,eToRemoveMirr,eToRemove2,eToRemove2Mirr;
		
		//printf("MSTedges:%d\n",MSTedges);
		for(int i = 0;i < MSTedges;i++){
			Edge e = MSTvector[i];
			maxN = std::max(e.from,e.to);
			minN = std::min(e.from,e.to);

			//printf("e(from:%d-to:%d)\n",e.from,e.to);

			if(std::abs(e.from-e.to) == 1){
				alpha = (4*minN+3) - 2*(maxN%cols);
				Edge eToRemoveTemp(alpha,alpha+2*cols,1);
				eToRemove.from = eToRemoveTemp.from;
				eToRemove.to = eToRemoveTemp.to;
				eToRemove.cost = eToRemoveTemp.cost;
				Edge eToRemoveMirrTemp(alpha+2*cols,alpha,1);

				eToRemoveMirr.from = eToRemoveMirrTemp.from;
				eToRemoveMirr.to = eToRemoveMirrTemp.to;
				eToRemoveMirr.cost = eToRemoveMirrTemp.cost;
				
				Edge eToRemove2Temp(alpha+1,alpha+1+2*cols,1);
				
				eToRemove2.from = eToRemove2Temp.from;
				eToRemove2.to = eToRemove2Temp.to;
				eToRemove2.cost = eToRemove2Temp.cost;

				Edge eToRemove2MirrTemp(alpha+1+2*cols,alpha+1,1);
				eToRemove2Mirr.from = eToRemove2MirrTemp.from;
				eToRemove2Mirr.to = eToRemove2MirrTemp.to;
				eToRemove2Mirr.cost = eToRemove2MirrTemp.cost;
				
			}else{
				alpha = (4*minN+2*cols) - 2*(maxN%cols);
				Edge eToRemoveTemp(alpha,alpha+1,1);
				eToRemove.from = eToRemoveTemp.from;
				eToRemove.to = eToRemoveTemp.to;
				eToRemove.cost = eToRemoveTemp.cost;
				Edge eToRemoveMirrTemp(alpha+1,alpha,1);

				eToRemoveMirr.from = eToRemoveMirrTemp.from;
				eToRemoveMirr.to = eToRemoveMirrTemp.to;
				eToRemoveMirr.cost = eToRemoveMirrTemp.cost;
				
				Edge eToRemove2Temp(alpha+2*cols,alpha+1+2*cols,1);
				
				eToRemove2.from = eToRemove2Temp.from;
				eToRemove2.to = eToRemove2Temp.to;
				eToRemove2.cost = eToRemove2Temp.cost;

				Edge eToRemove2MirrTemp(alpha+1+2*cols,alpha+2*cols,1);
				eToRemove2Mirr.from = eToRemove2MirrTemp.from;
				eToRemove2Mirr.to = eToRemove2MirrTemp.to;
				eToRemove2Mirr.cost = eToRemove2MirrTemp.cost;
			}

			if(allEdges_map_count(eToRemove))
				SafeRemoveEdge(eToRemove);
			if(allEdges_map_count(eToRemoveMirr))
				SafeRemoveEdge(eToRemoveMirr);
			if(allEdges_map_count(eToRemove2))
				SafeRemoveEdge(eToRemove2);
			if(allEdges_map_count(eToRemove2Mirr))
				SafeRemoveEdge(eToRemove2Mirr);
		}
	}
	
	bool allEdges_map_count(Edge& e)
	{
		/*auto iter = allEdges.begin();
		bool flag = false;

		while(iter != allEdges.end()){
			if((iter->second.from == e.from) && (iter->second.to == e.to) && (iter->second.cost == e.cost)){
				flag = true;
				break;
			}
			iter++;
		}
		*/
		if(allEdges.count(e))
			return true;
		else
			return false;
	}
	void SafeRemoveEdge(Edge& curEdge)
	{
		//printf("delete e(from:%d-to:%d)\n",curEdge.from,curEdge.to);
		/*static unsigned int delete_count = 0;

		delete_count++;

		auto iter = allEdges.begin();
		bool flag = false;

		while(iter != allEdges.end()){
			if((iter->second.from == curEdge.from) && (iter->second.to == curEdge.to) && (iter->second.cost == curEdge.cost)){
				allEdges.erase(iter++);
				flag = true;
				break;
			}
			else
				++iter;
		}
		
		if(!flag)
			printf("map<int,Edge> should have contained this element!!!\n");		
		*/
		map<Edge,int>::iterator iter;	
		iter = allEdges.find(curEdge);

		allEdges.erase(iter);

		nodes[curEdge.from].erase(curEdge.to);
		nodes[curEdge.to].erase(curEdge.from);

		//printf("delete_count:%d\n",delete_count);
	}
	
	void CalculatePathsSequence(int start_node)
	{
		std::cout << "start CalculatePathsSequence" << std::endl;
		int currentNode = start_node;
		hash_set<int> RemovedNodes;

		int prevNode,i,j,previ,prevj,offset;

		vector<int> movement;
		movement.push_back(2*cols);
		movement.push_back(-1);
		movement.push_back(-2*cols);
		movement.push_back(1);

		bool found = false;

		prevNode = 0;
		for(int idx = 0;idx < 4;idx++){
			if(nodes[currentNode].count(currentNode + movement[idx])){
				prevNode = currentNode + movement[idx];
				found = true;
				break;
			}
		}
	
		if(!found){
			//printf("CalculatePathsSequence not found_1\n");
			return;
		}
		
		do{
			RemovedNodes.insert(currentNode);
			offset = indexOf(movement,prevNode-currentNode);
			
			prevNode = currentNode;

			found = false;
			for(int idx = 0;idx < 4;idx++){
				if(nodes[prevNode].count(prevNode+movement[(idx+offset)%4]) && !RemovedNodes.count(prevNode+movement[(idx+offset)%4])){
					currentNode = prevNode + movement[(idx+offset)%4];
					found = true;
					break;
				}
			}
			
		if(!found){
			//printf("CalculatePathsSequence not found_2\n");
			break;
		}

		if(nodes[currentNode].count(prevNode)){nodes[currentNode].erase(prevNode);}
		if(nodes[prevNode].count(currentNode)){nodes[prevNode].erase(currentNode);}

		i = currentNode/(2*cols);
		j = currentNode%(2*cols);

		previ = prevNode/(2*cols);
		prevj = prevNode%(2*cols);

		PathNode pn;

		pn.pre_i = previ;
		pn.pre_j = prevj;
		pn.i = i;
		pn.j = j;

		//std::cout << "path_node(from:" << pn.pre_i << ","  << pn.pre_j << "to:" << pn.i << "," << pn.j << ")" << std::endl;
		PathSequence.push_back(pn);
		}while(true);

		std::cout << "PathSequence.size:" << PathSequence.size() << std::endl;

	}

	int indexOf(vector<int>& mm,int delta)
	{
		int res;
		for(int res = 0;res < mm.size();res++)
			if(delta == mm[res])
				return res;
	}

	};

/*int compare_edge(Edge& e1,Edge& e2)
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
*/
	void calculateMSTs(cv::Mat& region_,vector<Edge>& vec_,int nr_)
	{
		int rows = region_.rows;
		int cols = region_.cols;

		//Kruskal k(rows,cols);
    for (int r = 0;r < nr_;r++){
		  Kruskal k(rows,cols);
		std::cout << __FILE__ << __LINE__ << std::endl;
		  k.initializeGraph(region_,true);

		std::cout << __FILE__ << __LINE__ << std::endl;
		  k.performKruskal();

		std::cout << __FILE__ << __LINE__ << std::endl;

		//std::cout << __FILE__ << __LINE__ << std::endl;
			//k.printFinalEdges();
		  /*MSTs.add(k.getAllNewEdges());*/

		for(int i =0;i < k.allNewEdges.size();i++)
			vec_.push_back(k.allNewEdges[i]);
		}

		/*for(int i =0;i < k.allNewEdges.size();i++)
			vec_.push_back(k.allNewEdges[i]);*/
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

	void CalcRealBinaryReg(cv::Mat& BinaryRobotRegion,cv::Mat& RealBinaryRobotRegion)
	{
		int rows = BinaryRobotRegion.rows;
		int cols = BinaryRobotRegion.cols;

		for(int i = 0;i < 2*rows;i++){
			for(int j = 0;j < 2*cols;j++){
				RealBinaryRobotRegion.at<int>(i,j) = BinaryRobotRegion.at<int>(i/2,j/2);
			}
		}
	}


void MapToWorld(
    double resolution, double origin_x, double origin_y, unsigned int mx,
    unsigned int my, double *wx, double *wy) {
  *wx = origin_x + (mx + 0.5) * resolution;
  *wy = origin_y + (my + 0.5) * resolution;
}

bool WorldToMap(
    double resolution, double origin_x, double origin_y, unsigned int size_x,
    unsigned int size_y, double wx, double wy, int *mx, int *my) {
  if (wx < origin_x || wy < origin_y)
    return false;

  *mx = static_cast<int>((wx - origin_x) / resolution);
  *my = static_cast<int>((wy - origin_y) / resolution);

  if (*mx < size_x && *my < size_y)
    return true;

  return false;
}

 void drawArrow(cv::Mat& img, cv::Point pStart, cv::Point pEnd, int len, int alpha,             
     cv::Scalar& color, int thickness, int lineType)
	{    
		const double PI = 3.1415926;    
		cv::Point arrow;     
		double angle = atan2((double)(pStart.y - pEnd.y), (double)(pStart.x - pEnd.x));  

		cv::line(img, pStart, pEnd, color, thickness, lineType);   

		arrow.x = pEnd.x + len * cos(angle + PI * alpha / 180);     
		arrow.y = pEnd.y + len * sin(angle + PI * alpha / 180);  
		cv::line(img, pEnd, arrow, color, thickness, lineType);   

		arrow.x = pEnd.x + len * cos(angle - PI * alpha / 180);     
		arrow.y = pEnd.y + len * sin(angle - PI * alpha / 180);    
		cv::line(img, pEnd, arrow, color, thickness, lineType);
	}

bool isFlectionPoint(geometry_msgs::PoseStamped& pre_pose,geometry_msgs::PoseStamped& cur_pose,geometry_msgs::PoseStamped& nex_pose)
{
	float pre_x = pre_pose.pose.position.x;
	float pre_y = pre_pose.pose.position.y;

	float cur_x = cur_pose.pose.position.x;
	float cur_y = cur_pose.pose.position.y;

	float nex_x = nex_pose.pose.position.x;
	float nex_y = nex_pose.pose.position.y;

	//std::cout << "pre_x:" << pre_x << std::endl;
	//std::cout << "pre_y:" << pre_y << std::endl;

	//std::cout << "cur_x:" << cur_x << std::endl;
	//std::cout << "cur_y:" << cur_y << std::endl;

	//std::cout << "nex_x:" << nex_x << std::endl;
	//std::cout << "nex_y:" << nex_y << std::endl;

	//std::cout << "isFlectionPoint:" << fabs((nex_y-pre_y)*(cur_x-pre_x)-(nex_x-pre_x)*(cur_y-pre_y)) << std::endl;
	static int count_flection_point = 0;
	if(fabs((nex_y-pre_y)*(cur_x-pre_x)-(nex_x-pre_x)*(cur_y-pre_y)) < 0.001){
		//std::cout << "isFlectionPoint:" << fabs((nex_y-pre_y)*(cur_x-pre_x)-(nex_x-pre_x)*(cur_y-pre_y)) << std::endl;
		return false;
	}
	else{
		//count_flection_point++;
		//std::cout << "count_flection_point:" << count_flection_point << std::endl;
		//std::cout << "isFlectionPoint:" << fabs((nex_y-pre_y)*(cur_x-pre_x)-(nex_x-pre_x)*(cur_y-pre_y)) << std::endl;
		return true;
	}
}
bool CoveragePlanService(
    scale_map::GetCoveragePathScaleMap::Request &req,     
    scale_map::GetCoveragePathScaleMap::Response &resp) { 
  if (req.erosion_radius < 0) {
    ROS_ERROR("erosion_radius < 0");
    return false;
  }
  if (req.robot_radius < 0) {
    ROS_ERROR("robot_radius < 0");
    return false;
  }
  if (req.occupancy_threshold < 0 || req.occupancy_threshold > 100) {
    ROS_ERROR("occupancy_threshold out of range 0~100");
    return false;
  }
  if (req.start.header.frame_id != req.map.header.frame_id) {
    ROS_ERROR("start's frame_id != map's frame_id");
    return false;
  }
  /*if (req.goal.header.frame_id != req.map.header.frame_id) {
    ROS_ERROR("goal's frame_id != map's frame_id");
    return false;
  }*/
  if (req.map.info.resolution < 0) {
    ROS_ERROR("invalid map: resolution < 0");
    return false;
  }
  if (req.map.info.width * req.map.info.height < 1) {
    ROS_ERROR("invalid map: width * height < 1");
    return false;
  }
  if (req.map.info.width * req.map.info.height != req.map.data.size()) {
    ROS_ERROR("invalid map: width * height != data size");
    return false;
  }

  //  build start and goal
  cv::Point start;
  double start_x_world;
  double start_y_world;
  double start_x_origin = req.start.pose.position.x;
  double start_y_origin = req.start.pose.position.y;

  cv::Mat origin_rotate_mat_inv(3,3,CV_64FC1);
  cv::Mat origin_rotate_mat;
  origin_rotate_mat_inv.at<double>(0,0) = req.transform[0];
  origin_rotate_mat_inv.at<double>(0,1) = req.transform[1];
  origin_rotate_mat_inv.at<double>(0,2) = req.transform[2];
  origin_rotate_mat_inv.at<double>(1,0) = req.transform[3];
  origin_rotate_mat_inv.at<double>(1,1) = req.transform[4];
  origin_rotate_mat_inv.at<double>(1,2) = req.transform[5];
  origin_rotate_mat_inv.at<double>(2,0) = 0;
  origin_rotate_mat_inv.at<double>(2,1) = 0;
  origin_rotate_mat_inv.at<double>(2,2) = 1;

  origin_rotate_mat = origin_rotate_mat_inv.inv();
  double m00_ = origin_rotate_mat.at<double>(0,0);
  double m01_ = origin_rotate_mat.at<double>(0,1);
  double m02_ = origin_rotate_mat.at<double>(0,2);
  double m10_ = origin_rotate_mat.at<double>(1,0);
  double m11_ = origin_rotate_mat.at<double>(1,1);
  double m12_ = origin_rotate_mat.at<double>(1,2);
  
  std::cout << "origin_rotate_mat:" << origin_rotate_mat << std::endl;
  start_x_world = (((start_x_origin-req.map_origin_x)/req.map_resolution)*m00_+\
                  ((start_y_origin-req.map_origin_y)/req.map_resolution)*m01_+m02_)*req.map_resolution;
  start_y_world = (((start_x_origin-req.map_origin_x)/req.map_resolution)*m10_+\
                  ((start_y_origin-req.map_origin_y)/req.map_resolution)*m11_+m12_)*req.map_resolution;
  
  start_x_world += req.map_origin_x;
  start_y_world += req.map_origin_y;

  ROS_INFO("start_x_grid,start_y_grid:%f,%f",(start_x_origin-req.map_origin_x)/req.map_resolution,\
                                             (start_y_origin-req.map_origin_y)/req.map_resolution);
  ROS_INFO("start_x,start_y:%f,%f",req.start.pose.position.x,req.start.pose.position.y);
  ROS_INFO("rotate_start_x,rotate_start_y:%f,%f",start_x_world,start_y_world);
  if (false == WorldToMap(
                   req.map.info.resolution, req.map.info.origin.position.x,
                   req.map.info.origin.position.y, req.map.info.width,
                   req.map.info.height, start_x_world,
                   start_y_world, &start.x, &start.y)) {
    ROS_ERROR("Invalid start. Out of map.");
    return false;
  }

  /*if (false == WorldToMap(
                   req.map.info.resolution, req.map.info.origin.position.x,
                   req.map.info.origin.position.y, req.map.info.width,
                   req.map.info.height, req.start.pose.position.x,
                   req.start.pose.position.y, &start.x, &start.y)) {
    ROS_ERROR("Invalid start. Out of map.");
    return false;
  }*/
  //  build map
  for (int i = 0; i < req.map.data.size(); ++i) {
    if (-1 == req.map.data[i]) {  
      req.map.data[i] = 100;
    }
  }
  cv::Mat map(
      req.map.info.height, req.map.info.width, CV_8UC1, req.map.data.data());
	/*for(int i = 0;i < map.rows;i++){
		for(int j = 0;j < map.cols;j++){
			printf("%d--",map.at<unsigned char>(i,j));
			if(j == map.cols-1)
				printf("\n");
		}
	}*/

	//std::cout << "map:" << map << std::endl;
  //  binarization
  cv::Mat binarization;
  cv::threshold(
      map, binarization, req.occupancy_threshold, 255, cv::THRESH_BINARY_INV);

	//std::cout << __FILE__ << __LINE__ << std::endl;
	//std::cout << "binarization:" << binarization << std::endl;
  //  erosion
		ros::Time begin10 = ros::Time ::now();
  cv::Mat erosion, element;
  element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
  /*cv::erode(
      binarization, erosion, element, cv::Point(-1, -1),
      (req.erosion_radius + req.map.info.resolution - 0.01) /
          req.map.info.resolution);
	*/
  cv::erode(binarization, erosion, element);
		ros::Time end10 = ros::Time::now();
		std::cout << "time10 cost:" << (end10-begin10).toSec() << std::endl;
	//reconstruct map data
		ros::Time begin8 = ros::Time ::now();
	for(int i = 0;i < erosion.rows;i++){
		for(int j = 0;j < erosion.cols;j++){
			if(erosion.at<unsigned char>(i,j) == 0)
				erosion.at<unsigned char>(i,j) = 128;
			else if(erosion.at<unsigned char>(i,j) == 255)
				erosion.at<unsigned char>(i,j) = 0;
		}
	}
	erosion.at<unsigned char>(start.y,start.x) = 255;
	//std::cout << "erosion:" << erosion << std::endl;
		ros::Time end8 = ros::Time::now();
		std::cout << "time8 cost:" << (end8-begin8).toSec() << std::endl;
  //coverage path plan
  std::deque<cv::Point> path;

	if(erosion.empty())
	{
		std::cout << "map open fail.............." << std::endl;
		return false;
	}
	else{
		std::cout << "map_width:" << erosion.cols << std::endl;
		std::cout << "map_height:" << erosion.rows << std::endl;
		
		ros::Time begin7 = ros::Time ::now();
		environment_grid_.create(erosion.rows,erosion.cols,CV_32SC1);
		getGraphics(erosion,environment_grid_);
		ros::Time end7 = ros::Time::now();
		std::cout << "time7 cost:" << (end7-begin7).toSec() << std::endl;
		//std::cout << "environment_grid_:" << environment_grid_ << std::endl;
	}

	std::cout << "start Darp algorithm......\n" << std::endl;
	ConnectComponent cc(environment_grid_.rows,environment_grid_.cols);
	std::cout << "cc.rows:" << cc.rows << std::endl;
	std::cout << "cc.cols:" << cc.cols << std::endl;
	std::cout << "cc.MAX_LABELS:" << cc.MAX_LABELS << std::endl;

	ros::Time begin6 = ros::Time ::now();
	binary_grid_.create(cc.rows,cc.cols,CV_32SC1);
	makeGridBinary(environment_grid_,binary_grid_);
	ros::Time end6 = ros::Time::now();
	std::cout << "time6 cost:" << (end6-begin6).toSec() << std::endl;

	//std::cout << __FILE__ << __LINE__ << std::endl;
	//print binary_grid
	/*printf("binary_grid_\n");
	for(int i = 0;i < binary_grid_.rows;i++){
		for(int j = 0;j < binary_grid_.cols;j++){
			printf("%d--",binary_grid_.at<int>(i,j));

			if(j == binary_grid_.cols-1)
				printf("\n");
		}
	}
	*/
	ros::Time begin5 = ros::Time ::now();
	label_2d_.create(cc.rows,cc.cols,CV_32SC1);
	cc.compactLabeling(binary_grid_,label_2d_,binary_grid_.rows,binary_grid_.cols,true);
	ros::Time end5 = ros::Time::now();
	std::cout << "time5 cost:" << (end5-begin5).toSec() << std::endl;

	//print label_2d
	/*printf("label_2d_\n");
	for(int i = 0;i < label_2d_.rows;i++){
		for(int j = 0;j < label_2d_.cols;j++){
			printf("%d--",label_2d_.at<int>(i,j));

			if(j == label_2d_.cols-1)
				printf("\n");
		}
	}
	*/
	if(cc.getMaxLabel() > 1)
	{printf("The environment grid MUST not have unreachable and/or closed shape regions\n\n");}

	DARP p(environment_grid_.rows,environment_grid_.cols,environment_grid_);

	std::cout << "nr:" << p.getNr() << std::endl;
	std::cout << "obs:" << p.getNumOB() << std::endl;
	if(p.getNr() < 1)
	{printf("Please define at least one robot \n\n");};

	//std::cout << "BinaryRobotRegions:" << p.BinrayRobotRegions << std::endl;

	//std::cout << __FILE__ << __LINE__ << std::endl;
	ros::Time begin4 = ros::Time ::now();
	vector<Edge> vec_edge;
	calculateMSTs(p.BinrayRobotRegions,vec_edge,p.nr);
	ros::Time end4 = ros::Time::now();
	std::cout << "time4 cost:" << (end4-begin4).toSec() << std::endl;
	
	//std::cout << __FILE__ << __LINE__ << std::endl;
	ros::Time begin9 = ros::Time ::now();
	CalculateTrajectories ct(p.BinrayRobotRegions.rows,p.BinrayRobotRegions.cols,vec_edge);
	ros::Time end9 = ros::Time::now();
	std::cout << "time9 cost:" << (end9-begin9).toSec() << std::endl;

	//std::cout << __FILE__ << __LINE__ << std::endl;
	ros::Time begin3 = ros::Time ::now();
	cv::Mat RealBinaryRobotRegions(2*p.BinrayRobotRegions.rows,2*p.BinrayRobotRegions.cols,CV_32SC1);
	CalcRealBinaryReg(p.BinrayRobotRegions,RealBinaryRobotRegions);
	ros::Time end3 = ros::Time::now();
	std::cout << "time3 cost:" << (end3-begin3).toSec() << std::endl;
	
	//std::cout << __FILE__ << __LINE__ << std::endl;
	ros::Time begin2 = ros::Time ::now();
	ct.initializeGraph(RealBinaryRobotRegions,true);
	ros::Time end2 = ros::Time::now();
	std::cout << "time2 cost:" << (end2-begin2).toSec() << std::endl;

	//std::cout << __FILE__ << __LINE__ << std::endl;
	ros::Time begin1 = ros::Time ::now();
	ct.RemoveTheAppropriateEdges();
	ros::Time end1 = ros::Time::now();
	std::cout << "time1 cost:" << (end1-begin1).toSec() << std::endl;

	ros::Time begin = ros::Time ::now();
	ct.CalculatePathsSequence(4*p.RobotsInit[0].y*ct.cols+2*p.RobotsInit[0].x);
	ros::Time end = ros::Time::now();

	std::cout << "time cost:" << (end-begin).toSec() << std::endl;
	//std::cout << __FILE__ << __LINE__ << std::endl;
  // coordinate mapping
    geometry_msgs::PoseStamped current_pose;
	geometry_msgs::PoseStamped next_pose;
    cv::Point current,next;
	int index = 0;

	std::vector<geometry_msgs::PoseStamped> temp_path_node;

	//start_node
	current.x = ct.PathSequence[0].pre_j;
	current.y = ct.PathSequence[0].pre_i;

  MapToWorld(
      req.map.info.resolution/2.0, req.map.info.origin.position.x,
      req.map.info.origin.position.y, current.x, current.y, &current_pose.pose.position.x,
      &current_pose.pose.position.y);

	temp_path_node.push_back(current_pose);

  while (index < ct.PathSequence.size() ) {
    //current.x = ct.PathSequence[index].pre_j;
		//current.y = ct.PathSequence[index].pre_i;

    next.x = ct.PathSequence[index].j;
		next.y = ct.PathSequence[index].i;
		
    //  fill position
    /*MapToWorld(
        req.map.info.resolution/2.0, req.map.info.origin.position.x,
        req.map.info.origin.position.y, current.x, current.y, &current_pose.pose.position.x,
        &current_pose.pose.position.y);*/

    MapToWorld(
        req.map.info.resolution/2.0, req.map.info.origin.position.x,
        req.map.info.origin.position.y, next.x, next.y, &next_pose.pose.position.x,
        &next_pose.pose.position.y);

    //resp.plan.poses.push_back(current_pose);
    //resp.plan.poses.push_back(next_pose);

		//temp_path_node.push_back(current_pose);
		temp_path_node.push_back(next_pose);
		index++;
  }

    double map_origin_x = req.map_origin_x;
    double map_origin_y = req.map_origin_y;
    double map_resolution = req.map_resolution;
    
    double m00 = req.transform[0];
    double m01 = req.transform[1];
    double m02 = req.transform[2];
    double m10 = req.transform[3];
    double m11 = req.transform[4];
    double m12 = req.transform[5];

    geometry_msgs::PoseStamped stt_pose = temp_path_node[0];
    double x_stt = stt_pose.pose.position.x;
    double y_stt = stt_pose.pose.position.y;

    stt_pose.pose.position.x = (((x_stt-map_origin_x)/map_resolution)*m00+((y_stt-map_origin_y)/map_resolution)*m01+m02)\
                                                                                                            *map_resolution;
    stt_pose.pose.position.y = (((x_stt-map_origin_x)/map_resolution)*m10+((y_stt-map_origin_y)/map_resolution)*m11+m12)\
                                                                                                            *map_resolution;
    stt_pose.pose.position.x += map_origin_x;
    stt_pose.pose.position.y += map_origin_y;

    double x_delta = 0;
    double y_delta = 0;

    stt_pose.pose.position.x += x_delta;
    stt_pose.pose.position.y += y_delta;
    
    ROS_INFO("stt_start_x,stt_start_y:%f,%f",stt_pose.pose.position.x,stt_pose.pose.position.y);
    //ROS_INFO("x_delta,y_delta:%f,%f",x_delta,y_delta);
	resp.plan.poses.push_back(stt_pose);

	geometry_msgs::PoseStamped pre_pose;
	geometry_msgs::PoseStamped cur_pose;
	geometry_msgs::PoseStamped nex_pose;

	for(int i = 1;i < temp_path_node.size()-1;i++){
		pre_pose = temp_path_node[i-1];
		cur_pose = temp_path_node[i];
		nex_pose = temp_path_node[i+1];

		if(isFlectionPoint(pre_pose,cur_pose,nex_pose)){
            double x_rotate = cur_pose.pose.position.x;
            double y_rotate = cur_pose.pose.position.y;

            cur_pose.pose.position.x = (((x_rotate-map_origin_x)/map_resolution)*m00+((y_rotate-map_origin_y)/map_resolution)*m01+m02)\
                                                                                                                        *map_resolution;
            cur_pose.pose.position.y = (((x_rotate-map_origin_x)/map_resolution)*m10+((y_rotate-map_origin_y)/map_resolution)*m11+m12)\
                                                                                                                        *map_resolution;
            cur_pose.pose.position.x += map_origin_x;
            cur_pose.pose.position.y += map_origin_y;

            cur_pose.pose.position.z = 0;

            cur_pose.pose.position.x += x_delta;
            cur_pose.pose.position.y += y_delta;

			resp.plan.poses.push_back(cur_pose);

		}
	}
    
    geometry_msgs::PoseStamped lst_pose = temp_path_node[temp_path_node.size()-1];
    double x_lst = lst_pose.pose.position.x;
    double y_lst = lst_pose.pose.position.y;

    lst_pose.pose.position.x = (((x_lst-map_origin_x)/map_resolution)*m00+((y_lst-map_origin_y)/map_resolution)*m01+m02)\
                                                                                                                 *map_resolution;
    lst_pose.pose.position.y = (((x_lst-map_origin_x)/map_resolution)*m10+((y_lst-map_origin_y)/map_resolution)*m11+m12)\
                                                                                                                 *map_resolution;
	lst_pose.pose.position.x += map_origin_x;
    lst_pose.pose.position.y += map_origin_y;

    lst_pose.pose.position.x += x_delta;
    lst_pose.pose.position.y += y_delta;

    resp.plan.poses.push_back(lst_pose);

	environment_grid_.release();
	binary_grid_.release();
	label_2d_.release();

  return true;
}

}  // namespace darp_path_plan

int main(int argc, char **argv) {
  ros::init(argc, argv, "darp_path_plan_node");

  ros::NodeHandle private_nh("~");

  //  advertise a service for getting a coverage plan
  ros::ServiceServer make_coverage_plan_srv = private_nh.advertiseService(
      "/sweeper/scale_map_make_coverage_plan",
      scale_map::CoveragePlanService);

  ROS_INFO("Ready to make coverage plan.");

  ros::spin();
}
