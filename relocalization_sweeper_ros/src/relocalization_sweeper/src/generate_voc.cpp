#include "DBoW3.h"
#include "Vocabulary.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <vector>
#include <string>

#include "ros/ros.h"

using namespace cv;
using namespace std;


int main( int argc, char** argv )
{
    ros::init(argc,argv,"generate_voc_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    string dataset_dir;
	pn.param<string>("dataset_dir", dataset_dir, "../voc_data/");
    ROS_INFO("image data direction:%s",dataset_dir.c_str());
    string voc_dir;
	pn.param<string>("voc_dir", voc_dir, "../voc/");
    ROS_INFO("voc direction:%s",voc_dir.c_str());

    ifstream fin ( dataset_dir+"/voc_data.txt" );
    if ( !fin )
    {
        ROS_INFO("Read Data Fail...");
        return 1;
    }
    
    //read image
    vector<string> rgb_files;
    string rgb_file;
    while (getline(fin,rgb_file) && ros::ok())
    {
        rgb_files.push_back ( dataset_dir+rgb_file );
    }
    fin.close();

    for(int i = 0;i < rgb_files.size();i++)
        ROS_INFO("%s",rgb_files[i].c_str());

    ROS_INFO("rgb_files_size:%d",(int)rgb_files.size());    
    ROS_INFO("Generating Features ... ");
    vector<Mat> descriptors;
    //Ptr< Feature2D > detector = ORB::create();
    int num_of_features = 500;
    double scale_factor = 1.2;
    int level_pyramid = 5;
    cv::Ptr<cv::ORB> detector = cv::ORB::create(num_of_features,scale_factor,level_pyramid);
    
    //extract orb feature
    for ( string rgb_file:rgb_files )
    {
        Mat image = imread(rgb_file);

        if(image.empty())
            continue;

        vector<KeyPoint> keypoints; 
        Mat descriptor;
        detector->detectAndCompute( image, Mat(), keypoints, descriptor );
        descriptors.push_back( descriptor );
        //ROS_INFO("Extracting Features From Image:%d ",index++);
    }
    ROS_INFO("Extract Total :%d",(int)(descriptors.size()*500));
    
    //create vocabulary 
    ROS_INFO("Creating Vocabulary,Please Wait...");
    DBoW3::Vocabulary vocab(10,7);
    vocab.create( descriptors );
    cout << "Vocabulary Info: " << vocab << endl;
    vocab.save(voc_dir+"voc.bin");
    ROS_INFO("Done");
    
    return 0;
}
