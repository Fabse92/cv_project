#include "egbis.h"
#include "VOCUS2.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include "object_candidates/Objectcandidates.h"
#include "object_candidates/ArrayImages.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>
#include <sstream>



//get most salient region
vector<Point> seededRegionGrowing(Mat& salmap, Mat iorMap, Mat& considered, double& ma, double& meanSaliency, float candidateThreshold){
	Point p_ma;
	minMaxLoc(iorMap, nullptr, &ma, nullptr, &p_ma);
  
	vector<Point> msr;
	msr.push_back(p_ma);
	int pos = 0;
	float thresh = candidateThreshold*ma;
	
	considered = Mat::zeros(salmap.size(), CV_8U);
	considered.at<uchar>(p_ma) = 1;
	meanSaliency = ma;

	while(pos < (int)msr.size()){
		int r = msr[pos].y;
		int c = msr[pos].x;
		for(int dr = -1; dr <= 1; dr++){
			for(int dc = -1; dc <= 1; dc++){
				if(dc == 0 && dr == 0) continue;
				if(considered.ptr<uchar>(r+dr)[c+dc] != 0) continue;
				if(r+dr < 0 || r+dr >= salmap.rows) continue;
				if(c+dc < 0 || c+dc >= salmap.cols) continue;
				
				if(salmap.ptr<uchar>(r+dr)[c+dc] >= thresh){
					msr.push_back(Point(c+dc, r+dr));
					considered.ptr<uchar>(r+dr)[c+dc] = 1;
					meanSaliency += salmap.ptr<uchar>(r+dr)[c+dc];
				}
			}
		}
		pos++;
	}
	
	meanSaliency = (meanSaliency / msr.size());
	return msr;
}

void ior(Mat& salmap, float iorThreshold){
	double ma;
	Point p_ma;
	minMaxLoc(salmap, nullptr, &ma, nullptr, &p_ma);
  
	vector<Point> msr;
	msr.push_back(p_ma);
	int pos = 0;
	float thresh = iorThreshold*ma;
	
	Mat salmapCopy = salmap.clone();
	Mat considered = Mat::zeros(salmap.size(), CV_8U);
	considered.at<uchar>(p_ma) = 1;
	Mat parentSaliency = Mat::zeros(salmap.size(), CV_8U);
	parentSaliency.at<uchar>(p_ma) = ma;	
	salmap.at<uchar>(p_ma) = 0;
	
	while(pos < (int)msr.size()){
		int r = msr[pos].y;
		int c = msr[pos].x;
		for(int dr = -1; dr <= 1; dr++){
			for(int dc = -1; dc <= 1; dc++){
				if(dc == 0 && dr == 0) continue;
				if(considered.ptr<uchar>(r+dr)[c+dc] != 0) continue;
				if(r+dr < 0 || r+dr >= salmap.rows) continue;
				if(c+dc < 0 || c+dc >= salmap.cols) continue;
				
				if((salmap.ptr<uchar>(r+dr)[c+dc] >= thresh) || (salmap.ptr<uchar>(r+dr)[c+dc] < parentSaliency.ptr<uchar>(r+dr)[c+dc])){
					msr.push_back(Point(c+dc, r+dr));
					considered.ptr<uchar>(r+dr)[c+dc] = 1;
					for(int dr2 = -1; dr2 <= 1; dr2++){
			      for(int dc2 = -1; dc2 <= 1; dc2++){
			        if(r+dr+dr2 < 0 || r+dr+dr2 >= salmap.rows) continue;
				      if(c+dc+dc2 < 0 || c+dc+dc2 >= salmap.cols) continue;
			        if(salmapCopy.ptr<uchar>(r+dr)[c+dc] > parentSaliency.ptr<uchar>(r+dr+dr2)[c+dc+dc2]){
					      parentSaliency.ptr<uchar>(r+dr+dr2)[c+dc+dc2] = salmapCopy.ptr<uchar>(r+dr)[c+dc];
					    }
			      }
		      }					
			    salmap.ptr<uchar>(r+dr)[c+dc] = 0;					
				}
			}
		}
		pos++;
	}
}

double areaOverUnion(Mat mat1, Mat mat2, bool overlap = false){
  /*
  double area1, area2, areaint, areaunion;
  cv::Rect intersect; 

  area1 = rect1.height * rect1.width;
  area2 = rect2.height * rect2.width;
  intersect = rect1 & rect2;
  areaint = intersect.height * intersect.width;
  areaunion = area1 + area2 - areaint;
  
  if (overlap){
    return areaint/area2;
  }
  return areaint/areaunion;*/
  
  unsigned area1, area2, areaint = 0, areaunion;
  Mat diff;
  area1 = cv::countNonZero(mat1);
  area2 = cv::countNonZero(mat2);
 
  
  cv::compare(mat1,mat2,diff,cv::CMP_NE);
  // At complete overlap 0 pixels are unequal, with no overlap area1 + area2 are unequal
  // with each equal pixel the count of unequals decreases by two
  areaint = (area1 + area2 - cv::countNonZero(diff)) / 2; 
  areaunion = area1 + area2 - areaint;
  
  if (overlap){
    return (double) areaint/ (double) area2;
  }
  
  return (double) areaint/ (double) areaunion;
}

vector<Mat> getSalientBlobs(Mat& salmap, vector<vector<Point>>& msrs){
  double lastMax,meanSaliency;
  unsigned i = 0,j = 0;
  cv::Scalar scalar;
  vector<Point> msr;
   
  vector<Mat> salientBlobs;
  vector<float> means;
  vector<float> means2;
  
  Mat iorMap = salmap.clone();
  while(true){
    Mat blob;
    msr = seededRegionGrowing(salmap,iorMap,blob,lastMax,meanSaliency,0.65);
    if (meanSaliency < 130) break;
    ior(iorMap,0.9);
    salientBlobs.push_back(blob);
    means2.push_back(meanSaliency);
    meanSaliency = meanSaliency * std::sqrt(msrs.size());  
    means.push_back(meanSaliency);
    msrs.push_back(msr);    
  }/*
  iorMap = salmap.clone();
  while(true){
    Mat blob;
    msr = seededRegionGrowing(salmap,iorMap,blob,lastMax,meanSaliency,0.7);
    if (meanSaliency < 130) break;
    ior(iorMap,0.9);
    salientBlobs.push_back(blob);
    means2.push_back(meanSaliency);
    meanSaliency = meanSaliency * std::sqrt(msrs.size());  
    means.push_back(meanSaliency);
    msrs.push_back(msr);
  }*/
  cout << "Number of salient blobs: " << salientBlobs.size() << endl;
  if (salientBlobs.size() > 0) {
    for (i = 0; i < salientBlobs.size() - 1; ++i){
      for (j = i+1; j < salientBlobs.size(); ++j){
        if (areaOverUnion(salientBlobs[i],salientBlobs[j]) > 0.5){ // duplicate is removed
          if (means[i] < means[j]) {
            swap(salientBlobs[i],salientBlobs[j]);
            swap(means[i],means[j]);
            swap(means2[i],means2[j]);
            swap(msrs[i],msrs[j]);
          }
          salientBlobs.erase(salientBlobs.begin()+j);
          means.erase(means.begin()+j);
          means2.erase(means2.begin()+j);
          msrs.erase(msrs.begin()+j);
          if (i > 0) i = i - 1;
          j = i;
        }
      }
    }
    
    for (unsigned j = 0; j < salientBlobs.size(); ++j){    
      std::cout << means2[j] << std::endl;
    }
  }
  
  cout << "Number of different salient blobs: " << salientBlobs.size() << endl;   
  /*for (i = 0; i < salientBlobs.size(); ++i){
    cout << means[i] << endl;
  } */ 
  return salientBlobs;
}

cv::Mat removeOuterRingOfPixels(cv::Mat img){
  cv::Mat output = img.clone();
  for(int row = 0; row < img.rows; row++) {
    for(int col = 0; col < img.cols; col++) {
      if(img.at<uchar>(row + 1,col) == 0 ||
         img.at<uchar>(row - 1,col) == 0 ||
         img.at<uchar>(row,col + 1) == 0 ||
         img.at<uchar>(row,col - 1) == 0)
            output.at<uchar>(row,col) = 0;
    }
  }
  return output;
}

std::vector<cv::Mat> getObjectCandidates(cv::Mat img, cv::Mat& salmap, Mat& segmentedImage, vector<Mat>& blobs ) {
  long long start = getTickCount();
  double thresh;
  unsigned i,j;
  int num_ccs;
  bool nonzero;
  std::vector<cv::Mat> segmentMats, colorCandidates;
  std::vector<std::vector<cv::Point>> segments, msrs;
  
  VOCUS2_Cfg cfg;
	VOCUS2 vocus2(cfg);

  vocus2.process(img);			
  salmap = vocus2.get_salmap();

  salmap = salmap*255.f;
  salmap.convertTo(salmap,CV_8U);
  blobs = getSalientBlobs(salmap, msrs);
  cout << "Saliency calculated" << endl;
  segmentedImage = runEgbisOnMat(img, 1, 200, 75, &num_ccs, segments);
  std::cout << "Number of segments: " << segments.size() << " == " << num_ccs << endl;
  
  for(unsigned i = 0; i < segments.size(); ++i){
    //segmentMats.push_back(Mat(salmap.size(), CV_8U, cv::Scalar(255)));
    segmentMats.push_back(Mat::zeros(salmap.size(), CV_8U));
    for(unsigned j = 0; j < segments[i].size(); ++j){
      segmentMats[i].ptr<uchar>(segments[i][j].y)[segments[i][j].x] = 1;
    }
  }
  
  for (i = 0; i < msrs.size(); ++i){
    cv::Mat colorCandidate = Mat::zeros(salmap.size(), CV_8U);
    nonzero = false;
    thresh = 0.3;
    
    while(!nonzero){
      if (thresh < 0.3) {
        //std::cout << "Reduced threshold to " << thresh << std::endl; 
        /*
        cv::imshow("blob", blobs[i]*255);        
        rectangle(segmentedImage, boundingRect(msrs[i]), 10);
        cv::imshow("segment", segmentedImage);
        rectangle(img, boundingRect(msrs[i]), 10);
        cv::imshow("img", img);
        rectangle(salmap, boundingRect(msrs[i]), 10);
        cv::imshow("salmap", salmap);
        waitKey(0);
        */
        
      }
      for (j = 0; j < segments.size(); ++j){
        // Segment is part of the candidate
        if (areaOverUnion(blobs[i],segmentMats[j],true) > thresh){
          cv::bitwise_or(colorCandidate, segmentMats[j], colorCandidate);
          nonzero = true;         
        }
      }
      thresh -= 0.025; 
    }    
    if (thresh >= 0.25){
      //colorCandidates.push_back(removeOuterRingOfPixels(colorCandidate));
      colorCandidates.push_back(colorCandidate);
      //cv::imshow("before", colorCandidate*255.f);
      //cv::imshow("after", removeOuterRingOfPixels(colorCandidate)*255.f);
      //cv::imshow("after2", removeOuterRingOfPixels(removeOuterRingOfPixels(colorCandidate))*255.f);
      //  waitKey(0);
    }    
  }   
  
  long long stop = getTickCount();
  double elapsed_time = (stop-start)/getTickFrequency();
  cout << elapsed_time << "sec" << endl;
		
	return colorCandidates;
}

bool generate_candidates(object_candidates::Objectcandidates::Request  &req,
         object_candidates::Objectcandidates::Response &res)
{      
    cv::Mat image, segmentedImage, salmap;
    std::vector<std::vector<cv::Point>> segments;
    std::vector<cv::Mat> blobs, colorCandidates;
    
    object_candidates::ArrayImages rosimgs;
    //sensor_msgs::CvBridge bridge_;
      
    try
    {      
      //image = cv_bridge::toCvShare(*req.rgb_image, "bgr8")->image;
      image = cv_bridge::toCvCopy(req.rgb_image, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", req.rgb_image.encoding.c_str());
    }    
    
    colorCandidates = getObjectCandidates(image, salmap, segmentedImage, blobs);
    
    // header of rosimgs ignored for now
    rosimgs.data.clear();
    
    for (unsigned j = 0; j < colorCandidates.size(); ++j){
      try {
             rosimgs.data.push_back(*(cv_bridge::CvImage(std_msgs::Header(), "mono8", colorCandidates[j]*255).toImageMsg()));
      }
      catch (cv_bridge::Exception& e) {
              ROS_ERROR("error");
      }
    }
    
    res.candidates = rosimgs;
    ROS_INFO("Object candidates computed");
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_candidates_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("get_object_candidates", generate_candidates);
  ROS_INFO("Ready to produce object candidates");
  ros::spin();

  return 0;
}


