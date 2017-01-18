
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sys/stat.h>
#include <math.h>
#include <map>
#include <string>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#include "opencv-wrapper-egbis/egbis.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "vocus2-version1.1/src/VOCUS2.h"

using namespace std;
using namespace cv;

struct stat sb;


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
  
  Mat iorMap = salmap.clone();
  while(true){
    Mat blob;
    msr = seededRegionGrowing(salmap,iorMap,blob,lastMax,meanSaliency,0.6);
    if (meanSaliency < 120) break;
    ior(iorMap,0.9);
    salientBlobs.push_back(blob);
    meanSaliency = meanSaliency;
    means.push_back(meanSaliency);
    msrs.push_back(msr);    
  }
  iorMap = salmap.clone();
  while(true){
    Mat blob;
    msr = seededRegionGrowing(salmap,iorMap,blob,lastMax,meanSaliency,0.7);
    if (meanSaliency < 120) break;
    ior(iorMap,0.9);
    salientBlobs.push_back(blob);
    meanSaliency = meanSaliency;  
    means.push_back(meanSaliency);
    msrs.push_back(msr);
  }
  cout << "Number of salient blobs: " << salientBlobs.size() << endl;
  for (i = 0; i < salientBlobs.size() - 1; ++i){
    for (j = i+1; j < salientBlobs.size(); ++j){
      if (areaOverUnion(salientBlobs[i],salientBlobs[j]) > 0.5){ // duplicate is removed
        if (means[i] < means[j]) {
          swap(salientBlobs[i],salientBlobs[j]);
          swap(means[i],means[j]);
          swap(msrs[i],msrs[j]);
        }
        salientBlobs.erase(salientBlobs.begin()+j);
        means.erase(means.begin()+j);
        msrs.erase(msrs.begin()+j);
        if (i > 0) i = i - 1;
        j = i;
      }
    }
  }
  cout << "Number of different salient blobs: " << salientBlobs.size() << endl;   
  /*for (i = 0; i < salientBlobs.size(); ++i){
    cout << means[i] << endl;
  } */ 
  return salientBlobs;
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
  segmentedImage = runEgbisOnMat(img, 0, 200, 100, &num_ccs, segments);
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
        /**/
        
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
      colorCandidates.push_back(colorCandidate);
    }    
  }   
  
  long long stop = getTickCount();
  double elapsed_time = (stop-start)/getTickFrequency();
  cout << elapsed_time << "sec" << endl;
		
	return colorCandidates;
}

cv::Mat loadFile(std::string file, int flag = CV_LOAD_IMAGE_COLOR){
  cout << "Opening " << file << std::endl;
	cv::Mat img = imread(file, flag);
  
  if(! img.data){
    std::cout <<  "Could not open or find the image: "<< file << std::endl;
    exit(-1);
  }
  
  return img;
}

void benchmark(){
  boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::mean, boost::accumulators::tag::variance>> acc;
  cv::Mat img, segmentedImage, salmap, predictedObjects, gt;
  std::vector<cv::Mat> blobs, colorCandidates;
  
  std::string imgsPath = "/home/fobse/cv_project/Code/other/salObj/datasets/imgs/imgsal/";
  std::string gtPath =  "/home/fobse/cv_project/Code/other/salObj/datasets/masks/imgsal/";
  
  for(int i = 1; i <= 235; ++i){
    colorCandidates = getObjectCandidates(loadFile(imgsPath+std::to_string(i)+".jpg"), salmap, segmentedImage, blobs);
    gt = loadFile(gtPath+std::to_string(i)+".png", CV_LOAD_IMAGE_GRAYSCALE);
    predictedObjects = Mat::zeros(salmap.size(), CV_8U);
    for (unsigned j = 0; j < colorCandidates.size(); ++j){
      cv::bitwise_or(predictedObjects, colorCandidates[j], predictedObjects);
    }
    predictedObjects = predictedObjects * 255;
    acc(areaOverUnion(predictedObjects, gt));
    cout << "IoU: " << (areaOverUnion(predictedObjects, gt)) << endl;
  }
  
  cout << "mean: " << boost::accumulators::mean(acc) << endl;
  cout << "std deviation: " << sqrt(boost::accumulators::variance(acc)) << endl;
  
  cv::imshow("ground truth", gt);
  cv::imshow("pred", predictedObjects);
  waitKey(0);
}

int main(int argc, char* argv[]) {
  //std::string file = "../test-image-MSRA-3_97_97769.jpg";
  std::string file = "../235.jpg";
  
  cv::Mat img, segmentedImage, salmap, predictedObjects;
  std::vector<cv::Mat> blobs, colorCandidates;
  
  //benchmark();
  
  img = loadFile(file);
  
  colorCandidates = getObjectCandidates(img, salmap, segmentedImage, blobs);

  predictedObjects = Mat::zeros(salmap.size(), CV_8U);
  for (unsigned j = 0; j < colorCandidates.size(); ++j){
    cv::bitwise_or(predictedObjects, colorCandidates[j], predictedObjects);
  }

  cv::imshow("Segments", segmentedImage);  
  cv::imshow("Original image", img);
  cv::imshow("Saliency", salmap);
  //cv::imshow("Colour candidates0", colorCandidates[0]*255);
  //cv::imshow("blob", blobs[0]*255.f);
  cv::imshow("predictedObjects", predictedObjects*255);
  
  waitKey(0);
  /**/
  
  
  /*
  cv::VideoCapture cap;
  if(!cap.open(0))
    return 0;
  for(;;)
  {
    for(int skip = 0; skip < 200; ++skip){
      if(skip == 0){        
        cap >> frame;
      }
    }
    if (frame.empty()) break;
    rects = getObjectCandidates(vocus2, frame);
    
    for (unsigned i = 0; i < rects.size() - 1; ++i){
      rectangle(frame, rects[i], 10);
    }
    cv::imshow("Colour candidates", frame);
    if (cv::waitKey(1) == 27) break;
  }*/
  
  return EXIT_SUCCESS;
}
