//src from opencv.org

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <list>

using namespace cv;
using namespace std;

/// Global variables

Mat src, src_gray;
Mat dst, detected_edges;

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
char* window_name = "Edge Map";
int start;

#define GREATER_THAN_Y 0
#define SMALLER_THAN_Y 1

#define GREATER_THAN_X 2
#define SMALLER_THAN_X 3

//src: http://www.cplusplus.com/forum/general/43203/
using namespace std;
#include "cstdlib"
#include <sys/timeb.h>
int getMilliCount(){
	timeb tb;
	ftime(&tb);
	int nCount = tb.millitm + (tb.time & 0xfffff) * 1000;
	return nCount;
}

int getMilliSpan(int nTimeStart){
	int nSpan = getMilliCount() - nTimeStart;
	if(nSpan < 0)
		nSpan += 0x100000 * 1000;
	return nSpan;
}

struct MyStruct
{
    int key;
    std::string stringValue;

    MyStruct(int k, const std::string& s) : key(k), stringValue(s) {}

    bool operator < (const MyStruct& str) const
    {
        return (key < str.key);
    }
};

/**
 * @function CannyThreshold
 * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 */
void CannyThreshold(int, void*)
{
  /// Reduce noise with a kernel 3x3
  blur( src_gray, detected_edges, Size(3,3) );

  /// Canny detector
  Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

  /// Using Canny's output as a mask, we display our result
  dst = Scalar::all(0);

  src.copyTo( dst, detected_edges);
  
  //

  list<Point> points;
  //points.reserve(100);
	Mat dst_gray;
	cvtColor( dst, dst_gray, CV_BGR2GRAY );

  
  //cout << "cols: " << src_gray.cols << ",rows: " << src_gray.rows << endl;
  
  //cout << "x,y" << endl;
  unsigned int threshold = 10000;
  for(int x = 0 ; x < dst_gray.cols ; x++){
	  for(int y = 0; y < dst_gray.rows; y++){
		  Scalar intensity = dst_gray.at<uchar>(y, x);
		  //cout << x << "," << y << "," << intensity.val[0] << endl;

		  if(intensity.val[0] > 100 && y < 460){
			Point pnt(x,y);
			points.push_back(pnt);
			//FOR PRINTING!!
			//cout << x << "," << y << "," << intensity.val[0] << endl;
		  }
	  }
  }

  //cout << "list size: " << points.size() << endl;

  //Okey, find everything with the same x:
  Point lastpoint(0,points.size());
  list<Point> graph;
  list<Point>::iterator iter = points.begin();
  int count = 0;
  
	while(iter != points.end())
	{
		// retrieve the graph the iterator points at
		Point& p = *iter;
		if(lastpoint.x == p.x){
			//more of them:
			//get the average
			lastpoint.y = (lastpoint.y + p.y)/2;
		}else{
			//new set - print out lastpoint:
			int y =  dst_gray.rows - lastpoint.y;
			graph.push_back(Point(p.x,y));
			//cout << p.x << "," << y << endl;
			lastpoint.x = p.x;
			lastpoint.y = p.y;
		}     
		//cout << p.x << "," << p.y << endl;
        // now move the iterator over to the next element
        // in the list
		count++;
        iter++;
    }

	list<Point> graphDots ;
	double meanValue = 0;
	//find the mean value in y-vector
	//cout << "graph size: " << graph.size() << endl;

	//cout << "x,y" << endl;
	list<Point>::iterator iter2 = graph.begin();

	//get the offset
	list<Point>::iterator itz = points.begin();
	Point &pitz = *itz;
	count = pitz.x;
	while(iter2 != graph.end()){
		Point& p = *iter2;
		Point pnt = Point(p.x,p.y);
		graphDots.push_back(pnt);

		//cout << pnt.x << "," << pnt.y << endl;

		meanValue = meanValue+p.y;
		iter2++;
		count++;
	}

	meanValue = (meanValue*1.0)/(graph.size());
	//cout << "mean value: " << meanValue << endl;

	//SORT MANUALLY:
	//bubbleSort(graphDots,0);

	
	bool runMe = true;
	while(runMe){
		count = 0;
		list<Point>::iterator iter3 = graphDots.begin();

		while(iter3 != graphDots.end()){
			list<Point>::iterator p_tmp;

			p_tmp = iter3;
			p_tmp++;
			Point& p = *iter3;
			Point& p2 = (*p_tmp);
			Point p3;//kanske inte fungerar

			if(p2.y > p.y){
				//change place of points:
				p3.x = p.x;
				p3.y = p.y;

				p.x = p2.x;
				p.y = p2.y;

				p2.x = p3.x;
				p2.y = p3.y;

				//cout << "yolo: " << p.y << "," << p2.y << endl;
				break;
		}

			if(count == graphDots.size()-1){
				//cout << "hoooraaay" << endl;
				//MUST EXIT LOOP HERE:
				runMe = false;
				break;
			}
			count++;
			iter3++;
		}
	}




	//go through the twenty first ones:
	//create a new list with only these values
	list<Point> selectedList;
	list<Point>::iterator iter4 = graphDots.begin();
	count = 0;
	while(iter4 != graphDots.end()){
		if(count > 30){
			break;
		}
		list<Point>::iterator p4;
		Point& p = *iter4;

		selectedList.push_back(Point(p));
		//cout << "Vector: [" << (p.x) << "," << (int)(p.y) << "]" << endl;


		iter4++;
		count++;
	}
	

	//SORT list
	
	runMe = true;
	while(runMe){
		count = 0;
		list<Point>::iterator iter5 = selectedList.begin();

		while(iter5 != selectedList.end()){
			list<Point>::iterator p_tmp;

			p_tmp = iter5;
			p_tmp++;
			Point& p = *iter5;
			Point& p2 = (*p_tmp);
			Point p3;//kanske inte fungerar

			if(p2.x < p.x){
				//change place of points:
				p3.x = p.x;
				p3.y = p.y;

				p.x = p2.x;
				p.y = p2.y;

				p2.x = p3.x;
				p2.y = p3.y;

				//cout << "yolo: " << p.y << "," << p2.y << endl;
				break;
			}

			if(count == selectedList.size()-1){
				//cout << "hoooraaay" << endl;
				//MUST EXIT LOOP HERE:
				runMe = false;
				break;
			}
			count++;
			iter5++;
		}
	}
	
	//go through the twenty first ones:
	//create a new list with only these values
	list<Point> vectors ;

	list<Point>::iterator iter6 = selectedList.begin();
	int vectorCount = 0;
	count = 0;
	Point highestPoint = Point(0,0);
	Point &last_p = *iter6;
	while(iter6 != selectedList.end()){
		//this needs to be changed further down aswell!!!
		//WARNING
		if(count > 30){
			break;
		}
		list<Point>::iterator p4;
		Point& p = *iter6;

		//selectedList.push_back(p);
		if(p.x != 0){
			if(last_p.x +1 == p.x){
				vectorCount++;
				if(p.y > highestPoint.y){
					//cout << "hp [" << highestPoint.x << "," << highestPoint.y << " p [" << p.x << "," << p.y << "]" << endl;

					highestPoint.x = p.x;
					highestPoint.y = p.y;
				}
				//RIGHT HERE
				if(count+1 > 30){
					vectors.push_back(Point(highestPoint));
					highestPoint.x = 0;
					highestPoint.y = 0;
					vectorCount = 0;

					//cout << "h1p:" << highestPoint.x << "," << highestPoint.y << endl;
				
				}
			}else{
				if(vectorCount > 3){
					vectors.push_back(Point(highestPoint));
					//cout << "hp:" << highestPoint.x << "," << highestPoint.y << endl;
				}
				vectorCount = 0;
				highestPoint.x = 0;
				highestPoint.y = 0;
			}
			//cout << "Vector<-: [" << (p.x) << "," << (int)(p.y) << "]" << endl;
		}
		last_p = *iter6;
		iter6++;
		count++;
	}
	
	list<Point>::iterator iter7 = vectors.begin();
	count = 0;
	//cout << "possible vectors are:" << endl;
	while(iter7 != vectors.end()){
		Point& p = *iter7;
		//cout << "[" << p.x << "," << p.y << "]" << endl;
		arrowedLine(dst,Point(p.x,dst.rows-meanValue),Point(p.x,dst.rows - p.y ),Scalar(255,0,0),2,8);
		count++;
		iter7++;
	} 
	//create a baseline:
	line(dst,Point(0,dst.rows-meanValue),Point(dst.cols,dst.rows - meanValue),Scalar(0,255,0),1,8);

	int milliSecondsElapsed = getMilliSpan(start);
	printf("_MAINTHREAD,%u\n", milliSecondsElapsed);

	//draw the graph on a new matrix
	//new_matrix.push_back(

	imshow(window_name,dst);
 }

 
/** @function main */
int main( int argc, char** argv )
{

  /// Load an image
  src = imread( argv[1] );

  if( !src.data )
  { return -1; }
  	start = getMilliCount();
  /// Create a matrix of the same type and size as src (for dst)
  dst.create( src.size(), src.type() );

  /// Convert the image to grayscale
  cvtColor( src, src_gray, CV_BGR2GRAY );

  /// Create a window
  namedWindow( window_name, CV_WINDOW_AUTOSIZE );

  /// Create a Trackbar for user to enter threshold
  createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );

  /// Show the image
  CannyThreshold(0, 0);

  /// Wait until user exit program by pressing a key
  waitKey(0);

  return 0;
  }
