#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <stdio.h>

using namespace std;
using namespace cv;

namespace vision {
	void captureImage(Mat* frame) {
		VideoCapture capture;
	    capture.open( 0 );
	    if (!capture.isOpened()) { printf("--(!)Error opening video capture\n"); return; }
		capture.read(*frame);
    	if(frame->empty()) {
            printf(" --(!) No captured frame -- Break!");
            return;
        }
    }
    void captureImages(Mat* frame) {
    	VideoCapture capture;
	    capture.open( 0 );
	    if (!capture.isOpened()) { printf("--(!)Error opening video capture\n"); return; }
	    while (capture.read(*frame)) {
	    	if(frame->empty()) {
	            printf(" --(!) No captured frame -- Break!");
	            return;
	        }
	    }
    }
    void displayImage(Mat* frame) {
    	while (1) {
    		if(frame->empty()) {
	            printf(" --(!) No captured frame -- Break!");
	            continue;
	        }
			imshow("Image from camera", *frame);
			if( waitKey(10) == 27 ) { return; }
		}
    }
}