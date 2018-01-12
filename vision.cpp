#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <stdio.h>

using namespace std;
using namespace cv;

namespace vision {
	void captureImage(){
		VideoCapture capture;
	    Mat frame;
	    capture.open( 0 );
	    if ( ! capture.isOpened() ) { printf("--(!)Error opening video capture\n"); return; }
	    while ( capture.read(frame) )
	    {
	    	 if( frame.empty() )
        {
            printf(" --(!) No captured frame -- Break!");
            break;
        }

	        imshow("lol", frame);
	        if( waitKey(10) == 27 ) { break; } // escape
	    }
	}
}