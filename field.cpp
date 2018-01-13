#import "rl/mcl.cpp"
#import "objects.cpp"
#import "ot/cubetrack.cpp"

class Field {
public:
	void tick(Mat frame) {
		ot::tick(frame);
	// currentBestPose= MCL:Tick();

	// robottrack::tick(currentBestPose);
	// cubetrack::tick(currentBestPose);
}
};