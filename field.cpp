#import "rl/mcl.cpp"
#import "objects.cpp"
#import "ot/cubetrack.cpp"

class Field {
public:
	void tick() {
		ot::tick();
		Mat* lol = vision::getFrame();
		printf("Got frame");
		printf("%d\n", std::chrono::system_clock::now());
	// currentBestPose= MCL:Tick();

	// robottrack::tick(currentBestPose);
	// cubetrack::tick(currentBestPose);
}
};