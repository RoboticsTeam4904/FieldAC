#import "rl/mcl.cpp"
#import "objects.cpp"

void field::tick(){
	currentBestPose= MCL:Tick();

	robottrack::tick(currentBestPose);
	cubetrack::tick(currentBestPose);
}