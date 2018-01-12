#import "field.cpp"
#import "vision.cpp"

int main(){
	Field f;
	while(1){
		vision::captureImage();
		f.tick();
	}
}