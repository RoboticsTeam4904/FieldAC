#import "field.cpp"
#import "vision.cpp"
main(){
	while(1){
		vision::actuallyGetNewImage();
		field::tick();
	}
}