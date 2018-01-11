package fieldModel;

import java.awt.Point;
import java.util.ArrayList;

public class Field {

	ArrayList<Cube> cubes;
	ArrayList<Robot> robots;
	
	public Field() {
		cubes = new ArrayList<Cube>();
		robots = new ArrayList<Robot>();
	}
	
	public void addCube(int x, int y) {
		cubes.add(new Cube());
	}
	
	public void addRobot(int x, int y) {
		robots.add(new Robot());
	}
	
	/**
	 * idk if this is where we want to do the actual calculations, but we need to be able
	 * to convert all the data taken into this method into absolute field coordinates to
	 * use in the model.
	 * 
	 * @param robotX where our robot is at the time of viewing
	 * @param robotY where our robot is at the time of viewing
	 * @param viewLocation the pixel location on the camera input of the object
	 * @param textSize how big the bumper text is, in pixels
	 */
	public void addCube(int robotX, int robotY, int viewLocation, int textSize) {
		addCube(robotX, robotY);
	}
	
	public Cube getClosestCube(int probabilityThreshold, int targetX, int targetY) {
		return cubes.get(0);
		
	}
}
