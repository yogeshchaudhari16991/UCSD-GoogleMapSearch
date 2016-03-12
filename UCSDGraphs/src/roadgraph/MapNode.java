/**
 * 
 */
package roadgraph;

import java.util.List;

import geography.GeographicPoint;

/**
 * @author Yogesh
 *
 */
public class MapNode implements Comparable {
	private GeographicPoint location;
	private List<MapEdge> neighbors;
	private double length;
	private double distToGoal;
	
	public MapNode(){
		//construct new empty MapNode
	}
	
	public MapNode(GeographicPoint location, List<MapEdge> neighbors){
		this.location = location;
		this.neighbors = neighbors;
	}
	
	public MapNode(GeographicPoint location, double length){
		this.location = location;
		this.length = length;
		this.distToGoal = 0;
	}
	
	public MapNode(GeographicPoint location, double length, double distToGoal){
		this.location = location;
		this.length = length;
		this.distToGoal = distToGoal;
	}
	
	public GeographicPoint getLocation(){
		return location;
	}
	
	public List<MapEdge> getNeighbor(){
		return neighbors;
	}
	
	public double getLength(){
		return length;
	}
	
	public double getDistToGoal(){
		return distToGoal;
	}
	
	public void setLength(double length){
		this.length = length;
	}
	
	public void setDistToGoal(double distToGoal){
		this.distToGoal= distToGoal;
	}
	
	public boolean addNeighbor(MapEdge edge){
		if(! neighbors.contains(edge))
			if(neighbors.add(edge))
				return true;
		return false;
	}
	
	public int getNumOfNeighbors(){
		return neighbors.size();
	}
	
	@Override
	public int compareTo(Object o1){
		MapNode obj1 = (MapNode)o1;
		MapNode obj2 = this;
		double length1 = obj2.getLength() + obj2.getDistToGoal();
		double length2 = obj1.getLength() + obj1.getDistToGoal();
		if(length1 >= length2){
			return 1;
		}else {
				return -1;
		}
	}
}
