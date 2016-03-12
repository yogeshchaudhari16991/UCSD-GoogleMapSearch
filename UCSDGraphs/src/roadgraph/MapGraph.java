/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Stack;
import java.util.Vector;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and Yogesh
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */



public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	
	public static final double INFINITY = 32656;
	
	private HashMap<GeographicPoint, MapNode> vertices;

	/** 
	 * Create a new empty MapGraph 
	 */
	
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		vertices = new HashMap<GeographicPoint, MapNode>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return vertices.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		return vertices.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		Set<GeographicPoint> verticesSet = getVertices();
		int numberOfVertices = getNumVertices();
		int numEdges = 0;
		for(GeographicPoint location : verticesSet){
			MapNode vertex = vertices.get(location);
			numEdges += vertex.getNumOfNeighbors();
		}
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		if(location != null){
			if(! vertices.containsKey(location)){
				MapNode vertex = new MapNode(location, new ArrayList<MapEdge>());
				vertices.put(location, vertex);
				return true;
			}
		}
		return false;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
			if(from != null && to !=  null && roadName != null && roadType != null
					&& vertices.containsKey(from) && vertices.containsKey(to) && length > 0){
				MapEdge edge = new MapEdge(from, to, roadName, length, roadType);
				MapNode vertex = vertices.get(from);
				vertex.addNeighbor(edge);
				vertices.put(from, vertex);
			}
			else{
				throw new IllegalArgumentException();
			}
		//TODO: Implement this method in WEEK 2
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	
	public List<GeographicPoint> bfs(GeographicPoint start, 
		     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched){
		
		Vector<GeographicPoint> visited;
		HashMap<GeographicPoint, GeographicPoint> parent;
		Queue<GeographicPoint> queue;
		visited = new Vector<GeographicPoint>();
		parent = new HashMap<GeographicPoint, GeographicPoint>();
		queue = new LinkedList<GeographicPoint>();

		visited.add(start);
		queue.add(start);
		while(! queue.isEmpty()){
			GeographicPoint current = queue.remove();
			nodeSearched.accept(current);
			visited.add(current);
			if(current.equals(goal)){
				return getPath(parent, visited);
			}
			for(MapEdge e : vertices.get(current).getNeighbor()){
				GeographicPoint next = e.getEnd();
				if((! visited.contains(next)) && (! queue.contains(next))){
					parent.put(next, current);
					queue.add(next);
				}
			}
		}
		//null means there is no path between start and goal
		return null;
	}
	
	public List<GeographicPoint> getPath(HashMap<GeographicPoint, GeographicPoint> parent, Vector<GeographicPoint> visited){
		List<GeographicPoint> pathToVertex = new ArrayList<GeographicPoint>();
		GeographicPoint goal = visited.get((visited.size()-1));
		pathToVertex.add(goal);
		//System.out.println(path.toString());
		GeographicPoint current  = goal;
		while(parent.containsKey(current)){
			pathToVertex.add(parent.get(current));
			current = parent.get(current);
		}
//		for(int i=(visited.size()-1); i >= 0; i--){
//			if(parent.get(visited.get(i)) != null && !pathToVertex.contains(parent.get(visited.get(i)))){
//				pathToVertex.add(parent.get(visited.get(i)));
//			}
//		}
//		System.out.println("Parent List:");
//		parent.forEach((K, V)->System.out.println("Key: "+ K+", Value: " + V));
//		System.out.println();
		//reversing path, so it starts from start vertex and ends at goal vertex
		List<GeographicPoint> pathToGoal = new ArrayList<GeographicPoint>();
		for(int i = pathToVertex.size()-1; i>=0; i--) {
			pathToGoal.add(pathToVertex.get(i));
		}
		return pathToGoal;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		PriorityQueue<MapNode> queue = new PriorityQueue<MapNode>();
		HashMap<GeographicPoint, GeographicPoint> parent = new HashMap<GeographicPoint, GeographicPoint>();
		Vector<GeographicPoint> visited = new Vector<GeographicPoint>();
		
		//initialize all vertices with INFINITY distance
		for(GeographicPoint loc : vertices.keySet()){
			MapNode temp = new MapNode(loc, INFINITY);
			queue.add(temp);
		}
		
		//enqueue Start Vertex
		MapNode startNode = new MapNode(start, 0);
		queue.add(startNode);
		
		//run while queue is not empty
		while(! queue.isEmpty()){
			MapNode current = queue.remove();
			nodeSearched.accept(current.getLocation());
			//check if current node has been visited or not
			if(! visited.contains(current)){
				//mark current node visited
				visited.add(current.getLocation());
				//check if current node is goal node or not
				if(goal.equals(current.getLocation())){
					return getPath(parent, visited);
				}
				//iterate over all edges outgoing from current node
				for(MapEdge edge : vertices.get(current.getLocation()).getNeighbor()){
					GeographicPoint nextGeoLoc = edge.getEnd();
					MapNode next = vertices.get(nextGeoLoc);
					//check if next neighbor of current node is visited or not
					if(! visited.contains(nextGeoLoc)){
						//if not add it to queue with updated distance
						long distFromStart = (long)(current.getLength() + edge.getLength());
						if(queue.contains(next)){
							if(next.getLength() > distFromStart){
								next.setLength(distFromStart);
								parent.put(next.getLocation(), current.getLocation());
								queue.add(next);
							}
						}
						else {
								next.setLength(distFromStart);
								queue.add(next);
								parent.put(next.getLocation(), current.getLocation());
							}
						}
					}
				}
			}
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		PriorityQueue<MapNode> queue = new PriorityQueue<MapNode>();
		HashMap<GeographicPoint, GeographicPoint> parent = new HashMap<GeographicPoint, GeographicPoint>();
		Vector<GeographicPoint> visited = new Vector<GeographicPoint>();
		//initialize all vertices with INFINITY distance
		for(GeographicPoint loc : vertices.keySet()){
			MapNode temp = new MapNode(loc, INFINITY);
			queue.add(temp);
		}
		
		//enqueue Start Vertex
		MapNode startNode = new MapNode(start, 0, getDistToGoal(start, goal));
		queue.add(startNode);
		
		//run while queue is not empty
		while(! queue.isEmpty()){
			MapNode current = queue.remove();
			nodeSearched.accept(current.getLocation());
			//check if current node has been visited or not
			if(! visited.contains(current)){
				//mark current node visited
				visited.add(current.getLocation());
				//check if current node is goal node or not
				if(goal.equals(current.getLocation())){
					return getPath(parent, visited);
				}
				//iterate over all edges outgoing from current node
				for(MapEdge edge : vertices.get(current.getLocation()).getNeighbor()){
					GeographicPoint nextGeoLoc = edge.getEnd();
					MapNode next = vertices.get(nextGeoLoc);
					//check if next neighbor of current node is visited or not
					if(! visited.contains(nextGeoLoc)){
						//if not add it to queue with updated distance
						long distFromStart = (long)(current.getLength() + edge.getLength());
						if(queue.contains(next)){
							if(next.getLength() > distFromStart){
								next.setLength(distFromStart);
								parent.put(next.getLocation(), current.getLocation());
								queue.add(next);
							}
						}
						else {
								next.setLength(distFromStart);
								next.setDistToGoal(getDistToGoal(next.getLocation(),goal));
								queue.add(next);
								parent.put(next.getLocation(), current.getLocation());
							}
						}
					}
				}
			}
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	
	
	private double getDistToGoal(GeographicPoint current, GeographicPoint goal) {
		return current.distance(goal);
	}

	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		GeographicPoint start = new GeographicPoint(1.0,1.0);
		GeographicPoint end = new GeographicPoint(8.0,-1.0);
		List<GeographicPoint> path = theMap.aStarSearch(start, end);
		System.out.println("Path to Goal is" + path.toString());
		// You can use this method for testing.  
		
		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
