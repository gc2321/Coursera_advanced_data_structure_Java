/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;



/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	private int numVertices = 0;
	private int numEdges = 0;
	
	Map<GeographicPoint, MapNode> vertices = new HashMap<>();
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2 - DONE
		return this.numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2 - DONE
		
		Set<GeographicPoint> result = new HashSet<>();
		for (Map.Entry<GeographicPoint, MapNode> i : this.vertices.entrySet()){
			result.add(i.getKey());
		}		
		return result;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2 - DONE
		return this.numEdges;
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
		// TODO: Implement this method in WEEK 2 - DONE
		if (this.vertices.containsKey(location)==false){
			this.vertices.put(location, new MapNode(location));
			this.numVertices +=1;
			return true;
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

		//TODO: Implement this method in WEEK 2 - DONE
		if (vertices.containsKey(from)==false){
			this.addVertex(from);
		}
		if (vertices.containsKey(to)==false){
			this.addVertex(to);
		}
			
		if (this.vertices.get(from).addEdge(new Edge(from, to, roadName, roadType, length))){
			this.numEdges +=1;
			this.vertices.get(from).addNeighbor(to);
		}
		
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
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 2
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		if (!this.vertices.containsKey(start) ||!this.vertices.containsKey(goal)){
			return null;
		}
		
		
		List<GeographicPoint> result = new ArrayList<>();
		result.add(start);		
		
		if (start.equals(goal)){			
			return result;
		}
		
		Queue<GeographicPoint> queue = new LinkedList<>();
		List<GeographicPoint> visited = new ArrayList<>();
		Map<GeographicPoint, GeographicPoint>parent = new HashMap<>();
				
		queue.add(start);
		
		while(!queue.isEmpty()){
			GeographicPoint current = queue.remove();
			if (current.equals(goal)){
				break;
			}else{
				List <GeographicPoint> currentNeighbors = this.vertices.get(current).getNeighbors();
				for (GeographicPoint i : currentNeighbors){
					if (visited.contains(i)==false){
						visited.add(i);
						queue.add(i);
						
						if (parent.containsKey(i)==false){
							parent.put(i, current);
						}
					}
				}			
			}			
		}
		
		// trace back from goal to start
		List<GeographicPoint> traceback = new ArrayList<>();
				
		GeographicPoint curr = goal;
		while (parent.get(curr)!=start){
		//while (parent.get(curr)!=null && parent.get(curr)!=start){
			if (parent.get(curr)==null){
				return null;
			}
			
			
			traceback.add(parent.get(curr));
			curr = parent.get(curr);
		}
		
		Collections.reverse(traceback);
		result.addAll(traceback);
		result.add(goal);
		
		return result;
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
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		Map<GeographicPoint, Double> childOfGoal = new HashMap<>();
				
		if (!this.vertices.containsKey(start) ||!this.vertices.containsKey(goal)){
			return null;
		}
				
		List<GeographicPoint> result = new ArrayList<>();
		result.add(start);		
		
		if (start.equals(goal)){			
			return result;
		}
			
		//DNodeComparator comparator = new DNodeComparator();
		PriorityQueue<DNode> queue = new PriorityQueue<>();
				
		List<GeographicPoint> visited = new ArrayList<>();
		
		Map<MapNode, MapNode> parent = new HashMap<>();
				
		queue.add(new DNode(vertices.get(start), 0.0));
		
		while(!queue.isEmpty()){
			DNode current = queue.poll();
			
			//System.out.println("");
			System.out.println("Current: "+current.getGeographicPoint());
			
			//check if current is goal
			if (current.getGeographicPoint().distance(goal)==0 && visited.contains(current.getGeographicPoint())){
				//System.out.println("breaking loop");
				break;
			}
			
			if (!visited.contains(current.getGeographicPoint())){
				visited.add(current.getGeographicPoint());
								
				List <GeographicPoint> currentNeighbors = current.getNeighbors();
				
				for (GeographicPoint i : currentNeighbors){
					
					if (!visited.contains(i)){
						
						// add DNode of each neighbor not visited to queue
						MapNode neighbor = this.vertices.get(i);
						double distance_to_start = current.getDistance()+current.getDistanceInEdge(neighbor.getGeographicPoint());	
						queue.add(new DNode(neighbor, distance_to_start));
						
						// establish parent/child relationship in new node
						if (i.distance(goal)==0){
							childOfGoal.put(current.getGeographicPoint(), distance_to_start);
							
						}else if (parent.containsKey(neighbor)==false){
							parent.put(neighbor, current.getMapNode());
							
						}
					}
				}
				

			}			
		}
		
		// find lowest distance in childOfGoal
		double minValueInChildOfGoal = (Collections.min(childOfGoal.values())); 
		//System.out.println(minValueInChildOfGoal);
		for (Map.Entry<GeographicPoint, Double> i : childOfGoal.entrySet()) {
			//System.out.println(i.getKey()+" "+i.getValue());
			if (i.getValue()==minValueInChildOfGoal){
				parent.put(this.vertices.get(goal), this.vertices.get(i.getKey()));
			}
			
		}
			
		// trace back from goal to start
		List<GeographicPoint> traceback = new ArrayList<>();
				
		GeographicPoint curr = goal;
		while (parent.get(this.vertices.get(curr))!=this.vertices.get(start)){
		
			if (parent.get(this.vertices.get(curr))==null){
				return null;
			}
			GeographicPoint currPoint = parent.get(this.vertices.get(curr)).getGeographicPoint();
			
			traceback.add(currPoint);
			curr = currPoint;
		}
		
		Collections.reverse(traceback);
		result.addAll(traceback);
		result.add(goal);
		
		return result;
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
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		Map<GeographicPoint, Double> childOfGoal = new HashMap<>();
		
		if (!this.vertices.containsKey(start) ||!this.vertices.containsKey(goal)){
			return null;
		}
				
		List<GeographicPoint> result = new ArrayList<>();
		result.add(start);		
		
		if (start.equals(goal)){			
			return result;
		}
			
		//DNodeComparator comparator = new DNodeComparator();
		PriorityQueue<DNode> queue = new PriorityQueue<>();
				
		List<GeographicPoint> visited = new ArrayList<>();
		
		Map<MapNode, MapNode> parent = new HashMap<>();
				
		queue.add(new DNode(vertices.get(start), 0.0));
		
		while(!queue.isEmpty()){
			DNode current = queue.poll();
			
			//System.out.println("");
			System.out.println("Current: "+current.getGeographicPoint());
			
			//check if current is goal
			if (current.getGeographicPoint().distance(goal)==0 && visited.contains(current.getGeographicPoint())){
				//System.out.println("breaking loop");
				break;
			}
			
			if (!visited.contains(current.getGeographicPoint())){
				visited.add(current.getGeographicPoint());
								
				List <GeographicPoint> currentNeighbors = current.getNeighbors();
				
				for (GeographicPoint i : currentNeighbors){
					
					if (!visited.contains(i)){
						
						// add DNode of each neighbor not visited to queue
						MapNode neighbor = this.vertices.get(i);
						double distance_to_start = current.getDistance()+current.getDistanceInEdge(neighbor.getGeographicPoint());	
						distance_to_start += neighbor.getGeographicPoint().distance(goal);
						
						queue.add(new DNode(neighbor, distance_to_start));
						
						// establish parent/child relationship in new node
						if (i.distance(goal)==0){
							childOfGoal.put(current.getGeographicPoint(), distance_to_start);
							
						}else if (parent.containsKey(neighbor)==false){
							parent.put(neighbor, current.getMapNode());
							
						}
					}
				}
				

			}			
		}
		
		// find lowest distance in childOfGoal
		double minValueInChildOfGoal = (Collections.min(childOfGoal.values())); 
		//System.out.println(minValueInChildOfGoal);
		for (Map.Entry<GeographicPoint, Double> i : childOfGoal.entrySet()) {
			//System.out.println(i.getKey()+" "+i.getValue());
			if (i.getValue()==minValueInChildOfGoal){
				parent.put(this.vertices.get(goal), this.vertices.get(i.getKey()));
			}
			
		}
			
		// trace back from goal to start
		List<GeographicPoint> traceback = new ArrayList<>();
				
		GeographicPoint curr = goal;
		while (parent.get(this.vertices.get(curr))!=this.vertices.get(start)){
		
			if (parent.get(this.vertices.get(curr))==null){
				return null;
			}
			GeographicPoint currPoint = parent.get(this.vertices.get(curr)).getGeographicPoint();
			
			traceback.add(currPoint);
			curr = currPoint;
		}
		
		Collections.reverse(traceback);
		result.addAll(traceback);
		result.add(goal);
		
		return result;
	}

	

	
	public static void main(String[] args)
	{
//		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
//		System.out.println("DONE.");
		
		//System.out.println(theMap.getVertices());
		//System.out.println(theMap.numVertices);
		//System.out.println(theMap.numEdges);
		GeographicPoint start = new GeographicPoint(1.0, 1.0);
		GeographicPoint end = new GeographicPoint(8.0, -1.0);
		//System.out.println(theMap.bfs(start, end));
		System.out.println(theMap.dijkstra(start, end));
		System.out.println(theMap.aStarSearch(start, end));
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

class MapNode{
	
	private GeographicPoint node;
	
	public MapNode(GeographicPoint node){
		this.node = node;
	}
	
	List<Edge> edges = new ArrayList<>();
	
	public List<Edge> getEdges(){
		return this.edges;
	}
	public Boolean addEdge(Edge edge){
		if (this.edges.contains(edge)==false){
			this.edges.add(edge);
			return true;
		}
		return false;
	}
	
	// get distance from this GeographicPoint to 'other' GeographicPoint
	public double getDistanceInEdge(GeographicPoint other){
		return this.node.distance(other);
		
	}
	
	List<GeographicPoint>neighbors = new ArrayList<>();
	
	public Boolean addNeighbor(GeographicPoint neighbor){
		if (this.neighbors.contains(neighbor)==false){
			this.neighbors.add(neighbor);
			return true;
		}
		return false;
	}
	public List<GeographicPoint> getNeighbors(){
		return this.neighbors;
	}
	
	public GeographicPoint getGeographicPoint(){
		return this.node;
	}
	
}


class Edge{
	private GeographicPoint end1;
	private GeographicPoint end2;
	private String roadName;
	private String roadType;
	private double length;
	
	public Edge (GeographicPoint end1, GeographicPoint end2, String roadName,String roadType, double length){
		this.end1 = end1;
		this.end2 = end2;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}
	
	public GeographicPoint getEnd1(){
		return this.end1;
	}
	public GeographicPoint getEnd2(){
		return this.end2;
	}
	public String getRoadName(){
		return this.roadName;
	}
	public String getRoadType(){
		return this.roadType;
	}
	public double getLength(){
		return this.length;
	}
}

class DNode implements Comparable<DNode>{
	
	private MapNode node;
	private GeographicPoint point;
	
	// distance to start node in Dijkstra's search
	private double distance;
	
	public DNode(MapNode node, double distance){
		this.node = node;
		this.distance = distance;
	}
	public MapNode getMapNode(){
		return this.node;
	}
	public GeographicPoint getGeographicPoint(){
		return this.node.getGeographicPoint();
	}
	
	public double getDistance(){
		return this.distance;
	}

	public List<GeographicPoint> getNeighbors(){
		return this.node.getNeighbors();
	}
	
	public double getDistanceInEdge(GeographicPoint other){		
		return this.node.getDistanceInEdge(other);
	}
	
	@Override
	public int compareTo(DNode other){	
		
		return Double.compare(this.distance, other.getDistance());
	}
	
}

//class DNodeComparator implements Comparator<DNode>{
	
//	@Override
//	public int compare(DNode one, DNode other){	
//		
//		if (one.getDistance() < other.getDistance()){
//			return -1;
//		}
//		if (one.getDistance() > other.getDistance()){
//			return 1;
//		}
//	
//		return 0;
//	}
	
	
	
//}









