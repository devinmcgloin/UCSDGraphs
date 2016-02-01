/**
 * @author UCSD MOOC development team and YOU
 * <p/>
 * A class which reprsents a graph of geographic locations Nodes in the graph are intersections between
 */
package roadgraph;


import geography.GeographicPoint;
import util.GraphLoader;

import java.util.*;
import java.util.function.Consumer;

/**
 * @author UCSD MOOC development team and YOU
 *
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
    //TODO: Add your member variables here in WEEK 2
    private int numEdges;
    private int numVert;
    private Hashtable<GeographicPoint, List<GeographicPoint>> map;
    private HashSet<GeographicPoint> vertices;

    /**
     * Create a new empty MapGraph
     */
    public MapGraph() {
        // TODO: Implement in this constructor in WEEK 2
        numEdges = 0;
        numVert = 0;
        map = new Hashtable<GeographicPoint, List<GeographicPoint>>();
        vertices = new HashSet<GeographicPoint>();
    }

    public static void main(String[] args) {
        System.out.print("Making a new map...");
        MapGraph theMap = new MapGraph();
        System.out.print("DONE. \nLoading the map...");
        GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
        System.out.println("DONE.");

        System.out.println(theMap);

        GeographicPoint start = new GeographicPoint(7,3);
        GeographicPoint end = new GeographicPoint(8,-1);

        System.out.println(theMap.bfs(start,end));

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

    /**
     * Get the number of vertices (road intersections) in the graph
     * @return The number of vertices in the graph.
     */
    public int getNumVertices() {
        //TODO: Implement this method in WEEK 2
        return numVert;
    }

    /**
     * Return the intersections, which are the vertices in this graph.
     * @return The vertices in this graph as GeographicPoints
     */
    public Set<GeographicPoint> getVertices() {
        //TODO: Implement this method in WEEK 2
        return (Set<GeographicPoint>) vertices.clone();
    }

    /**
     * Get the number of road segments in the graph
     * @return The number of edges in the graph.
     */
    public int getNumEdges() {
        //TODO: Implement this method in WEEK 2
        return numEdges;
    }

    /** Add a node corresponding to an intersection at a Geographic Point
     * If the location is already in the graph or null, this method does
     * not change the graph.
     * @param location  The location of the intersection
     * @return true if a node was added, false if it was not (the node
     * was already in the graph, or the parameter is null).
     */
    public boolean addVertex(GeographicPoint location) {
        // TODO: Implement this method in WEEK 2
        if(location == null || vertices.contains(location))
            return false;

        vertices.add(location);
        map.put(location, new LinkedList<>());
        numVert++;
        return true;

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

        //TODO: Implement this method in WEEK 2

        if(!vertices.contains(from) || !vertices.contains(to))
            throw new IllegalArgumentException("Arguments to addEdge did not exist in the graph.");
        if(length < 0)
            throw new IllegalArgumentException("length of edge was negative");
        if(roadName == null || roadType == null)
            throw new IllegalArgumentException("Labels must not be null");

        numEdges++;
        map.get(from).add(to);
//        map.get(to).add(from);

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
        Consumer<GeographicPoint> temp = (x) -> {
        };
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
                                     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        // TODO: Implement this method in WEEK 2
        if(start.equals(goal)) {
            List<GeographicPoint> list = new LinkedList<>();
            list.add(goal);
            return list;
        }
        HashSet<GeographicPoint> visted = new HashSet<>();
        LinkedList<GeographicPoint> queue = new LinkedList<>();
        HashMap<GeographicPoint, GeographicPoint> path = new HashMap<>();

        visted.add(start);
        queue.offer(start);

        while(!queue.isEmpty()){
            GeographicPoint current = queue.poll();
            System.out.println(current);
            if(current.equals( goal)) {
                LinkedList<GeographicPoint> finalPath = new LinkedList<>();
                finalPath.add(goal);
                GeographicPoint g = path.get(goal);
                while(!g.equals(start)){
                    finalPath.addFirst(g);
                    g = path.get(g);
                }
                finalPath.addFirst(start);
                return finalPath;
            }
            for(GeographicPoint neigh : map.get(current)){
                if(visted.contains(neigh))
                    continue;
                visted.add(neigh);
                path.put(neigh, current);
                queue.offer(neigh);
            }


        }

        // Hook for visualization.  See writeup.
        //nodeSearched.accept(next.getLocation());

        return null;
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
        Consumer<GeographicPoint> temp = (x) -> {
        };
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
                                          GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        // TODO: Implement this method in WEEK 3

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
        Consumer<GeographicPoint> temp = (x) -> {
        };
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
                                             GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        // TODO: Implement this method in WEEK 3

        // Hook for visualization.  See writeup.
        //nodeSearched.accept(next.getLocation());

        return null;
    }

    @Override
    public String toString() {
        final StringBuffer buff = new StringBuffer();
        for(GeographicPoint p : vertices)
            buff.append(p).append(";\t ");
        final StringBuffer sb = new StringBuffer("MapGraph{");
        sb.append("vertices=").append(buff);
        sb.append(", numVert=").append(numVert);
        sb.append(", numEdges=").append(numEdges);
        sb.append('}');
        return sb.toString();
    }
}
