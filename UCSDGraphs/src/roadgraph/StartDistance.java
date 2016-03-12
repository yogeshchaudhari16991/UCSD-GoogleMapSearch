/**
 * 
 */
package roadgraph;

import java.util.Comparator;

import geography.GeographicPoint;

/**
 * @author Yogesh
 *
 */
public class StartDistance implements Comparable {
		private GeographicPoint vertex;
		private double distance;
		
		StartDistance(GeographicPoint vertex, double distance){
			this.vertex = vertex;
			this.distance = distance;
		}
		
		public double getLength(){
			return distance;
		}
		
		public GeographicPoint getVertex(){
			return vertex;
		}
		
		@Override
		public String toString(){
			return "Distance of Vertex: " + vertex + " from Start vertex is: " + distance;
		}

		@Override
		public int compareTo(Object o) {
			StartDistance s1 = (StartDistance) this;
			StartDistance s2 = (StartDistance) o;
			if(s1.getLength() >= s2.getLength()){
				return 1;
			} else {
				return -1;
			}
		}
		
}
