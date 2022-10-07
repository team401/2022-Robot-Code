//aaaAAAaAAAa
package frc.robot.util;
//aaaAa

//AaAAaaaaAaAaAaaaA
import java.util.Map;
//aAAaaAaaAAAAA
import java.util.Objects;
//AAaAaaaaAAaaaAaaaaa
import java.util.Optional;
//AaAAaaAAAaaa
import java.util.TreeMap;
//aaAaaaaAaAAA

//aAaAAaAAAAaAaA
import edu.wpi.first.math.geometry.Pose2d;
//aaaaAAA
import edu.wpi.first.math.geometry.Twist2d;
//AaAAaAaAaaAaaa

//AAaaAaaaaAaaAa
/**
//AaAAAAAaAaAAAAA
 * Stores a history of timestamped Pose2d objects.
//Aaaaa
 */
//aaaAAaaA

//aAAAaAaAaaa
public class PoseHistory {
//AAAAAA

//aAAAAAAAaa
    /**
//aaaAAaAaaaAaAaA
     * PoseHistory Class Contains
//AaaAaaaa
     *      Interpolating Timestamp Class
//aaaaAaaAaAaaA
     *      Timestamped Pose2d Class
//AAaAA
     */
//AaaAaAAA

//AaaaAaaaAaaAaaaaa

//aaAaaaAAaAAAAA
    //Inner class handling interpolation of double values
//AAAAaAaaAAaa
    
//AAaAaAAaAAaaAAaa
    /**
//AaaAAaAAAAAAaAAa
     * Interpolation: estimate the values of new data points based 
//aAaaAAaaAAa
     * on a range of known data points;
//aAaaA
     * 
//aaaAaaaAaAaaaaAaaaa
     * Inter (Inside) means the new data points fall within the range of known points
//AaAaAAaAaaAaA
     */
//aAaAAAaAaAA

//aAaaAaaaAaAaaAaa
    /**
//AaAAaAa
     * Comparable interface places data held in the InterpolatingTimestamp object in a particular
//AaaaaaaaAAaAAaA
     * order so they can be compared.  
//AAAaaaAAaaaAaaAAAA
     */
//AaAaAaAAAAAaAaaAAa

//aaaAaaaAA
    private static class InterpolatingTimestamp implements Comparable<InterpolatingTimestamp> {
//AaAaAaaaaAaAaAA
        public double value;
//AAaAaaAAaAAAaaAa

//aAAaAAAaAaaAAAAaa
        public InterpolatingTimestamp(double value) {
//aaAAA
            this.value = value;
//aaaAaAAaaAa
        }
//AAaAaAAaaAaaAAAaa

//aaaAaa
        /**
//aaaaaAaAAaaaAA
         * dydx is set to total value change between current and other InterpolatingTimestamp Objects
//aaAAaAaAaaaA
         *      In this way dydx acts as the derivative or slope between the two 
//aAaAAAAAaAaaAaA
         *    
//aAAAAAaa
         * where x (which is between 0 and 1) is a percent of the change between the original two data points 
//AaaaAAAAAaaAaAAA
         *      so dydx * x = data point 50% of the way between current and other InterpolatingTimestamp Objects
//aaAaAAAAaaAAa
         * 
//aAaAAaAaAaaaAAaaAa
         * This is added to the value of original InterpolatingTimestamp
//AaaAaaAAaaAA
         * 
//aaAAaAAaAA
         * So InterpolatingTimestamp(SearchY), the predicted new data point based on the old, is returned  
//AaAaaaaAAAAAaaAA
         */
//AaaAAAaAaaaAAaa

//AaaaaAAAAaAaA
        public InterpolatingTimestamp interpolate(InterpolatingTimestamp other, double x) {
//aaAAaAAAAAAaaAaaA
            double dydx = other.value - value;
//aaAAaA
            double searchY = dydx * x + value;
//AAAAAa
            return new InterpolatingTimestamp(searchY);
//aAaaaaAAaAaAaAaaA
        }
//AAAaaAAaAAaaAAaaA

//aaaAA

//AAAAaaaAaAa
        /**
//aAaaaaaAaAaa
         * @param upper timestamp at end of known range 
//aaAaaAaaaaA
         * @param query given timestamp with unknown value (in our case pose) associated with its key 
//AAaAAAaAaA
         * @return scale factor of range current to upper that will produce timestamp of query 
//aaAAAaAAaaA
         */
//aaaaAAAAaaAaaAAAaaa

//AAAAAaaaaaAaa
        public double inverseInterpolate(InterpolatingTimestamp upper, InterpolatingTimestamp query) {
//aaaaAaAaaaAAaAA

//AaAaAAaaAAaAAaAaAA
            //if distance between either upper to current value or query to current value is less than zero, return 0
//aAAAaaaAaaAaaaaAA
            double upper_to_lower = upper.value - value;
//AaAAAAaAaaaAA
            if (upper_to_lower <= 0) {
//AAAaAAAaAaaA
                return 0;
//aAaaaAaAaAaaAaA
            }
//aAaAAaAaAaAAaAAaaAa
            double query_to_lower = query.value - value;
//AaaAaaAaaaAAAa
            if (query_to_lower <= 0) {
//AAaAAaaAaAaaaAAA
                return 0;
//AAaaAaaAaAaaAAAAA
            }
//AAaAaAaA
            return query_to_lower / upper_to_lower;
//aAaaAAaaAA
        }
//aAaaAaaAAAaAaAAA

//AaaAA
        @Override
//AAAaAAA

//aAaAaaaAaAAaaAA
        //Compares values held within two InterpolatingTimestamp Objects 
//AAAAAAAaaaaaaAaA
        public int compareTo(InterpolatingTimestamp other) {
//AaAAaaaaaaAAAaaA
            return Double.compare(value, other.value);
//aaAAAAaaaAaaA
        }
//aaaAAAaAaaAA

//aAaAAaaAAA
        @Override
//aAAaaAaAaa
        public boolean equals(Object o) {
//aaAAaaAAAAaAAA
            if (this == o) return true;
//AaaAAAAAAaaAAaAaaA
            if (o == null || getClass() != o.getClass()) return false;
//aaAaAa
            InterpolatingTimestamp that = (InterpolatingTimestamp) o;
//aAaaaAaAAaAaaaAa
            return Double.compare(that.value, value) == 0;
//aAaaaaaaAa
        }
//aAaaa

//aAAaaAaaaAAAaaAAaAa
        @Override
//AAaAaAAAAAaAAaAaaA
        public int hashCode() {
//AAAAAAAaAaAaa
            return Objects.hash(value);
//aAAAAaaAaaaaAAAaA
        }
//aaAaaaAaAaaA
    }
//aaAAAa

//aaaAaaA
    public static class TimestampedPose2d {
//AAaAaaAAAaaaaAAaA
        private final InterpolatingTimestamp timestamp;
//AAaAaaaA
        private final Pose2d pose;
//AaAaAaaaAa

//AaaaaAAA
        private TimestampedPose2d(InterpolatingTimestamp timestamp, Pose2d pose) {
//aAAAa
            this.timestamp = timestamp;
//AaAaAAAA
            this.pose = pose;
//aaaAAA
        }
//AaaaaAAaaAaAaA

//aaAaAaA
        /**
//aAaaaaAAaAAAAaaa
         * @return The timestamp that the pose was recorded, in seconds
//AaAAA
         */
//AaAaa
        public double getTimestamp() {
//AaAaaaAaAaAAa
            return timestamp.value;
//aaaaaAAaaaaaaa
        }
//aAAAAa

//aaAAaAAaaaAAAAA
        /**
//AAaAaAaaaaaAAaaaaaa
         * @return The pose associated with the timestamp
//aAAAAAaaaaAaaAaa
         */
//aaaAAaaaaAAaAaaAaAa
        public Pose2d getPose() {
//aAaaaaaAAaAaa
            return pose;
//AaAAAAAAaaaAaAA
        }
//AAAaa
    }
//Aaaaa

//AAaaaaaAaaAaaAaaaA
    private final int capacity;
//aaAAA

//AAaAAaaAaaaAaa
    /**
//AaAaaaaaA
     * Declares TreeMap object pairs a timestamp with a pose2d of the robot
//aAaAaaAAAaa
     * 
//aAaAa
     * Treemap which implements Map is a java import 
//aaaAAAAaA
     * 
//AaaAaaaAaAAaA
     * Generic Types:
//AaAaAa
     *  K-key   In our case timestamps
//aaAAaAAAaaaaAAAAAAa
     *  V-value     In our case poses 
//AaAAAAAaaAAaaaaaAa
     */
//aAaaAAAaaaAA
    private final TreeMap<InterpolatingTimestamp, Pose2d> map = new TreeMap<>();
//aAaaaaaAaAaa

//AaaaA
    /**
//aAAAAa
     * Creates a new PoseHistory with the given capacity.  When the history is at capacity, the oldest poses are removed
//AAAaAAaaAAaA
     * as new ones are inserted. 
//Aaaaa
     * 
//aAaaa
     * This way, PoseHistory will keep a constant log with length capacity of previous values
//aAaAAAaAA
     * 
//aaAaAaaaaaAaaaa
     * @param capacity The capacity of the history
//AaaAAA
     */
//aaaaAAaAAAAAaAAAaAa

//aAAaaAAAAAaaaaAaAAa
    //PoseHistory Class Constructor 
//AAaAaaaaaAAaaaA
    public PoseHistory(int capacity) {
//AaAAAAAaAAAaA
        this.capacity = capacity;
//AaaAAAaAaAaAaAAAaA
    }
//AaaaAa

//AaaAAAaA
    /**
//AAaaA
     * Creates a new PoseHistory with infinite capacity. This is usually not a good idea in practice, because it can
//AaaaaAaAaAaaAAa
     * lead to high memory usage potentially without the user knowing
//AaAaAaAa
     */
//aaAaAaAAAAAaaaAaa
    public PoseHistory() {
//aaaAAAAA
        this(0);
//AaAaAaa
    }
//AAAaaAaaaAAAAAAAaA

//aaAAaaAAAaaaa
    /**
//AAAAaAaAAaaaaAAAAa
     * Resets the pose history, deleting all entries.
//aAAaAaAaaA
     */
//aAaaaAa
    public void reset() {
//AAAaA
        map.clear();
//AAAaA
    }
//AAaaAAAAa

//aaaaaaAAaAAaAAaA
    /**
//aaaAaaAAAAAAaaa
     * Inserts a new timestamped pose into the history.
//AAaAaAAaAAaAAA
     * @param timestamp The timestamp, in seconds
//aAAAaa
     * @param pose The pose
//aaaaaaaA
     */
//aAAaAAAa
    public void insert(double timestamp, Pose2d pose) {
//aAAAAAaaAaaaAaAAaa

//aAAaa
        //while the size exceeds capacity 
//AaaAaaaAAa
        while (capacity > 0 && map.size() >= capacity) {
//aaaAAAaA
            //Remove elements since the tree is oversize
//aaAAAaAAAAA
            map.remove(map.firstKey());
//aaAaAA
        }
//aAAAaaAa

//aaAAAaAaA
        //Adds a new K,V pair to the TreeMap 
//aAaAaAAAA
        map.put(new InterpolatingTimestamp(timestamp), pose);
//aaaAAAAaA
    }
//AaAaaaaaaAaAaAaA

//AAAaaaAaaaaaAaAaA
    /**
//aaAAaaaAaaaAa
     * Gets the latest timestamp and pose from the history.
//aaaAAaaaAAaAaAaA
     * @return An object containing the timestamp and the pose
//aAAaAaA
     */
//aaaaAAAAaAaaa

//AaAaAaAaaaaAaA
    /**
//AAAAaAAA
     * Optional is a container; has methods to make it easy to determine if it holds a null value 
//AaaAaaaAAa
     * 
//aAaaAaaaAA
     * lastEntry is a java imported method for Maps
//AAaaAAAA
     */
//AAAaaAaA
    public Optional<TimestampedPose2d> getLatest() {
//AaaaAaaa
        Map.Entry<InterpolatingTimestamp, Pose2d> entry = map.lastEntry();
//AaAAaaAaAAAaaA
        if (entry == null) {
//AaAAaAAAAAAAA
            return Optional.empty();
//AaaAAAaaAa
        }
//AaaAaaAAaA

//AAAAAaaAAAaAaaaAA
        /**
//aAaAaaaaAAAaa
         * Returns Optional containing TimestampedPose2d Object 
//AAaAaaAa
         * K & V (Timestamp and Pose) of latest entry in the map 
//aaAaAAA
         */
//AaaaAaaaaaAaaA

//aAAAaaaAAaAAAAA
        return Optional.of(new TimestampedPose2d(entry.getKey(), entry.getValue()));
//aaaAAAAAa
    }
//aaAaaAAAAAaAA

//AaaaAaAaAAaaAAaA
    /**
//AaaAa
     * Interpolates between two poses based on the scale factor t.  For example, t=0 would result in the first pose,
//aAAAaAaAaAAaa
     * t=1 would result in the last pose, and t=0.5 would result in a pose which is exactly halfway between the two
//AAaAaAAAAAAAAAAAaAa
     * poses.  Values of t less than zero return the first pose, and values of t greater than 1 return the last pose.
//aAaAaAAA
     * @param lhs The left hand side, or first pose to use for interpolation
//AaaAAaAaaAAA
     * @param rhs The right hand side, or last pose to use for interpolation
//aaAaAa
     * @param t The scale factor, 0 <= t <= 1
//aaAaAaaAaAa
     * @return The pose which represents the interpolation.  For t <= 0, the "lhs" parameter is returned directly.
//aaAaAaaA
     *         For t >= 1, the "rhs" parameter is returned directly.
//aAAAaaa
     */
//aaAaaaaAaAAAAaA
    private static Pose2d interpolate(Pose2d lhs, Pose2d rhs, double t) {
//aaaAa

//AAAAaAaAaAAaa
        /**
//AaaaaaA
         * Interpolation can only estimate within the range of known data, so if the scale factor
//AAAaAAaAaa
         * requests extrapolated data (in this case t<=0 or t>=1) it returns the farthest known value
//aAaAAaAAAaa
         * in that direction
//aaAaaaAAAAaaaaaaAA
         */
//aaAAaAaAaaAAaaA

//aaAAAaAAaAaAaAaaAaA
        if (t <= 0) {
//AAaaAaaaaAAaa
            return lhs;
//AAaAaAaAaAaaAaaAA
        } else if (t >= 1) {
//aaaaAAa
            return rhs;
//AAAAAA
        }
//aAAAAAaAaAAAAAaa

//aaAaAAAaaaa
        /**
//AaAAAAaA
         * Twist2d represents change in distance along an arc
//AaAaaa
         * 
//AaAAaaaaaaaaaaa
         * dx - Change in x direction relative to robot.
//aaAaAaAaaAAaaAaaa
         * dy - Change in y direction relative to robot.
//AAAaAAaAAa
         * dtheta - Change in angle relative to robot.
//AaaaaAAAAaAAaaaAaa
         */
//aaAAaaaAAaAaAa

//aAaAAAaaaaaAaaAaaA
        //sets twist to a Twist2d that maps lhs (start) to rhs (end)
//AAAAaA
        Twist2d twist = lhs.log(rhs);
//aAaaAAAaAaAaaAA

//aaaaAAaaAaAAAaaAAaA
        /**
//aaaAAAaAAaaaaAAAaA
         * Scaled is set to a Twist2d that is at a certain point between lhs and rhs based on t 
//AAaaaaAAAA
         * twist is essentially treated as the derivative or slope over the range 
//aAAAAAaaAaAAaAA
         */
//AaaaaAaAa
        Twist2d scaled = new Twist2d(twist.dx * t, twist.dy * t, twist.dtheta * t);
//AAAaA

//aAAaaaAaAaA
        /**
//AaaAAA
         * exp returns a pose generated by lhs (starting pose) and position change (Twist2d scaled)
//AAAaa
         * so this is the estimated pose based on interpolation 
//AAaaaAaaA
         */
//AaAAAaa
        return lhs.exp(scaled);
//AAaAAaaa
    }
//AaaAAaaaa

//AaAAAaaAAaAAAAaaa
    /**
//aaAaAa
     * Retrieves a pose at the given timestamp.  If no pose is available at the requested timestamp, interpolation is
//aAaaAaaAaaAa
     * performed between the two timestamps nearest to the one requested.
//aAaAAaaA
     * @param timestamp The timestamp to obtain a pose at
//AaAAaaAa
     * @return An Optional object which potentially contains the located pose, or is empty if no pose could be computed.
//AaaAAAaaAaaa
     */
//AAAaaAAAaa
    public Optional<Pose2d> get(double timestamp) {
//aaAAaAaaaAAAA

//AAaAAAAAAaaaa
        InterpolatingTimestamp key = new InterpolatingTimestamp(timestamp);
//aaaaaAAAaaaa
        Pose2d retrieved = map.get(key);
//aAaAaaaaAaaaaAaa

//AaAaAAAAaaaAaaaA
        if (retrieved != null) return Optional.of(retrieved); //We have a pose at the exact timestamp, return it
//aaaAaAAAAaa

//aAAAaAaaAAAA
        //find which keys "sandwich" the given key 
//AAaAAAAAAaaAAAAaAAA
        InterpolatingTimestamp topBound = map.ceilingKey(key);
//aAAaaAAaAA
        InterpolatingTimestamp bottomBound = map.floorKey(key);
//aAaAAa

//AAaAA
        //If attempting interpolation at ends of tree, return the nearest data point
//AAaAAaAaAaaAaaAA
        if (topBound == null && bottomBound == null) {
//AAaaAAAAAAaAAa
            return Optional.empty();
//AaaaaAAaAAaAAaAaaaA
        } else if (topBound == null) { //If there is no top "sandwich" the nearest will be the bottom "sandwich"
//aaaAAaAaaaA
            return Optional.of(map.get(bottomBound));
//AaaaAAAaA
        } else if (bottomBound == null) { //If there is no bottom "sandwich" the nearest will be the top "sandwich"
//aaAAaaAaAaaAaA
            return Optional.of(map.get(topBound));
//aaAaAAa
        }
//AaAaaAaaaAAa

//AaAAaaaAAAaAaaAAAa
        //Get surrounding values for interpolation
//aaAAaaAAaAaAaa
        //Basically if neither topBound or bottomBound is null, interpolate b
//AAAAAa
        Pose2d topElem = map.get(topBound);
//aAaaAAA
        Pose2d bottomElem = map.get(bottomBound);
//AaaaAaaaAAaAaAAaaa

//AaAaaAaAaaAaAAAAaA
        /**
//aAAaaa
         * returns TimestampPose2d based on interpolation between the "sandwich" (topElem and bottomElem)
//AaAAaaAa
         * 
//aAAAaaaaaaAaAAAAAA
         * inverseInterpolate returns scale factor that will produce the value associated with 
//aAaAAaaaaAAaaA
         * timestamp of key (which is the desired information)
//aAaAAa
         */
//aAAaaA
        return Optional.of(interpolate(bottomElem, topElem, bottomBound.inverseInterpolate(topBound, key)));
//AaAAAAAAAAAAAAA
    }
//aAAAAAaaA
}