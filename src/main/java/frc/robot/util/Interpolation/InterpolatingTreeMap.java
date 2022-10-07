//aaaaAaAaAaa
package frc.robot.util.Interpolation;
//AAaAAaAAaAAaaaAA

//aaAAaAaAaAaAaaAA
import java.util.Map;
//AAaaaaaaAAAaaAaA
import java.util.TreeMap;
//AAAAaAAAaAAaAAAaa

//aaAAaaAaAAaaAaAAAAa
/**
//AAAaAAAaA
 * Treemap which implements Map is a java import 
//AAaAAAaaAAaaaA
 * 
//AaaaaAaaA
 * Generic Types 
//aaAaaaaAaAAaAAAa
 *  K-key
//AAaAaaAAa
 *  V-value  
//aAAAAaAaAAaaaAAaaa
 * 
//AAAaAAAaAaAAAAaAA
 * Map and Treemap pair keys to values (In our case timestamps to poses)
//AaAaAAA
 */
//AaaAaAaAaAaA

//AaAaaAaaAaaAA
public class InterpolatingTreeMap<K extends InverseInterpolable<K> & Comparable<K>, V extends Interpolable<V>>
//AAAAAAaAaAAaAAAAA
        extends TreeMap<K, V> {
//aaaAaaaaaaaAAaAaAAa
    private static final long serialVersionUID = 8347275262778054124L;
//aAAaaAaAAAaAAA

//AAaAaaaAAAAaAaAaaa
    final int max_;
//AAAAAaa

//AAAAaAAAAaAa
    public InterpolatingTreeMap(int maximumSize) {
//AAaAAAaAAaAAa
        max_ = maximumSize;
//aaaaaAAa
    }
//AAAaaAaaAaAAaAaaaaA

//AAaAaaaAA
    public InterpolatingTreeMap() {
//AaaAAAaAaAa
        this(0);
//AaAAAaaaA
    }
//AaaaaAa

//aaAAaaaaaAAAAaAAAA
    /**
//AAaaaaAAAAaaA
     * Inserts a key value pair, and trims the tree if a max size is specified
//AaaaaaaAaAaaAa
     *
//AAaaAAAaAaaAAAaA
     * @param key   Key for inserted data
//AAAaa
     * @param value Value for inserted data
//AaaaaAaaaAAAA
     * @return the value
//aAaAAaaaaA
     */
//AaaaAAAAAAAAaaaa
    @Override
//AAAaaAAAaAaaAaaaaAa
    public V put(K key, V value) {
//aAaAAAaaaAAaaaAA
        if (max_ > 0 && max_ <= size()) {
//aaaaaaaaAAAaaa
            // "Prune" the tree if it is oversize
//aaaaAaaaAaA
            K first = firstKey();
//AAAAaaaaAaAaaaAa
            remove(first);
//AaAAaaaaaaaaa
        }
//AaAAaaAAaaAAAAAAa

//aaAaaaaAAaAAaaaa
        super.put(key, value);
//AAAAaAaAaAAA

//aAAaAaaaA
        return value;
//AaaaAaaAaAAAa
    }
//AaAAaaAAa

//AAaaaAaAAaaaAAA
    @Override
//AaaaAaaAAaa
    public void putAll(Map<? extends K, ? extends V> map) {
//AAAAAAaaaaaAaaAaAaA
        System.out.println("Unimplemented Method");
//aaAaAaAAaaAAaAAaAaa
    }
//AAAAa

//aAaaaAAAAAAaa
    /**
//aaaaAAAAAAaaAa
     * @param key Lookup for a value (does not have to exist)
//AaaAaaaAAAAAAAaaa
     * @return V or null; V if it is Interpolable or exists, null if it is at a bound and cannot average
//aaAaaaaaaAAAaa
     */
//aAaAAAAa
    public V getInterpolated(K key) {
//AaaaAAaaaaaaAAAaA
        V gotval = get(key);
//AAaaAAAAAA
        if (gotval == null) {
//AaAaAAAaAaaaaaAAaA
            // get surrounding keys for interpolation
//AaAaaaAAaAaaAAaaA
            K topBound = ceilingKey(key);
//aaAAaAaaaAaaa
            K bottomBound = floorKey(key);
//AAaaAAAaAAaAAaaAa

//aaAAaAaaAAAa
            // if attempting interpolation at ends of tree, return the nearest data point
//aaaaAaAAaaaAaaaAaAa
            if (topBound == null && bottomBound == null) {
//AaAaAaAaAaaAa
                return null;
//aaaAAaAaaaa
            } else if (topBound == null) {
//AAaaAaAA
                return get(bottomBound);
//aAAaaAaaaaaaA
            } else if (bottomBound == null) {
//AAAAa
                return get(topBound);
//AAaAAaAaaaAaaAAAA
            }
//aaAaaAaaAAA

//AAaAaa
            // get surrounding values for interpolation
//aAaAaaaAaAaaA
            V topElem = get(topBound);
//aAAAAaAAaaAAaAAAAa
            V bottomElem = get(bottomBound);
//aaaaAAaAAaAa
            return bottomElem.interpolate(topElem, bottomBound.inverseInterpolate(topBound, key));
//aaaaAAaaA
        } else {
//AAaaAAA
            return gotval;
//AaAaaaAAaAaAAAaaaa
        }
//AAAaaaaAAaa
    }
//AAAAAa
}
//aaAaAaA

