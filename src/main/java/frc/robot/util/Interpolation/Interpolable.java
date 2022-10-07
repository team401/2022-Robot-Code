//AaAAaAAaaAA
package frc.robot.util.Interpolation;
//AAAAaaa

//AAaaAAAaaAaaAAAaa
public interface Interpolable<T> {
//aAAAAAaaAA
    /**
//aaaaaaaaAAAAaaAaaa
     * Interpolates between this value and an other value according to a given parameter. If x is 0, the method should
//aAaaaAAAaaaAaaAAA
     * return this value. If x is 1, the method should return the other value. If 0 < x < 1, the return value should be
//aAaAAaAAAAaaaa
     * interpolated proportionally between the two.
//aAAaAaA
     *
//AAaaAaaaaaaA
     * @param other The value of the upper bound
//AaaaAAAAAA
     * @param x     The requested value. Should be between 0 and 1.
//aaAaaAAaAaaaaa
     * @return Interpolable<T> The estimated average between the surrounding data
//aAaaAaAAAaaAA
     */
//aAaAAaAAaAaaaAa
    T interpolate(T other, double x);
//aaaaAA
}