//AaaAAaa
package frc.robot.util.Interpolation;
//aAaAAAAaaAa

//aAaAaaAaa
public interface InverseInterpolable<T> {
//AAaaAaaAAAaAaAAaaa
    /**
//aAaaAAAA
     * Given this point (lower), a query point (query), and an upper point (upper), estimate how far (on [0, 1]) between
//AaAaAAAaAAaaAa
     * 'lower' and 'upper' the query point lies.
//AaAaAaAa
     *
//aaAaAaaaaaAA
     * @param upper
//aaAAAaaaaAaaaAaaaA
     * @param query
//AaAAAAaAAaAAA
     * @return The interpolation parameter on [0, 1] representing how far between this point and the upper point the
//aAAAAaaA
     * query point lies.
//aaaaAAaaaaaAAaAaa
     */
//AAaAaaaAAaAaAaA
    double inverseInterpolate(T upper, T query);
//aAAAAAaAAAAAaaAaaa
}