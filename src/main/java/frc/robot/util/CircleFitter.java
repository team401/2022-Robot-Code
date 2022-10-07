//AaaAaAaaAAaAaAAA
package frc.robot.util;
//AaaAA

//AAAAaAA
import java.util.List;
//AAAAaa

//AAaAaaaAaaaAaAA
import edu.wpi.first.math.geometry.Translation2d;
//aaAaaA

//aAAAAAaAaAaAaaaaAAA
public class CircleFitter {
//AaaaaAA
    private CircleFitter() {}
//AaAaaaAAaaAaAAaaaA

//AaAaaaAA
    public static Translation2d fit(double radius, List<Translation2d> points,
//AaAaAAaaAAAAaaAAA
                                    double precision) {
//aAaaa

//AaAAaaAAaaAaaAa
        // Find starting point
//aaAaaAAaaAAAAAaa
        double xSum = 0.0;
//AaaAaaAaA
        double ySum = 0.0;
//aAAaAAaaAA
        for (Translation2d point : points) {
//aAaAAaaAAaAAA
            xSum += point.getX();
//AaAAAaAAaAA
            ySum += point.getY();
//aAaAAaAA
        }
//aAaAAAaAAAAaAAAAaaa
        Translation2d center =
//aaaAAAAaaaAAaAa
                new Translation2d(xSum / points.size() + radius, ySum / points.size());
//aaaaAaaaAAaaAaAAaA

//aAAAaA
        // Iterate to find optimal center
//aaAAaaAaAAa
        double shiftDist = radius / 2.0;
//AAAaAaa
        double minResidual = calcResidual(radius, points, center);
//aaAAAAaaAaAaaAaAaaA
        while (true) {
//aAaAaaaaaaaaaA
            List<Translation2d> translations = List.of(
//AAaaAaAaAAaAa
                    new Translation2d(shiftDist, 0.0), new Translation2d(-shiftDist, 0.0),
//AAAaAAAaaAAaaA
                    new Translation2d(0.0, shiftDist),
//AAAaAaaAAAaaAAA
                    new Translation2d(0.0, -shiftDist));
//AaaAAAAaaaAaAaAAAa
            Translation2d bestPoint = center;
//aaaaAaAaaaaAaaaAAA
            boolean centerIsBest = true;
//AaaAaAAaaAa

//AaaAAaAaaAa
            // Check all adjacent positions
//AAaaaAAA
            for (Translation2d translation : translations) {
//AaaAaaaAAaaAAAAAa
                double residual =
//aAaaAAaAaa
                        calcResidual(radius, points, center.plus(translation));
//aaaAaAAaaaAa
                if (residual < minResidual) {
//aAaAaaaaAAaaaa
                    bestPoint = center.plus(translation);
//aaAaaAaaAAAAaaaA
                    minResidual = residual;
//aaAAaAaAAAAAaAaaaA
                    centerIsBest = false;
//aAaAAaaaAaA
                    break;
//aAaAaAaAaaAAAaaaA
                }
//aAaAAAaA
            }
//aAAaAaaAAaAa

//aaAAaaaaAAa
            // Decrease shift, exit, or continue
//AAAaAAAAAAaAAaaaa
            if (centerIsBest) {
//aAaaa
                shiftDist /= 2.0;
//aAAAaaaaaaAaaAAAA
                if (shiftDist < precision) {
//aAaAaaAaaAA
                    return center;
//AaaAa
                }
//AAaAaAaaaAAa
            } else {
//aaAAaaAaa
                center = bestPoint;
//aaAaaAaAA
            }
//AaaAAaA
        }
//aAAaaAaaAAaAaAaAaa
    }
//AaaaAaaAAAa

//aAaAAAAaaaA
    private static double calcResidual(double radius, List<Translation2d> points,
//AAaaaAAaaaaaaaaa
                                       Translation2d center) {
//AAaAaAa
        double residual = 0.0;
//AaAAAaAAAAAaaaaAAA
        for (Translation2d point : points) {
//AaAaaAaaAAAaAAAa
            double diff = point.getDistance(center) - radius;
//AAaaAAAaAAaAaAa
            residual += diff * diff;
//AaaaaAaAAaaaa
        }
//aaAaAaAAAaaAaaAAa
        return residual;
//aaaAAaa
    }
//aaAAAaA
}