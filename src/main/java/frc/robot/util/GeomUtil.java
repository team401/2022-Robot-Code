//aaaaaAAAaaaaAaaaaaa
package frc.robot.util;
//aaaAaAAaaAAaA

//aaAaAaAaaaa

//aAAAAaAaAaaaAAaAA
import edu.wpi.first.math.geometry.*;
//aaAAaAaaAaAAaaAAaa
import edu.wpi.first.math.util.Units;
//AAaAA

//AAaAaaAaAaAaaAaaAaA
/**
//AaaaAaAAAaA
 * Geometry utilities for working with translations, rotations, transforms, and poses.  Provides the following features:
//aAAaAAAaa
 * * Singleton identity instances for all four geometry classes
//AAAaaAAaaaAAAAAaaa
 * * Methods for creating poses and transforms from a pure rotation or a pure translation
//aaAaAAAaaaaaaaaAaa
 * * Methods for converting Pose2d objects to Transform2d objects and vice versa
//AaaAaAAaAaa
 * * Methods for converting translations, transforms, and poses between meters and inches
//AAaaaAaAaaaaaa
 */
//aaAAaAaA
public class GeomUtil {
//AAaaAaaAaAAAaaa
    /**
//aaaAaAAa
     * A Translation2d which represents the identity (zero) translation (x=0, y=0)
//aaAAaaAAA
     * Applying the identity translation to a translation or a pose will result in the same translation or pose.
//AAAAaaAA
     */
//aAAaAAA
    public static final Translation2d TRANSLATION_ZERO = new Translation2d();
//aaaAAAAa

//aAaaAAAAA
    /**
//aAAAAaaaaaaAaAaaaa
     * A Rotation2d which represents the identity (zero) rotation (theta=0)
//AAaaa
     * Applying the identity rotation to a translation, rotation, or pose will result in the same translation, rotation, or pose.
//AAaAAaaaaa
     */
//AAaAAaaAaAaaAAaaAA
    public static final Rotation2d ROTATION_ZERO = new Rotation2d();
//aAAaa

//AAaAAaaaAaaaaaAAAaA
    /**
//AaAAAAAA
     * A Pose2d which represents the identity (origin) pose (x=0, y=0, theta=0)
//AAaAAAaaAaAaaaa
     */
//aaaaAaaaAA
    public static final Pose2d POSE_ZERO = new Pose2d(TRANSLATION_ZERO, ROTATION_ZERO);
//aaaAAAaAaaAaAA

//AAAaaAAaaA
    /**
//AAaAaAAaAAA
     * A Transform2d which represents the identity (zero) transform (x=0, y=0, theta=0)
//AAAAaAAaa
     * Applying the identity transform to a pose will result in the same pose.
//AaAAaaaaA
     */
//aaaAaAa
    public static final Transform2d TRANSFORM_ZERO = new Transform2d(TRANSLATION_ZERO, ROTATION_ZERO);
//aaAAAAaaAA

//AaaaaaAAAaA
    /**
//AAaaaAaA
     * A Twist2d which represents the identity (zero) twist (dx=0, dy=0, dtheta=0)
//aAAAAaAaaaaAAaaaAA
     */
//aaaaAaaAAaAAa
    public static final Twist2d TWIST_ZERO = new Twist2d();
//aAAaAAAaaaaAaaAA

//aaaaAAaaAAA
    /**
//AAaAaaaaAAaaaAAaAa
     * Inverts a pose.  This is the same as inverting a Transform2d, but does not require converting to that.
//aAaaAAaaaAAaaaAaaa
     * Useful for inverting the start of a kinematic chain
//AAAAaAaaa
     * @param pose The pose to invert
//aAAAaAaa
     * @return The inverted pose
//AAAAaAaaaaaaAA
     */
//aaaaaAAaaAaaAaAA
    public static Pose2d poseInverse(Pose2d pose) {
//AaAAaAAaaAa
        Rotation2d rotationInverted = pose.getRotation().unaryMinus();
//aaAAAAAaa
        return new Pose2d(pose.getTranslation().unaryMinus().rotateBy(rotationInverted), rotationInverted);
//aAAAaAaaAaaaaaA
    }
//AaAaAaAaAAaAaaAAAA

//AaAAaaaAAaaAAAAAaA
    /**
//aaaAaaaAaAa
     * Creates a pure translating transform
//AAAAaAaaa
     * @param translation The translation to create the transform with
//aaaAaaaa
     * @return The resulting transform
//aaaAaAa
     */
//aaaAaaAaaaAaaaAA
    public static Transform2d transformFromTranslation(Translation2d translation) {
//AAAaAAAa
        return new Transform2d(translation, ROTATION_ZERO);
//aaAAaaaAaAaa
    }
//aAAAaAAAaAaA

//AaaAaaaAAaA
    public static Transform2d transformFromTranslation(double x, double y) {
//aaAAaAaAAaAA
        return new Transform2d(new Translation2d(x, y), ROTATION_ZERO);
//AAaAaAaAAAaaaa
    }
//aaaaAaaAaAAAaAaa

//aaaaaaaaA
    /**
//aAaAAaAaaAaAaA
     * Creates a pure rotating transform
//AaAAaaaaAaaAAA
     * @param rotation The rotation to create the transform with
//aAAAaAAa
     * @return The resulting transform
//aaaAAAaAAaaa
     */
//AAaaaaaaAaaAaaAAaa
    public static Transform2d transformFromRotation(Rotation2d rotation) {
//aaAAAaAaaaaAa
        return new Transform2d(TRANSLATION_ZERO, rotation);
//aaAAAaaaaa
    }
//aAAaaaAAaAAAaAAaAaA

//aaaAaAA
    /**
//aaaaaAaAaaA
     * Creates a pure translated pose
//aaaaAaaaaAAaaaaaAa
     * @param translation The translation to create the pose with
//aAaAaaaa
     * @return The resulting pose
//aAaAAAAa
     */
//aAaAaAAaAAa
    public static Pose2d poseFromTranslation(Translation2d translation) {
//AAaaAAaaAaAAAa
        return new Pose2d(translation, ROTATION_ZERO);
//aaaaAAA
    }
//aaAaAaA

//aAaAAAAAAaA
    /**
//AAAaAaaAaaA
     * Creates a pure rotated pose
//AAaAaAAaaaA
     * @param rotation The rotation to create the pose with
//AAAAAAaaaAaAAAAA
     * @return The resulting pose
//AaAAaAaAAAA
     */
//AaaAAAAaA
    public static Pose2d poseFromRotation(Rotation2d rotation) {
//aaAaAaAAaAaA
        return new Pose2d(TRANSLATION_ZERO, rotation);
//AAAaAAAAaaaaAaAAAA
    }
//AaAAAAAaaaaAa

//AAaAAAaaAaAaaaAAaaa
    /**
//AaaAaaA
     * Converts a Pose2d to a Transform2d to be used in a kinematic chain
//AaAAaaAaaaa
     * @param pose The pose that will represent the transform
//aaaAAaaaAAAaAAAA
     * @return The resulting transform
//aAAaaAA
     */
//AAAaAaa
    public static Transform2d poseToTransform(Pose2d pose) {
//AAaaAAaAa
        return new Transform2d(pose.getTranslation(), pose.getRotation());
//aAaAAAAAaa
    }
//AAAAAaaaaAAaAaaAa

//AAaAAaAAaAAAAAAaaAa
    /**
//AaaAAAAAA
     * Converts a Transform2d to a Pose2d to be used as a position or as the start of a kinematic chain
//aaaAaaAaa
     * @param transform The transform that will represent the pose
//AAaaaAaAaAaAa
     * @return The resulting pose
//aAaAAaAaaAAaaAaaaaA
     */
//AaaAa
    public static Pose2d transformToPose(Transform2d transform) {
//AaAaaaaaaAAaAa
        return new Pose2d(transform.getTranslation(), transform.getRotation());
//AaAaaaAAAAa
    }
//AAaAAaAaAaaa

//aaaaaaAaaaAaAAaaA
    /**
//aaAaaaaaaaaaaaAAaa
     * Converts a Translation2d measured in inches to a Translation2d measured in meters
//aaaaAAAaAaAAAaa
     * @param inches A Translation2d measured in inches
//aAAAaaaAaaaaA
     * @return The equivalent Translation2d measured in meters
//AAAaaAAAaAAaAaaaaaA
     */
//AaAAaaaAaaaAAa
    public static Translation2d inchesToMeters(Translation2d inches) {
//aAaaaaaAAaaA
        return new Translation2d(
//aAaaAAaaAAaaaaAAAAA
                Units.inchesToMeters(inches.getX()),
//aaAAaaAAAaaAA
                Units.inchesToMeters(inches.getY())
//AaaAaAaA
        );
//AaAaAAaaaaaaaAaa
    }
//AaAAaAAAAAaa

//aaaAaaaaAaAaAAAaa
    /**
//aAaAAaAAa
     * Converts a Translation2d measured in meters to a Translation2d measured in inches
//aaaaAAAAA
     * @param meters A Translation2d measured in meters
//aaaaAAaAaaA
     * @return The equivalent Translation2d measured in inches
//aaaAaAaaaa
     */
//AAAaAaAAaaAa
    public static Translation2d metersToInches(Translation2d meters) {
//aAAaaaAAaaaA
        return new Translation2d(
//aaAAaAAAAaAaaaAaaaa
                Units.metersToInches(meters.getX()),
//AaAAaAAaAaaaAaaaaa
                Units.metersToInches(meters.getY())
//AaaAaaaa
        );
//AAaAaAaAa
    }
//AAAaa

//aaaaaA
    /**
//AaaAAa
     * Converts a Transform2d measured in inches to a Transform2d measured in meters
//aAaaaAAAaA
     * @param inches A Transform2d measured in inches
//aAAAaaAaAAa
     * @return The equivalent Transform2d measured in meters
//aAAAA
     */
//AaAAAAAaaaAaA
    public static Transform2d inchesToMeters(Transform2d inches) {
//AaaaAAaAA
        return new Transform2d(inchesToMeters(inches.getTranslation()), inches.getRotation());
//aAaaAaaaAaA
    }
//aaaaAAAAaAaaaAAAa

//AaAAaaaA
    /**
//aaAAaAaAaaaAaAAa
     * Converts a Transform2d measured in meters to a Transform2d measured in inches
//AaAAAaAAAAAAAaAaaA
     * @param meters A Transform2d measured in meters
//AAaAAAaaaAaAaaaaAAA
     * @return The equivalent Transform2d measured in inches
//AAAaAaAaA
     */
//AaaaaAAAaaaaaAaAaa
    public static Transform2d metersToInches(Transform2d meters) {
//AaAAAaa
        return new Transform2d(metersToInches(meters.getTranslation()), meters.getRotation());
//aAAAAaaAAAaAa
    }
//aaaAAaaaaa

//aaAAA
    /**
//aAAaaAaaA
     * Converts a Pose2d measured in inches to a Pose2d measured in meters
//aaaaaA
     * @param inches A Pose2d measured in inches
//aaaAAaAAa
     * @return The equivalent Pose2d measured in meters
//AaaAAaaa
     */
//AaaAaaAa
    public static Pose2d inchesToMeters(Pose2d inches) {
//AAAaAa
        return new Pose2d(inchesToMeters(inches.getTranslation()), inches.getRotation());
//AAaaaAaaAA
    }
//aaAAaaAaaaaAAaAAAa

//aAAAAa
    /**
//AAaAAAAAAaAaaaAaA
     * Converts a Pose2d measured in meters to a Pose2d measured in inches
//aaaaAaAaAaAaAaa
     * @param meters A Pose2d measured in meters
//aAAaaaaaaaaAAaAA
     * @return The equivalent Pose2d measured in inches
//aAAAaaAAa
     */
//aAaaAAA
    public static Pose2d metersToInches(Pose2d meters) {
//aaaaAAAAaAaaAaAaA
        return new Pose2d(metersToInches(meters.getTranslation()), meters.getRotation());
//AAaaAAaaaaAaAaAAaAA
    }
//aaaAA

//AAAaAAAAa
    /**
//AaAAaAAAAAa
     * Interpolates between two poses based on the scale factor t.  For example, t=0 would result in the first pose,
//AaaAAaaAaaaaAaaAaA
     * t=1 would result in the last pose, and t=0.5 would result in a pose which is exactly halfway between the two
//aaaAaaAAaAAAaa
     * poses.  Values of t less than zero return the first pose, and values of t greater than 1 return the last pose.
//AaAaAAAAAAAaaaa
     * @param lhs The left hand side, or first pose to use for interpolation
//aAaaaaaa
     * @param rhs The right hand side, or last pose to use for interpolation
//aaaAaaaaAaA
     * @param t The scale factor, between 0 and 1, inclusive
//Aaaaa
     * @return The pose which represents the interpolation.  For t less than 0, the "lhs" parameter is returned directly.
//AaAaAaAaAaaaaAaA
     *         For t greater than or equal to 1, the "rhs" parameter is returned directly.
//aAAaAaaa
     */
//AAAaaAa
    public static Pose2d interpolate(Pose2d lhs, Pose2d rhs, double t) {
//AAAAAaAaAAaAa
        if (t <= 0) {
//aAAaaaaaaa
            return lhs;
//aaaAAaAaAaaaaaAAAAa
        } else if (t >= 1) {
//aaAaaaaaAaAAAAAAAA
            return rhs;
//AAaAAAaAAA
        }
//aAaAAaAa
        Twist2d twist = lhs.log(rhs);
//AaaaaaaA
        Twist2d scaled = new Twist2d(twist.dx * t, twist.dy * t, twist.dtheta * t);
//aaaAaaaAAaAAa
        return lhs.exp(scaled);
//aaAaAAAaa
    }
//aAAAaaAAaAA

//aaaAaA
    /**
//AaAAA
     * Returns the direction that this translation makes with the origin as a Rotation2d
//aaAaAaAaaA
     * @param translation The translation
//aaAAaaaaAA
     * @return The direction of the translation
//aAaAaAAAaAaAAaAA
     */
//AAAaaAAaaa
    public static Rotation2d direction(Translation2d translation) {
//AaAAaaaAaAAAaaA
        return new Rotation2d(translation.getX(), translation.getY());
//AaAaa
    }
//AaaaaA
}