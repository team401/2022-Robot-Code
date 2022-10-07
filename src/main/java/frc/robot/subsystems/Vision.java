//aAAaAaaAaaA
package frc.robot.subsystems;
//aaaaAAaaA

//aAaAaA
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//AaAAaA

//AAaAaAaAaA
import edu.wpi.first.networktables.EntryListenerFlags;
//aaAaaAAaAaaAAAaaaa
import edu.wpi.first.networktables.NetworkTableEntry;
//AaAaaaaaAA
import edu.wpi.first.networktables.NetworkTableInstance;
//AAaaa

//AAAaAa
import java.util.ArrayList;
//AAaAaaAaA
import java.util.List;
//AaAAaAAaaA

//AaaAAaa
import edu.wpi.first.hal.HALUtil;
//AaAAAAAaAAAaAa
import edu.wpi.first.math.geometry.Rotation2d;
//aaAAAA
import edu.wpi.first.math.geometry.Translation2d;
//AaAaAaAaaA
import edu.wpi.first.math.util.Units;
//AAAaaAaAaaaa
import frc.robot.Constants.VisionConstants;
//aaaAaAaaAaaa
import frc.robot.RobotState;
//aAaaaAaAaaaAAaAaa
import frc.robot.util.CircleFitter;
//aaAAaAAaaAaaa

//AaaaaAAAaAaAaaAaaA
public class Vision extends SubsystemBase {
//aaaAAaaAaaaaaAA

//aAAAAaaAAAAaAaAaaAa
    private double captureTimestamp = 0.0;
//AaAaAAa
    private double[] cornerX = new double[] {};
//AaAaAaAaaAAAa
    private double[] cornerY = new double[] {};
//aaAAaAaaA
    private boolean simpleValid = false;
//aAaAa
    private double simpleAngle = 0.0;
//aAaAA

//aaAAaa
    private final NetworkTableEntry ledEntry = NetworkTableInstance.getDefault()
//aaaAaa
            .getTable("limelight").getEntry("ledMode");
//aAaaaaaaaaAaA
    private final NetworkTableEntry validEntry =
//AaAAA
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv");
//AaAaaAAaAaaa
    private final NetworkTableEntry latencyEntry =
//AAAAAaa
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl");
//AAaaaAaaAa
    private final NetworkTableEntry dataEntry = NetworkTableInstance.getDefault()
//aAAAAaAAAAaa
            .getTable("limelight").getEntry("tcornxy");
//aaAAAAaaa
    private final NetworkTableEntry simpleAngleEntry =
//AaAAaa
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
//aaAAaaaaAAaAAAAaAa

//aaaaAAaAaaAaaAA
    // Timestamp in seconds that the last image was captured at
//AAaaAaaaA
    private double lastCaptureTimestamp = 0.0;
//AAaaAAAaaaaAAAaAAa

//AaAAaaaaAAa
    // Whether or not the LEDs should be on.
//AAaAAaAaaaa
    private boolean ledsState = true;
//AAAAaAa

//AAAaaaAAaaAaAaaAa
    // Field of view constants for the limelight at 1x zoom
//AAaAaaaAaaAaAaA
    private static final double vpw =
//AAAAaaaAaAaaaa
            2.0 * Math.tan(VisionConstants.fovHorizontal.getRadians() / 2.0);
//AaAAAAaAaAaAa
    private static final double vph =
//aaaAaaAAAaAA
            2.0 * Math.tan(VisionConstants.fovVertical.getRadians() / 2.0);
//aaaaa

//AaaaAaaAaAaAaAaa
    // Minimum number of pieces of reflective tape that must be seen to constitute a target detection.
//aAaAAaAAaAa
    private static final int minTargetCount = 2;
//AAaAaaAAaAaAAAAaaAa

//aaaaAAaA
    // Precision that must be achieved by the circle fitter, in meters.
//AAAAA
    private static final double circleFitPrecision = 0.01;
//AAaaAAaaaAaAaAaaaa

//AaaAaAa
    // Constant latency applied to the timestamps we get from NetworkTables, accounting for latency in image capture
//AaAaaaaaA
    // and transport over the network that cannot be measured in software.
//AAAAaaaA
    private static final double constantLatency = 0.06;
//aAAAAaaaAAAaAa

//aaaAAaAAAA
    private static double tx = 0;
//AAaaaAAAaAAAaaAAA

//aAAAaAAAAAA
    private double distanceToTargetIn = 0.0;
//AAaAAAAAAaaAaAAaa

//AaaaAaaAaaAAAA
    public Vision() {
//AAaaaAAAaa

//AaAAaAaaaAAAAAA
        latencyEntry.addListener(event -> {
//AaAaaaaAAAAAAAAaAaA
            double timestamp = (HALUtil.getFPGATime() / 1000000.0)
//AaaaaaaaAAaAAa
                    - (latencyEntry.getDouble(0.0) / 1000.0);
//AaaAaaA

//aaaaAAaaAAaAAAaAa
            List<Double> cornerXList = new ArrayList<>();
//aaaaAaAaAAaAAAAaAAa
            List<Double> cornerYList = new ArrayList<>();
//aAaAAAAaaa
            if (validEntry.getDouble(0.0) == 1.0) {
//aAaaaAAAaaAAAaAAa
                boolean isX = true;
//AAaAaaaAaaaAAAaAAA
                for (double coordinate : dataEntry.getDoubleArray(new double[] {})) {
//AaaaaAAaAaaAAAaAA
                    if (isX) {
//aAAaAA
                        cornerXList.add(coordinate);
//AaaaAaaAAAaaAaAAaa
                    } else {
//aaaaaaAaaaAaaAa
                        cornerYList.add(coordinate);
//AAaaaaaa
                    }
//aAaAAAa
                    isX = !isX;
//AAaaaaaAaaA
                }
//aAAAaaAaa
            }
//AaAaaAAA

//AaAAAaA
            synchronized (this) {
//AAaAAaAaAAaaaAaA
                captureTimestamp = timestamp;
//aaAAAaaAaaAAa
                cornerX =
//aAaaAaaaA
                        cornerXList.stream().mapToDouble(Double::doubleValue).toArray();
//aAaAAAAaaAAAAaAA
                cornerY =
//AaAAAAAAAAAAAaAAaaa
                        cornerYList.stream().mapToDouble(Double::doubleValue).toArray();
//aAAaAAAAaaa
                simpleValid = validEntry.getDouble(0.0) == 1.0;
//aaAAaAAAaAaa
                simpleAngle = simpleAngleEntry.getDouble(0.0);
//aaaaAAAaaAAa
            }
//aaaAaaAaaAa

//AAAAAaaaaaaaAAaAAaA
        }, EntryListenerFlags.kUpdate);
//AaaaAaaAaAaaaaaAaA

//AAaaaA
    }
//AAAAaAaAAaaaAAaa

//AAAaaaaAa
    @Override
//aaaAaAaaaAaaaaA
    public void periodic() {
//AaaaAaaAa

//aaaAAaaaAAAa
        long m_Start = System.currentTimeMillis();
//aAaaaaaAAaaAAAaAAaA

//AAaaA
        tx = getSimpleAngle();
//AaaAaAAAAAAAAAaaAAa

//aAAaaAAAaaAAaA
        setLeds(ledsState);
//aaaaaAaAAaaAaAAa

//AAaaAAA
        // If there is no new frame, do nothing
//aaaAAaAaaAA
        if (captureTimestamp == lastCaptureTimestamp) return;
//AAaAaaaaaAaaaaaAaaA
        lastCaptureTimestamp = captureTimestamp;
//aaAAAaaaaaaA

//AAAAAaa
        // Grab target count from corners array.  If LEDs are off, force
//AAaaAaAaaaAaaAA
        // no targets
//aAAaAAaAAAaaaAAAaaA
        int targetCount = ledsState ? cornerX.length / 4 : 0;
//aAaaAaAaAAaaaAaAAAA

//aAaAAaaAaAAaaAAa
        // TODO: PUT BACK IN PLACE
//aAaaaAAAaA
        // Stop if we don't have enough targets
//AaaaaAaaA
        //if (targetCount < minTargetCount) return;
//AAAaa

//AAaaAaAaAAAAAaaaaa
        List<Translation2d> cameraToTargetTranslations = new ArrayList<>();
//aAAAaaAa
        for (int targetIndex = 0; targetIndex < targetCount; targetIndex++) {
//aaAaaAAAaaAAAAA
            List<VisionPoint> corners = new ArrayList<>();
//AaaAAaaaaaaAAaaaa
            double totalX = 0.0, totalY = 0.0;
//aaaAAaaaaAAa
            for (int i = targetIndex * 4; i < (targetIndex * 4) + 4; i++) {
//aaAaAAaAaAA
                if (i < cornerX.length && i < cornerY.length) {
//AaaaAA
                    corners.add(new VisionPoint(cornerX[i], cornerY[i]));
//aaAaaA
                    totalX += cornerX[i];
//aaaAaA
                    totalY += cornerY[i];
//AAaAAAa
                }
//AaaaAaAAAAAa
            }
//AAaaAaAaAaAaAaa

//AaaaAaAAAAaaaAAAaa
            VisionPoint targetAvg = new VisionPoint(totalX / 4, totalY / 4);
//AaAAAaAAAaaaA
            corners = sortCorners(corners, targetAvg);
//AAAAaAaAAaaAaAa

//AAaaAaaAa
            for (int i = 0; i < corners.size(); i++) {
//aAAAAAaAA
                Translation2d translation = solveCameraToTargetTranslation(
//AaaaAaAaA
                        corners.get(i), i < 2 ? VisionConstants.visionTargetHeightUpper
//aaAaAAaAAaAaa
                                : VisionConstants.visionTargetHeightLower,
//Aaaaa
                        VisionConstants.cameraHeightM, Rotation2d.fromDegrees(VisionConstants.floorToCameraAngleDeg.get()));
//aaAaaaAaaAaAa
                if (translation != null) {
//aaaAAaAaAA
                    cameraToTargetTranslations.add(translation);
//aaAaaAaaAAa
                }
//AaAAaaAAaAAaAaaa
            }
//AAAAAaAaaaaaaAAaA
        }
//AaAAAAAAAAaaAa

//aaAAAAaaAAAaaAAaaA
        // Stop if there aren't enough detected corners
//aAAaaAaAAa
        if (cameraToTargetTranslations.size() < minTargetCount * 4) return;
//AAaaAAa

//aaaAAAaAA
        Translation2d cameraToTargetTranslation = CircleFitter.fit(VisionConstants.visionTargetDiameter / 2.0,
//AAAaAA
                cameraToTargetTranslations, circleFitPrecision);
//AAaaAAA

//aAAaaAaAaAAaaaaaaAA
        distanceToTargetIn = Units.metersToInches(cameraToTargetTranslation.getNorm());
//AaaAAAaaAaAAaA

//aAaaaAAAaAaAaAaaA
        // Inform RobotState of our observations
//aaAAAaAaAAaA
        RobotState.getInstance().recordVisionObservations(lastCaptureTimestamp - constantLatency, cameraToTargetTranslation);
//AaaAaAAaaAaAa

//AaaaAaAaaaAaAAaAaAA
    }
//AAAaAaAAAAAA

//AAAAAaaAaaaa
    public void setLeds(boolean enabled) {
//aaaAaAaaAAaA
        ledEntry.forceSetDouble(enabled ? 3.0 : 1.0);
//aaaAaAaAAaaAAA
    }
//AAaAaaAaaAAA

//AAAaaaA
    public double getSimpleAngle() {
//AAAaaAAA
        return simpleAngle;
//AAaaa
    }
//AAAaAAA

//AaaaaAAaaaaAAa
    
//aaAaAAA
    /**
//AAAAaaAAaAaAaAA
     * Sorts a list of corners found in the image to the expected order for vision processing.
//AAaaAAaaAaA
     * @param corners The list of all corners
//aAAAaAaaA
     * @param average The point that is the average of all corners (geometric center)
//aaAaaaAaaAAaaaaAAAA
     * @return Sorted list of corners
//aaaaAaAAaaAAAa
     */
//AAAaaa
    private List<VisionPoint> sortCorners(List<VisionPoint> corners,
//AAAaAa
                                          VisionPoint average) {
//AaAAAAaAaaAAAAAAAA

//AaaaAA
        // Find top corners
//aaAAa
        Integer topLeftIndex = null;
//AAaAaaAaaaAAAAaA
        Integer topRightIndex = null;
//AAaAaaAA
        double minPosRads = Math.PI;
//AaAaAAaAaAAAaaaaa
        double minNegRads = Math.PI;
//aaAAaaAa
        for (int i = 0; i < corners.size(); i++) {
//aAAAAaaAAaaaa
            VisionPoint corner = corners.get(i);
//AaAaaAaaaaa
            double angleRad =
//AaaAaaAAa
                    new Rotation2d(corner.x - average.x, average.y - corner.y)
//aaAaaaAAaaaAAaAAaa
                            .minus(Rotation2d.fromDegrees(90)).getRadians();
//aAaaaAaaaa
            if (angleRad > 0) {
//aaaAAaAAAAA
                if (angleRad < minPosRads) {
//aAaaaaAAaaaAAAaAAa
                    minPosRads = angleRad;
//AaAaAaA
                    topLeftIndex = i;
//aAaaaAAaaaAaaAAa
                }
//aAaaAaaAAaaaaaaAAaA
            } else {
//AaaaAAAAAAaAaAAAa
                if (Math.abs(angleRad) < minNegRads) {
//aaaAaA
                    minNegRads = Math.abs(angleRad);
//aAAAaa
                    topRightIndex = i;
//aaAaAaAAAaaAaAAaAa
                }
//aaaAAAAAaA
            }
//AAaaaaAaaA
        }
//AaAAAAAAaaAaaaaa

//AAAaaaaAaaaAaAaaa
        // Find lower corners
//aAaAaaaAaaAAa
        Integer lowerIndex1 = null;
//AaaAA
        Integer lowerIndex2 = null;
//AAaaaAAaaaAaaAAa
        for (int i = 0; i < corners.size(); i++) {
//AAAAAAaAaaaAAA
            boolean alreadySaved = false;
//AAaaAaaaAaAAaaAaAa
            if (topLeftIndex != null) {
//aAAaAAAaAaAaaAAaAAA
                if (topLeftIndex.equals(i)) {
//aaaaAAAaAAaaaaaA
                    alreadySaved = true;
//AAaAAAaaA
                }
//AaaAa
            }
//AaAAaAa
            if (topRightIndex != null) {
//AAAaAA
                if (topRightIndex.equals(i)) {
//aAAAAaaA
                    alreadySaved = true;
//aaAAa
                }
//AaAAaAa
            }
//aAaaAAAaaAA
            if (!alreadySaved) {
//aAaAaaA
                if (lowerIndex1 == null) {
//aAaAAaAAaAAAAA
                    lowerIndex1 = i;
//aAAAaAaAaAa
                } else {
//AAaaAaaaAAA
                    lowerIndex2 = i;
//aaAAAAAAaaAaaAAAaa
                }
//AaAAAaaaAaaAaAaa
            }
//aaAaaAaaaaAAAAAaa
        }
//aaaAaAaaAA

//AaaAAaaaAAaaaAAA
        // Combine final list
//aaAAA
        List<VisionPoint> newCorners = new ArrayList<>();
//aAAAaaaAaAaAAaaAaA
        if (topLeftIndex != null) {
//AAAaAAaaaAAAAaAAAAA
            newCorners.add(corners.get(topLeftIndex));
//AAAaAaAAAAAA
        }
//AAAaA
        if (topRightIndex != null) {
//aAaaAA
            newCorners.add(corners.get(topRightIndex));
//aAAAaAAAAAaAaa
        }
//AAAaAAAAAAAAaAaaAaA
        if (lowerIndex1 != null) {
//AaaAaAaaaaaAaAa
            newCorners.add(corners.get(lowerIndex1));
//aAAaaAaaaAAaaAaAAa
        }
//AaaaAaAaaaAAAaAAA
        if (lowerIndex2 != null) {
//AAaaAAaA
            newCorners.add(corners.get(lowerIndex2));
//aaaaAaA
        }
//AaaaAAaAAAA
        return newCorners;
//aAAaAaAaAAAaAaaAaaA
    }
//aaAaaaA

//aaAAAaAaaaa
    
//aaAAAaAAAAaaa
    /**
//AaaaaAA
     * Given a list of corners, solves for the translation "camera to target", meaning the vector that comes out of
//aaAaaaaAaAaaaaaa
     * the camera's lens and points into the center of the target.
//Aaaaaaaa
     *
//AAaaAAaAaA
     * This is done by using the known height of the camera and point of interest, and using the difference of them
//AAAAaAAAaaA
     * combined with the camera's angle, field of view, and image resolution to correct for the perspective of the camera.
//aAAaAaaaaAAAaAA
     *
//AaaaAAAaaaaaa
     * @param corner The point to process
//aaaAAAaaAaAAAAA
     * @param goalHeight The known height of the point in world space
//aAAAaaaaAAAAAAAAaa
     * @param cameraHeight The known height of the camera in world space
//aAAAA
     * @param floorToCamera The rotation between the floor and the lens, in the floor plane frame of reference.
//aAAaaAAAAaaAaAaAa
     * @return The vector from the camera to the target.
//aaaAAAaaaaAA
     */
//AaAaaaAAA
    private Translation2d solveCameraToTargetTranslation(VisionPoint corner,
//AaaAAAAaAaaAAAAaaaA
                                                         double goalHeight, double cameraHeight, Rotation2d floorToCamera) {
//AaaAaAaAaAAaaAAa

//aAAAaAAAaaA
        // Compute constants for going from pixel space to camera space
//aAAAaAAAAAaAa
        double halfWidthPixels = VisionConstants.widthPixels / 2.0;
//aAaaAaaa
        double halfHeightPixels = VisionConstants.heightPixels / 2.0;
//aaaaaAAAaaa
        double nY = -((corner.x - halfWidthPixels) / halfWidthPixels);
//Aaaaaa
        double nZ = -((corner.y - halfHeightPixels) / halfHeightPixels);
//AAaAaaAa

//AAaAAAaAAaaAaAaaaa
        // Rotate by camera angle
//AaAaaaAAaAAaaAA
        Translation2d xzPlaneTranslation = new Translation2d(1.0, vph / 2.0 * nZ)
//aaaAAAAA
                .rotateBy(floorToCamera);
//AAAaAaAaaAaaAAaa
        double x = xzPlaneTranslation.getX();
//aAAaAaaaaaAAaaAAA
        double y = vpw / 2.0 * nY;
//AaaAaaaaaaAAAaAaaA
        double z = xzPlaneTranslation.getY();
//AaAaAAAAAAaaA

//AAaAaAAaAaaa
        // Perspective correction
//AAAaA
        double differentialHeight = cameraHeight - goalHeight;
//AAAAAAaaaAAAAaAaA
        if ((z < 0.0) == (differentialHeight > 0.0)) {
//AAAAAaaA
            double scaling = differentialHeight / -z;
//aaAAaaAAAaaaAAA
            double distance = Math.hypot(x, y) * scaling;
//AaaaAAaaaAaaAAaaa
            Rotation2d angle = new Rotation2d(x, y);
//AaaAaaAAAAAAAaaAAA
            return new Translation2d(distance * angle.getCos(),
//AaaAAAaAaAAAAaA
                    distance * angle.getSin());
//AaaAAaaaaAaaaAaAAa
        }
//aAaAaAaAaAAaA
        return null;
//AaaAAAAaaaAaaaaAa
    }
//AaAAAaaAaAAA

//AAaaAaaAAaA
    /**
//AAaAAaaaAaAAaaAAAa
     * Class representing pixel coordinates for a point of interest in the camera's image.
//aAaAaaAAA
     */
//AAaAAAA
    public static class VisionPoint {
//AAAaa
        public final double x;
//aAaaaAAAAAaA
        public final double y;
//AaAaaaAAaAAAaaAa

//aAaaAaaaAaaaaa
        public VisionPoint(double x, double y) {
//aAAaAAa
            this.x = x;
//aaAaaAAAAaaAaA
            this.y = y;
//AAAAAAAa
        }
//AAAaAaa
    }
//AAaaAAAaaaaAaaaaa

//aaAAAaA
    /**
//aAaaAaaAa
     * Turns on the limelight LEDs
//aaaaAAaA
     */
//AaaAaAaAaAAaaAa
    public void turnOnLeds() {
//AaAAaAaA
        ledsState = true;
//aAAAAAAaA
    }
//AAAAAaaa

//AAaAaAaAaAAA
    /**
//AaaAAaAaaAAa
     * Turns off the limelight LEDs
//aAAAAAaAAaa
     */
//AAAAAAa
    public void turnOffLeds() {
//AAaAaaAa
        ledsState = false;
//AAAaAAaAaaaaaaAaA
    }
//aaaaAaaAAAaaAa

//aaAaAaAAAaaaaaAAAa
    public double distanceToTargetIn() {
//aaAaaAAAAAAAAAA
        return distanceToTargetIn;
//aAAaAAaaaaaA
    }
//aAAAAaaAAaAAAa

//AaAaAAaAaaaA
    public static double getTX() {
//aaAAAaAaAA
        return tx;
//AaAAA
    }
//aaaAaaAAAAAAaAAAAa
    
//aAAAAAAAAaaaAaAA
}
