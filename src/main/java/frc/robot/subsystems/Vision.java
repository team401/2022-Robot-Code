package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotState;
import frc.robot.util.CircleFitter;

public class Vision extends SubsystemBase {

    private double captureTimestamp = 0.0;
    private double[] cornerX = new double[] {};
    private double[] cornerY = new double[] {};
    private boolean simpleValid = false;
    private double simpleAngle = 0.0;

    private final NetworkTableEntry ledEntry = NetworkTableInstance.getDefault()
            .getTable("limelight").getEntry("ledMode");
    private final NetworkTableEntry validEntry =
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv");
    private final NetworkTableEntry latencyEntry =
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl");
    private final NetworkTableEntry dataEntry = NetworkTableInstance.getDefault()
            .getTable("limelight").getEntry("tcornxy");
    private final NetworkTableEntry simpleAngleEntry =
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");

    // Timestamp in seconds that the last image was captured at
    private double lastCaptureTimestamp = 0.0;

    // Whether or not the LEDs should be on.
    private boolean ledsState = true;

    // Field of view constants for the limelight at 1x zoom
    private static final double vpw =
            2.0 * Math.tan(VisionConstants.fovHorizontal.getRadians() / 2.0);
    private static final double vph =
            2.0 * Math.tan(VisionConstants.fovVertical.getRadians() / 2.0);

    // Minimum number of pieces of reflective tape that must be seen to constitute a target detection.
    private static final int minTargetCount = 2;

    // Precision that must be achieved by the circle fitter, in meters.
    private static final double circleFitPrecision = 0.01;

    // Constant latency applied to the timestamps we get from NetworkTables, accounting for latency in image capture
    // and transport over the network that cannot be measured in software.
    private static final double constantLatency = 0.06;

    private static double tx = 0;

    private double distanceToTargetIn = 0.0;

    private final Timer targetTimer = new Timer();

    public Vision() {

        latencyEntry.addListener(event -> {
            double timestamp = (HALUtil.getFPGATime() / 1000000.0)
                    - (latencyEntry.getDouble(0.0) / 1000.0);

            List<Double> cornerXList = new ArrayList<>();
            List<Double> cornerYList = new ArrayList<>();
            if (validEntry.getDouble(0.0) == 1.0) {
                boolean isX = true;
                for (double coordinate : dataEntry.getDoubleArray(new double[] {})) {
                    if (isX) {
                        cornerXList.add(coordinate);
                    } else {
                        cornerYList.add(coordinate);
                    }
                    isX = !isX;
                }
            }

            synchronized (this) {
                captureTimestamp = timestamp;
                cornerX =
                        cornerXList.stream().mapToDouble(Double::doubleValue).toArray();
                cornerY =
                        cornerYList.stream().mapToDouble(Double::doubleValue).toArray();
                simpleValid = validEntry.getDouble(0.0) == 1.0;
                simpleAngle = simpleAngleEntry.getDouble(0.0);
            }

        }, EntryListenerFlags.kUpdate);

    }

    @Override
    public void periodic() {

        SmartDashboard.putBoolean("Visible Target", !targetTimer.hasElapsed(0.3));

        tx = getSimpleAngle();

        setLeds(ledsState);

        // If there is no new frame, do nothing
        if (captureTimestamp == lastCaptureTimestamp) return;
        lastCaptureTimestamp = captureTimestamp;

        // Grab target count from corners array.  If LEDs are off, force
        // no targets
        int targetCount = ledsState ? cornerX.length / 4 : 0;

        // TODO: PUT BACK IN PLACE
        // Stop if we don't have enough targets
        //if (targetCount < minTargetCount) return;

        List<Translation2d> cameraToTargetTranslations = new ArrayList<>();
        for (int targetIndex = 0; targetIndex < targetCount; targetIndex++) {
            List<VisionPoint> corners = new ArrayList<>();
            double totalX = 0.0, totalY = 0.0;
            for (int i = targetIndex * 4; i < (targetIndex * 4) + 4; i++) {
                if (i < cornerX.length && i < cornerY.length) {
                    corners.add(new VisionPoint(cornerX[i], cornerY[i]));
                    totalX += cornerX[i];
                    totalY += cornerY[i];
                }
            }

            VisionPoint targetAvg = new VisionPoint(totalX / 4, totalY / 4);
            corners = sortCorners(corners, targetAvg);

            for (int i = 0; i < corners.size(); i++) {
                Translation2d translation = solveCameraToTargetTranslation(
                        corners.get(i), i < 2 ? VisionConstants.visionTargetHeightUpper
                                : VisionConstants.visionTargetHeightLower,
                        VisionConstants.cameraHeightM, Rotation2d.fromDegrees(VisionConstants.floorToCameraAngleDeg.get()));
                if (translation != null) {
                    cameraToTargetTranslations.add(translation);
                }
            }
        }

        // Stop if there aren't enough detected corners
        if (cameraToTargetTranslations.size() < minTargetCount * 4) return;

        Translation2d cameraToTargetTranslation = CircleFitter.fit(VisionConstants.visionTargetDiameter / 2.0,
                cameraToTargetTranslations, circleFitPrecision);

        distanceToTargetIn = Units.metersToInches(cameraToTargetTranslation.getNorm());

        SmartDashboard.putBoolean("Visible Target", true);
        targetTimer.reset();
        targetTimer.start();

        // Inform RobotState of our observations
        RobotState.getInstance().recordVisionObservations(lastCaptureTimestamp - constantLatency, cameraToTargetTranslation);

    }

    public void setLeds(boolean enabled) {
        ledEntry.forceSetDouble(enabled ? 3.0 : 1.0);
    }

    public double getSimpleAngle() {
        return simpleAngle;
    }

    
    /**
     * Sorts a list of corners found in the image to the expected order for vision processing.
     * @param corners The list of all corners
     * @param average The point that is the average of all corners (geometric center)
     * @return Sorted list of corners
     */
    private List<VisionPoint> sortCorners(List<VisionPoint> corners,
                                          VisionPoint average) {

        // Find top corners
        Integer topLeftIndex = null;
        Integer topRightIndex = null;
        double minPosRads = Math.PI;
        double minNegRads = Math.PI;
        for (int i = 0; i < corners.size(); i++) {
            VisionPoint corner = corners.get(i);
            double angleRad =
                    new Rotation2d(corner.x - average.x, average.y - corner.y)
                            .minus(Rotation2d.fromDegrees(90)).getRadians();
            if (angleRad > 0) {
                if (angleRad < minPosRads) {
                    minPosRads = angleRad;
                    topLeftIndex = i;
                }
            } else {
                if (Math.abs(angleRad) < minNegRads) {
                    minNegRads = Math.abs(angleRad);
                    topRightIndex = i;
                }
            }
        }

        // Find lower corners
        Integer lowerIndex1 = null;
        Integer lowerIndex2 = null;
        for (int i = 0; i < corners.size(); i++) {
            boolean alreadySaved = false;
            if (topLeftIndex != null) {
                if (topLeftIndex.equals(i)) {
                    alreadySaved = true;
                }
            }
            if (topRightIndex != null) {
                if (topRightIndex.equals(i)) {
                    alreadySaved = true;
                }
            }
            if (!alreadySaved) {
                if (lowerIndex1 == null) {
                    lowerIndex1 = i;
                } else {
                    lowerIndex2 = i;
                }
            }
        }

        // Combine final list
        List<VisionPoint> newCorners = new ArrayList<>();
        if (topLeftIndex != null) {
            newCorners.add(corners.get(topLeftIndex));
        }
        if (topRightIndex != null) {
            newCorners.add(corners.get(topRightIndex));
        }
        if (lowerIndex1 != null) {
            newCorners.add(corners.get(lowerIndex1));
        }
        if (lowerIndex2 != null) {
            newCorners.add(corners.get(lowerIndex2));
        }
        return newCorners;
    }

    
    /**
     * Given a list of corners, solves for the translation "camera to target", meaning the vector that comes out of
     * the camera's lens and points into the center of the target.
     *
     * This is done by using the known height of the camera and point of interest, and using the difference of them
     * combined with the camera's angle, field of view, and image resolution to correct for the perspective of the camera.
     *
     * @param corner The point to process
     * @param goalHeight The known height of the point in world space
     * @param cameraHeight The known height of the camera in world space
     * @param floorToCamera The rotation between the floor and the lens, in the floor plane frame of reference.
     * @return The vector from the camera to the target.
     */
    private Translation2d solveCameraToTargetTranslation(VisionPoint corner,
                                                         double goalHeight, double cameraHeight, Rotation2d floorToCamera) {

        // Compute constants for going from pixel space to camera space
        double halfWidthPixels = VisionConstants.widthPixels / 2.0;
        double halfHeightPixels = VisionConstants.heightPixels / 2.0;
        double nY = -((corner.x - halfWidthPixels) / halfWidthPixels);
        double nZ = -((corner.y - halfHeightPixels) / halfHeightPixels);

        // Rotate by camera angle
        Translation2d xzPlaneTranslation = new Translation2d(1.0, vph / 2.0 * nZ)
                .rotateBy(floorToCamera);
        double x = xzPlaneTranslation.getX();
        double y = vpw / 2.0 * nY;
        double z = xzPlaneTranslation.getY();

        // Perspective correction
        double differentialHeight = cameraHeight - goalHeight;
        if ((z < 0.0) == (differentialHeight > 0.0)) {
            double scaling = differentialHeight / -z;
            double distance = Math.hypot(x, y) * scaling;
            Rotation2d angle = new Rotation2d(x, y);
            return new Translation2d(distance * angle.getCos(),
                    distance * angle.getSin());
        }
        return null;
    }

    /**
     * Class representing pixel coordinates for a point of interest in the camera's image.
     */
    public static class VisionPoint {
        public final double x;
        public final double y;

        public VisionPoint(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    /**
     * Turns on the limelight LEDs
     */
    public void turnOnLeds() {
        ledsState = true;
    }

    /**
     * Turns off the limelight LEDs
     */
    public void turnOffLeds() {
        ledsState = false;
    }

    public double distanceToTargetIn() {
        return distanceToTargetIn;
    }

    public static double getTX() {
        return tx;
    }
    
}
