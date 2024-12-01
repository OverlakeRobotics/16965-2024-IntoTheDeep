package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class AprilTagLocalization {

    // List of VisionPortals and AprilTagProcessors for each camera
    private List<VisionPortal> visionPortals = new ArrayList<>();
    private List<AprilTagProcessor> aprilTagProcessors = new ArrayList<>();

    // Robot's calculated position
    private double robotX = Double.NaN;      // in feet
    private double robotY = Double.NaN;      // in feet
    private double robotYaw = Double.NaN;    // in degrees

    // Camera positions and orientations on the robot
    // Assume each camera may have different positions and orientations
    private List<Position> cameraPositions = new ArrayList<>();
    private List<YawPitchRollAngles> cameraOrientations = new ArrayList<>();

    // Hardware Map
    private HardwareMap hardwareMap;

    // List of camera names as configured in the hardware map
    private List<String> cameraNames;

    // Constructor
    public AprilTagLocalization(HardwareMap hardwareMap, List<String> cameraNames,
                                List<Position> cameraPositions, List<YawPitchRollAngles> cameraOrientations) {
        this.hardwareMap = hardwareMap;
        this.cameraNames = cameraNames;
        this.cameraPositions = cameraPositions;
        this.cameraOrientations = cameraOrientations;
        // Initialize AprilTag detection for each camera
        initAprilTag();
    }

    // Initialize the AprilTag processors and vision portals for each camera
    private void initAprilTag() {
        for (int i = 0; i < cameraNames.size(); i++) {
            String cameraName = cameraNames.get(i);
            Position camPosition = cameraPositions.get(i);
            YawPitchRollAngles camOrientation = cameraOrientations.get(i);

            // Create the AprilTag processor for this camera
            AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                    .setDrawAxes(true)
                    .setDrawCubeProjection(false)
                    .setDrawTagOutline(true)
                    .setCameraPose(camPosition, camOrientation)
                    .build();

            // Create the vision portal using the camera
            VisionPortal visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, cameraName))
                    .addProcessor(aprilTagProcessor)
                    .build();

            // Add to lists
            aprilTagProcessors.add(aprilTagProcessor);
            visionPortals.add(visionPortal);
        }
    }

    // Process AprilTag detections from all cameras and update robot's position
    public void updateRobotPosition() {
        // Lists to store robot positions and orientations from each detection
        List<Double> xPositions = new ArrayList<>();
        List<Double> yPositions = new ArrayList<>();
        List<Double> yaws = new ArrayList<>();

        // Iterate over each AprilTagProcessor to collect detections
        for (AprilTagProcessor aprilTagProcessor : aprilTagProcessors) {
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

            // Process detections that have a valid robotPose
            for (AprilTagDetection detection : detections) {
                if (detection.robotPose != null) {
                    // Get robot's position from robotPose
                    double xInches = detection.robotPose.getPosition().x;
                    double yInches = detection.robotPose.getPosition().y;
                    double yawDegrees = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                    // Convert inches to feet
                    double xFeet = xInches / 12.0;
                    double yFeet = yInches / 12.0;

                    // Add to lists
                    xPositions.add(xFeet);
                    yPositions.add(yFeet);
                    yaws.add(yawDegrees);
                }
            }
        }

        // If we have collected positions, compute the averages
        if (!xPositions.isEmpty()) {
            robotX = average(xPositions);
            robotY = average(yPositions);
            robotYaw = averageAngles(yaws);
        } else {
            // No valid detections from any camera
            robotX = Double.NaN;
            robotY = Double.NaN;
            robotYaw = Double.NaN;
        }
    }

    // Helper method to compute the average of a list of doubles
    private double average(List<Double> values) {
        double sum = 0.0;
        for (Double val : values) {
            sum += val;
        }
        return sum / values.size();
    }

    // Helper method to compute the average of a list of angles in degrees
    private double averageAngles(List<Double> angles) {
        // Convert angles to radians
        double sumSin = 0.0;
        double sumCos = 0.0;
        for (Double angle : angles) {
            double radians = Math.toRadians(angle);
            sumSin += Math.sin(radians);
            sumCos += Math.cos(radians);
        }
        // Compute average angle
        double avgRadians = Math.atan2(sumSin / angles.size(), sumCos / angles.size());
        return Math.toDegrees(avgRadians);
    }

    // Getters for the robot's position
    public double getRobotX() {
        return robotX;
    }

    public double getRobotY() {
        return robotY;
    }

    public double getRobotYaw() {
        return robotYaw;
    }

    // Method to close vision portals when done
    public void close() {
        for (VisionPortal visionPortal : visionPortals) {
            if (visionPortal != null) {
                visionPortal.close();
            }
        }
    }
}
