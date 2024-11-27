package org.firstinspires.ftc.teamcode;

import java.util.HashMap;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class AprilTagLocalization {

    // Vision variables
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    // Mapping of Tag IDs to their field positions (x, y, yaw) in feet and degrees
    private final HashMap<Integer, double[]> tagPositions = new HashMap<>();

    // Robot's calculated position
    private double robotX = Double.NaN;      // in feet
    private double robotY = Double.NaN;      // in feet
    private double robotYaw = Double.NaN;    // in degrees

    // Conversion factor from inches to feet
    private static final double INCHES_TO_FEET = 1.0 / 12.0;
    // Camera offsets relative to the robot's center (in inches)
    private static final double CAMERA_OFFSET_X_INCHES = 4.0; // Positive forward
    private static final double CAMERA_OFFSET_Y_INCHES = 2.0; // Positive left
    private static final double CAMERA_HEADING_OFFSET_DEGREES = 15.0; // Positive CCW rotation

    // Constructor
    public AprilTagLocalization(HardwareMap hardwareMap) {
        // Initialize AprilTag detection
        initAprilTag(hardwareMap);

        // Initialize tag positions
        initTagPositions();
    }

    // Initialize the AprilTag processor and vision portal
    private void initAprilTag(HardwareMap hardwareMap) {
        // Create the AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        // Create the vision portal using a webcam
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();
    }

    // Initialize the known positions of the AprilTags
    private void initTagPositions() {
        // Each entry: {x, y, tagYaw}
        tagPositions.put(11, new double[]{2.0, 0.0, 180.0});
        tagPositions.put(12, new double[]{0.0, 6.0, 90.0});
        tagPositions.put(13, new double[]{2.0, 12.0, 0.0});
        tagPositions.put(14, new double[]{10.0, 12.0, 0.0});
        tagPositions.put(15, new double[]{12.0, 6.0, -90.0});
        tagPositions.put(16, new double[]{10.0, 0.0, 180.0});
    }

    // Process AprilTag detections and update robot's position
    public void updateRobotPosition() {
        // Get current detections
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        // Reset robot's position if no tags are detected
        if (detections.size() == 0) {
            robotX = Double.NaN;
            robotY = Double.NaN;
            robotYaw = Double.NaN;
            return;
        }

        // Process each detection
        for (AprilTagDetection detection : detections) {
            int tagID = detection.id;

            // Check if the detected tag is in our known list
            if (tagPositions.containsKey(tagID)) {
                double[] tagData = tagPositions.get(tagID);
                double tagX = tagData[0];
                double tagY = tagData[1];
                double tagYaw = tagData[2];

                // Get the robot's relative position to the tag (in inches)
                double robotXOffsetInches = detection.ftcPose.x; // in inches
                double robotYOffsetInches = detection.ftcPose.y; // in inches
                double robotYawOffset = detection.ftcPose.yaw;   // in degrees

                // Convert offsets from inches to feet
                double robotXOffset = robotXOffsetInches * INCHES_TO_FEET;
                double robotYOffset = robotYOffsetInches * INCHES_TO_FEET;

                // Adjust offsets based on the tag's orientation
                double tagYawRadians = Math.toRadians(tagYaw);
                double cosTheta = Math.cos(tagYawRadians);
                double sinTheta = Math.sin(tagYawRadians);

                double adjustedXOffset = robotXOffset * cosTheta - robotYOffset * sinTheta;
                double adjustedYOffset = robotXOffset * sinTheta + robotYOffset * cosTheta;

                // Calculate the camera's absolute position in the field coordinate frame
                double cameraX = tagX + adjustedXOffset;
                double cameraY = tagY + adjustedYOffset;
                double cameraYaw = tagYaw + robotYawOffset;

                // Now adjust for the camera's offset relative to the robot
                // Convert camera offsets from inches to feet
                double cameraOffsetX = CAMERA_OFFSET_X_INCHES * INCHES_TO_FEET;
                double cameraOffsetY = CAMERA_OFFSET_Y_INCHES * INCHES_TO_FEET;
                double cameraHeadingOffset = CAMERA_HEADING_OFFSET_DEGREES;

                // The robot's heading is the camera's heading minus the camera's heading offset
                robotYaw = normalizeAngle(cameraYaw - cameraHeadingOffset);

                // Convert robotYaw to radians for calculations
                double robotYawRadians = Math.toRadians(robotYaw);

                // Transform the camera offset from the robot's coordinate frame to the field coordinate frame
                double offsetXField = cameraOffsetX * Math.cos(robotYawRadians) - cameraOffsetY * Math.sin(robotYawRadians);
                double offsetYField = cameraOffsetX * Math.sin(robotYawRadians) + cameraOffsetY * Math.cos(robotYawRadians);

                // The robot's position is the camera's position minus the transformed offsets
                robotX = cameraX - offsetXField;
                robotY = cameraY - offsetYField;

                // Since we've found a valid tag, we can break out of the loop
                break;
            }
        }
    }

    private double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle > 180) {
            angle -= 360;
        } else if (angle < -180) {
            angle += 360;
        }
        return angle;
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

    // Method to close vision portal when done
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}