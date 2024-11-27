/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.view.ViewOutlineProvider;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

@Config
@TeleOp(name="Mecanum Driver", group="TeleOp")
public class MecanumDriver extends OpMode {
    private MecanumRobotController robotController;
    private final ElapsedTime runtime = new ElapsedTime();
    private int pivotPosition = 0;
    private int viperPosition = 0;
    private int hingeDegree = 0;
    private double timeAPressed = 0;
    private double timeXPressed = 0;
    private final static double TURN_POWER = 2.3;
    private final static double FORWARD_POWER = 1.0;
    private final static double VIPER_VELOCITY_CONSTANT = 1800;
    private final static double BASE_PIVOT_VELOCITY = 240;
    private final static double MAX_PIVOT_VELOCITY = 840;
    private final static double PIVOT_RAMP_TIME = 0.5;
    private final static double STRAFE_POWER = FORWARD_POWER * 1.192;
    private final static double SPEED_MULTIPLIER = 2.7;
    private final static double INTAKE_COOLDOWN = 0.25;
    public static final double MAX_EXTENSION_BACK = 7.0;
    public static final double MAX_EXTENSION_FORWARD = 17.0;
    public static int PIVOT_PICKUP_SPECIMEN = -490;
    public static int VIPER_PICKUP_SPECIMEN = ViperSlide.MIN_POSITION;
    // About 60 degrees
    public static int OLD_PIVOT_PLACE_SPECIMEN = -1160;
    public static int OLD_VIPER_PLACE_SPECIMEN = 1200;
    public static int PIVOT_PLACE_SPECIMEN = -1172;
    public static int VIPER_PLACE_SPECIMEN = 417;
    public static int PIVOT_POSITION_OFFSET_COUNTS = 650;
    public static double MAX_HORIZONTAL_SIZE = 40;
    public final boolean isFieldCentric = false;
    private ViperSlide viperSlide;
    private Pivot pivot;
    private Intake intake;
    private double pivotStartedTime;
    private boolean pivotStarted;
    private boolean isSpecimenReady = false;
    private boolean isPickupSubReady = false;
    private boolean lastRightBumper = false;
    private boolean lastXButton = false;
    private double placeSpecimenStartTime;
    private boolean placingSpecimen = false;

    @Override
    public void init() {
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "BACKLEFT");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "BACKRIGHT");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "FRONTLEFT");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "FRONTRIGHT");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
////        claw = hardwareMap.get(Servo.class, "CLAWLEFT");
////        claw.setDirection(Servo.Direction.REVERSE);
////        wrist = hardwareMap.get(Servo.class, "CLAWRIGHT");
//
        IMU gyro = hardwareMap.get(IMU.class, "imu2");
//
        SparkFunOTOS photoSensor = hardwareMap.get(SparkFunOTOS.class, "PHOTOSENSOR");

        viperSlide = new ViperSlide(
                hardwareMap.get(DcMotorEx.class, "VIPERLEFT"),
                hardwareMap.get(DcMotorEx.class, "VIPERRIGHT"),
                false
        );
        pivot = new Pivot(
                hardwareMap.get(DcMotorEx.class, "PIVOTLEFT"),
                hardwareMap.get(DcMotorEx.class, "PIVOTRIGHT"),
                PIVOT_POSITION_OFFSET_COUNTS,
                false
        );
        intake = new Intake(hardwareMap);
        robotController = new MecanumRobotController(backLeft, backRight, frontLeft, frontRight, gyro, photoSensor);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
        intake.close();
        intake.hingeToDegree(hingeDegree);
    }

    @Override
    public void loop() {
        // Drivetrain
        if (!placingSpecimen) {
            robotController.continuousDrive(gamepad1.left_stick_y * SPEED_MULTIPLIER * FORWARD_POWER,
                    gamepad1.left_stick_x * SPEED_MULTIPLIER * STRAFE_POWER,
                    gamepad1.right_stick_x * TURN_POWER, isFieldCentric);
        }
        double extensionBeyondChassis = viperSlide.getExtensionBeyondChassis(pivot.getAngleDegrees());
        telemetry.addData("Viper Extension Beyond Chassis", extensionBeyondChassis);

        double pivotPower = Math.min(MAX_PIVOT_VELOCITY, BASE_PIVOT_VELOCITY + (MAX_PIVOT_VELOCITY - BASE_PIVOT_VELOCITY) * (runtime.seconds() - pivotStartedTime) / PIVOT_RAMP_TIME);
        double maxViperExtension = 24;
        if (hingeDegree > 15) maxViperExtension = 18;
        if (pivot.getAngleDegrees() > 70) maxViperExtension = (double) ViperSlide.MAX_POSITION / ViperSlide.MOVE_COUNTS_PER_INCH;
//        double maxViperExtension = Math.abs((MAX_HORIZONTAL_SIZE - ViperSlide.CHASSIS_TO_PIVOT_LENGTH) / Math.cos(Math.toRadians(pivot.getAngleDegrees()))) - ViperSlide.BASE_ARM_LENGTH;
        //double pivotAngleLimit = Math.toDegrees(Math.acos((MAX_HORIZONTAL_SIZE - ViperSlide.CHASSIS_TO_PIVOT_LENGTH) / (viperSlide.getCurrentPositionInches() + ViperSlide.BASE_ARM_LENGTH)));
//        telemetry.addData("Max Viper Extension", maxViperExtension);
//        telemetry.addData("Pivot Angle Limit", pivotAngleLimit);
//        if (pivot.getAngleDegrees() < 0) pivotAngleLimit = -pivotAngleLimit;
        // Pivot
        if (gamepad2.dpad_down) {
//            if (viperSlide.getCurrentPositionInches() > maxViperExtension && !Double.isNaN(pivotAngleLimit)) {
//                pivot.setAngleDegrees(pivotAngleLimit);
//            } else
            if (!pivotStarted) {
                pivotStartedTime = runtime.seconds();
                pivotStarted = true;
                pivot.move(pivotPower);
            } else {
                pivot.move(pivotPower);
            }
            isSpecimenReady = false;
        } else if (gamepad2.dpad_up) {
            if (pivot.getAngleDegrees() > 90) {
                pivot.setAngleDegrees(90);
//            } else if (viperSlide.getCurrentPositionInches() > maxViperExtension && !Double.isNaN(pivotAngleLimit)) {
//                pivot.setAngleDegrees(pivotAngleLimit);
            } else if (!pivotStarted) {
                pivotStartedTime = runtime.seconds();
                pivotStarted = true;
                pivot.move(-pivotPower);
            } else {
                pivot.move(-pivotPower);
            }
            isSpecimenReady = false;
        } else {
            pivot.hold();
            pivotStarted = false;
        }

        double triggerPower = gamepad2.left_trigger - gamepad2.right_trigger;
        // Viper
        if (triggerPower > 0 && maxViperExtension - viperSlide.getCurrentPositionInches() > 0.5) {
            if (viperSlide.getCurrentPositionInches() < maxViperExtension) {
                viperSlide.move(triggerPower * VIPER_VELOCITY_CONSTANT);
            } else {
                viperSlide.setTargetPosition((int) (maxViperExtension * ViperSlide.MOVE_COUNTS_PER_INCH));
            }
            isSpecimenReady = false;
        } else if (triggerPower < 0) {
            viperSlide.move(triggerPower * VIPER_VELOCITY_CONSTANT);
            isSpecimenReady = false;
        } else {
            viperSlide.hold();
        }

        // Open/close intake
        if (gamepad2.a && runtime.seconds() - timeAPressed >= INTAKE_COOLDOWN) {
            timeAPressed = runtime.seconds();
            if (intake.isOpen()) {
                intake.close();
            } else {
                intake.open();
            }
            isSpecimenReady = false;
        }
        int hingeDegreeChange = (int) gamepad2.left_stick_y * 8;
        if (maxViperExtension - viperSlide.getCurrentPositionInches() > 6 || hingeDegree + hingeDegreeChange <= 15 || maxViperExtension == 18) {
            hingeDegree += hingeDegreeChange;
        }

        hingeDegree = Math.min(Math.max(-20, hingeDegree), 187);

        intake.hingeToDegree(hingeDegree);

        // Pickup Specimen Macro
        if (gamepad2.left_bumper) {
//            pivot.setAngleDegrees(180);
//            viperSlide.setTargetPosition(VIPER_PICKUP_SPECIMEN);
//            intake.setWristDegree(180);
//            hingeDegree = 90;
            intake.setWristDegree(0);
            intake.largeOpen();
            pivot.setTargetPosition(207);
            hingeDegree = 157;
            viperSlide.setTargetPosition(292);
            isPickupSubReady = false;
            isSpecimenReady = false;
        }

        // Place Specimen Macro
        if (gamepad2.right_bumper != lastRightBumper && gamepad2.right_bumper) {
            if (!isSpecimenReady) {
                pivot.setTargetPosition(PIVOT_PLACE_SPECIMEN);
                viperSlide.setTargetPosition(VIPER_PLACE_SPECIMEN);
//                hingeDegree = 0;
                hingeDegree = 99;
//                intake.setWristDegree(0);
                intake.setWristDegree(-81);
                isSpecimenReady = true;
            } else {
//                viperSlide.setTargetPosition(VIPER_PLACE_SPECIMEN - 200);
                viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
                placeSpecimenStartTime = runtime.seconds();
                placingSpecimen = true;
                isSpecimenReady = false;
            }
            isPickupSubReady = false;
        }

        if (placingSpecimen && runtime.seconds() - placeSpecimenStartTime > 0.05 && runtime.seconds() - placeSpecimenStartTime < 0.25) {
            telemetry.addData("Driving", "Back");
            robotController.continuousDrive(1.25, 0, 0);
        } else if (runtime.seconds() - placeSpecimenStartTime > 0.25) {
            placingSpecimen = false;
        }

        // Reset Slides Macro
        if (gamepad2.b) {
            viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
            hingeDegree = 90;
            isPickupSubReady = false;
            isSpecimenReady = false;
        }

        // Pickup submersible
        if (gamepad2.x != lastXButton && gamepad2.x) {
            if (!isPickupSubReady) {
                viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
                pivot.setAngleDegrees(5);
                hingeDegree = 0;
                isPickupSubReady = true;
            } else {
                pivot.setAngleDegrees(-10);
                isPickupSubReady = false;
            }
            isSpecimenReady = false;
        }

        // Place Basket Macro
        if (gamepad2.y) {
            pivot.setAngleDegrees(80);
            viperSlide.setTargetPosition(ViperSlide.MAX_POSITION);
            hingeDegree = 80;
            intake.setWristDegree(0);
            isPickupSubReady = false;
            isSpecimenReady = false;
        }

        if (gamepad2.dpad_left) {
            intake.setWristDegree(intake.getWristPositionDegrees() + 10);
            isPickupSubReady = false;
            isSpecimenReady = false;
        } else if (gamepad2.dpad_right) {
            intake.setWristDegree(intake.getWristPositionDegrees() - 10);
            isPickupSubReady = false;
            isSpecimenReady = false;
        }

        lastRightBumper = gamepad2.right_bumper;
        lastXButton = gamepad2.x;

//        SparkFunOTOS.Pose2D position = photoSensor.getPosition();
//        for (int i = 0; i < 10; ++i) {
//            robotController.turnTo(180, TURN_POWER);
//            robotController.turnTo(-180, TURN_POWER);
//            robotController.turnTo(0, TURN_POWER);
//        }


//        telemetry.addData("Position", position.x + ", " + position.y);
//        telemetry.addData("Rotation", position.h);
//        telemetry.addData("", "");
        telemetry.addData("Claw Left Position", intake.getLeftClawPosition());
        telemetry.addData("Claw Right Position", intake.getRightClawPosition());
////        telemetry.addData("Claw Position", claw.getPosition());
////        telemetry.addData("Wrist Position", wrist.getPosition());
        telemetry.addData("Viper Slide Position", viperSlide.getCurrentPosition());
//        telemetry.addData("Viper Extension Beyond Chassis", extensionBeyondChassis);
        telemetry.addData("Viper Left Pos", viperSlide.getLeftPosition());
        telemetry.addData("Viper Right Pos", viperSlide.getRightPosition());
        telemetry.addData("Pivot Position", pivot.getCurrentPosition());
        telemetry.addData("Pivot Angle", pivot.getAngleDegrees());
        telemetry.addData("Hinge Degree", intake.getHingePositionDegrees());
        telemetry.addData("Wrist Degree", intake.getWristPositionDegrees());
//        telemetry.addData("Whacker Position", intake.getWhackerPosition());
        telemetry.addData("", "");
        robotController.sendTelemetry(telemetry);
    }
}
