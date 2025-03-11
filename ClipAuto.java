package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name="ClipAuto", preselectTeleOp = "finalDrive")
public class ClipAuto extends LinearOpMode {
    public class arm {

        private final double RIGHT_GRABBED_POS = 1;
        private final double LEFT_GRABBED_POS = 0.25;
        private final double RIGHT_RELEASE_POS = 0.7;
        private final double LEFT_RELEASE_POS = 0.4;
        private DcMotorEx armraise, armraiseB, armrotate;

        private CRServo armextend;
        private Servo grabLeft, grabRight;

        private Servo eyeLeft, eyeRight;

        private TouchSensor magSwitch;

        public arm() {
            armraise = (DcMotorEx) hardwareMap.dcMotor.get("armraiseM");
            armraiseB = (DcMotorEx) hardwareMap.dcMotor.get("armraiseB");
            armraiseB.setDirection(DcMotorSimple.Direction.REVERSE);
            armraiseB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            armrotate = (DcMotorEx) hardwareMap.dcMotor.get("armrotate");

            armextend = hardwareMap.crservo.get("armextend");
            grabLeft = hardwareMap.servo.get("grabLeft");
            grabRight = hardwareMap.servo.get("grabRight");

            eyeRight = hardwareMap.servo.get("eyeRight");
            eyeLeft = hardwareMap.servo.get("eyeLeft");

            magSwitch = hardwareMap.touchSensor.get("magSwitch");
        }
        public class RaiseArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                eyeLeft.setPosition(0.6);
                eyeRight.setPosition(0.6);

                armraise.setTargetPosition(2300);
                armraiseB.setTargetPosition(2300);
                armrotate.setTargetPosition(-4200);
                armraise.setPower(1);
                armraiseB.setPower(1);
                armrotate.setPower(1);
                armraise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armraiseB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return false;
            }
        }
        public class ArmToGrab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                eyeLeft.setPosition(0.6);
                eyeRight.setPosition(0.6);

                grabLeft.setPosition(LEFT_RELEASE_POS);
                grabRight.setPosition(RIGHT_RELEASE_POS);
                armraise.setTargetPosition(0);
                armraiseB.setTargetPosition(0);
                armrotate.setTargetPosition(-3350);
                armraise.setPower(1);
                armraiseB.setPower(1);
                armrotate.setPower(1);
                armraise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armraiseB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return false;
            }
        }

        public class GrabSpec implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                eyeLeft.setPosition(1);
                eyeRight.setPosition(0.3);

                grabLeft.setPosition(LEFT_GRABBED_POS);
                grabRight.setPosition(RIGHT_GRABBED_POS);
                return false;
            }
        }
        public class PlaceSpec implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                eyeLeft.setPosition(0.6);
                eyeRight.setPosition(0.6);

                armraise.setTargetPosition(1850);
                armraiseB.setTargetPosition(1850);
                armraise.setPower(1);
                armraiseB.setPower(1);
                armraise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armraiseB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return false;
            }
        }
        public class DropSpec implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                eyeLeft.setPosition(1);
                eyeRight.setPosition(0.3);

                grabLeft.setPosition(LEFT_RELEASE_POS);
                grabRight.setPosition(RIGHT_RELEASE_POS);
                return false;
            }
        }
        public class ArmBusy implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                while (armraise.isBusy() || armraiseB.isBusy() || armrotate.isBusy()) {
                    if (magSwitch.isPressed()) {
                        armraise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        armraiseB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    }
                }
                return false;
            }
        }
        public class RetractArm implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                armextend.setPower(-1);
                sleep(200);
                armextend.setPower(0);
                return false;
            }
        }
        public class RaiseTolerances implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                armraise.setTargetPositionTolerance(50);
                armraiseB.setTargetPositionTolerance(400);
                armrotate.setTargetPositionTolerance(100);
                armrotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armraise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armraiseB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armraiseB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                return false;
            }
        }
        public Action armToGrab() {
            return new ArmToGrab();
        }
        public Action raiseArm() {
            return new RaiseArm();
        }
        public Action grabSpec() {
            return new GrabSpec();
        }
        public Action placeSpec() { return new PlaceSpec();}
        public Action dropSpec() { return new DropSpec(); }
        public Action armBusy() {
            return new ArmBusy();
        }
        public Action retractArm() { return new RetractArm(); }
        public Action raiseTolerances() {
            return new RaiseTolerances();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {


        /*
         * TODO:
         *  Fill out preset positions
         */

        final double SPEC_WALL_X = 9.5;
        final double SPEC_WALL_Y = -18;
        final double SPEC_WALL_OFF = 3;
        final double SPEC_BAR_X = 16;
        final double SPEC_BAR_Y = 4;
        final double MIN_SPEC_DIST = 3;
        final double WAIT_TO_PLACE = 0.3;

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        arm arm = new arm();

        waitForStart();
        arm.grabSpec();
        if (isStopRequested()) return;


        Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(0, 0, 0))
                                .stopAndAdd(arm.grabSpec())
                                .stopAndAdd(arm.raiseTolerances())
                                .stopAndAdd(arm.raiseArm())
                                .strafeTo(new Vector2d(SPEC_BAR_X, SPEC_BAR_Y))
                                .stopAndAdd(arm.armBusy())
                                .lineToX(SPEC_BAR_X + 5)
                                .waitSeconds(WAIT_TO_PLACE)
                                .stopAndAdd(arm.placeSpec())
                                .lineToX(SPEC_BAR_X)
                                .stopAndAdd(arm.dropSpec())
                                .stopAndAdd(arm.armToGrab())
                                // first placed
                                .strafeTo(new Vector2d(SPEC_BAR_X - 2, -20))
                                .strafeTo(new Vector2d(47, -20))
                                .strafeTo(new Vector2d(47, -30))
                                .strafeTo(new Vector2d(12, -30))
                                // third moved to obs zone

                                .strafeTo(new Vector2d(47, -30))
                                .strafeTo(new Vector2d(47, -41))
                                .strafeTo(new Vector2d(12, -41))
                                // fourth moved to obs zone

                                .strafeToLinearHeading(new Vector2d(SPEC_WALL_X + SPEC_WALL_OFF, SPEC_WALL_Y), Math.PI)
                                .stopAndAdd(arm.retractArm())
                                .strafeTo(new Vector2d(SPEC_WALL_X, SPEC_WALL_Y))
                                .stopAndAdd(arm.grabSpec())
                                .lineToX(SPEC_WALL_X - 2)
                                .stopAndAdd(arm.raiseArm())
                                // second grabbed
                                .strafeToLinearHeading(new Vector2d(SPEC_BAR_X + 7, SPEC_BAR_Y + MIN_SPEC_DIST), 0)
                                //.stopAndAdd(arm.armBusy())
                                //.lineToX(SPEC_BAR_X + 7)
                                .waitSeconds(WAIT_TO_PLACE)
                                .stopAndAdd(arm.placeSpec())
                                .lineToX(SPEC_BAR_X)
                                .stopAndAdd(arm.dropSpec())
                                .stopAndAdd(arm.armToGrab())
                                // second placed
                                .strafeToLinearHeading(new Vector2d(SPEC_WALL_X + SPEC_WALL_OFF, SPEC_WALL_Y), Math.PI)
                                .stopAndAdd(arm.retractArm())
                                .lineToX(SPEC_WALL_X)
                                .stopAndAdd(arm.grabSpec())
                                .lineToX(SPEC_WALL_X - 2)
                                .stopAndAdd(arm.raiseArm())
                                // third grabbed
                                .strafeToLinearHeading(new Vector2d(SPEC_BAR_X + 8, SPEC_BAR_Y + MIN_SPEC_DIST * 2), 0)
                                //.stopAndAdd(arm.armBusy())
                                //.lineToX(SPEC_BAR_X + 8)
                                .waitSeconds(WAIT_TO_PLACE)
                                .stopAndAdd(arm.placeSpec())
                                .lineToX(SPEC_BAR_X)
                                .stopAndAdd(arm.dropSpec())
                                .stopAndAdd(arm.armToGrab())
                                // third placed
                                .strafeToLinearHeading(new Vector2d(2, SPEC_WALL_Y - 10), Math.PI / 2)
                        .build()
        );
    }
}
