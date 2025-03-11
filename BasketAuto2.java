package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name="BasketAuto2", preselectTeleOp = "finalDrive")
public class BasketAuto2 extends LinearOpMode {
    public class arm {

        private final double RIGHT_GRABBED_POS = 1;
        private final double LEFT_GRABBED_POS = 0.25;
        private final double RIGHT_RELEASE_POS = 0.7;
        private final double LEFT_RELEASE_POS = 0.35;
        final double EXTEND_START_POS;
        private DcMotorEx armraise, armraiseB, armrotate;
        private CRServo armextend;

        private Servo grabLeft, grabRight;

        private Servo eyeLeft, eyeRight;

        private TouchSensor magSwitch;

        private Encoder extendEnc;

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

            extendEnc = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frontleft")));
            EXTEND_START_POS = extendEnc.getPositionAndVelocity().rawPosition;
        }
        public class RaiseArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                eyeLeft.setPosition(0.6);
                eyeRight.setPosition(0.6);

                armraise.setTargetPosition(4850);
                armraiseB.setTargetPosition(4850);
                armrotate.setTargetPosition(-4600);
                armraise.setPower(1);
                armraiseB.setPower(1);
                armrotate.setPower(1);
                armraise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armraiseB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return false;
            }
        }
        public class ArmAscend implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                eyeLeft.setPosition(0.6);
                eyeRight.setPosition(0.6);

                armraise.setTargetPosition(1600);
                armraiseB.setTargetPosition(1600);
                armrotate.setTargetPosition(-500);
                armraise.setPower(1);
                armraiseB.setPower(1);
                armrotate.setPower(1);
                armraise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armraiseB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return false;
            }
        }
        public class LowerArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                eyeLeft.setPosition(0.6);
                eyeRight.setPosition(0.6);
                armraise.setTargetPosition(0);
                armraiseB.setTargetPosition(0);
                armrotate.setTargetPosition(0);
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
                sleep(200);
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

        public Action raiseArm() {
            return new RaiseArm();
        }
        public Action armAscend() { return new ArmAscend(); };
        public Action lowerArm() {
            return new LowerArm();
        }
        public Action grabSpec() {
            return new GrabSpec();
        }
        public Action dropSpec() {
            return new DropSpec();
        }
        public Action armBusy() {
            return new ArmBusy();
        }
        public Action raiseTolerances() {
            return new RaiseTolerances();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        final double BASKET_PLACE_X = 10;
        final double BASKET_PLACE_Y = -19;
        final double BASKET_DROP_X = 14;

        final Vector2d toPlace = new Vector2d(BASKET_PLACE_X, BASKET_PLACE_Y);

        final Vector2d toDrop = new Vector2d(BASKET_PLACE_X + 3, BASKET_PLACE_Y + 3);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        arm arm = new arm();

        TrajectoryActionBuilder mainTraj = drive.actionBuilder(new Pose2d(0, 0, 0))
                .stopAndAdd(arm.raiseTolerances())
                .stopAndAdd(arm.grabSpec())
                .stopAndAdd(arm.raiseArm())
                .strafeToLinearHeading(toPlace, Math.PI / 4, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 10;
                    }
                })
                .stopAndAdd(arm.armBusy())
                .strafeTo(toDrop)
                .stopAndAdd(arm.dropSpec())
                .lineToX(BASKET_PLACE_X)
                // first dropped
                .stopAndAdd(arm.lowerArm())
                .strafeToLinearHeading(new Vector2d(11, -21), -Math.PI / 2, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 1.5;
                    }
                })
                //.stopAndAdd(arm.reachArm())
                .strafeTo(new Vector2d(10, -26))
                .stopAndAdd(arm.armBusy())
                .stopAndAdd(arm.grabSpec())
                // second grabbed
                .stopAndAdd(arm.raiseArm())
                .lineToY(-23)
                .strafeToLinearHeading(toPlace, Math.PI / 4, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 3;
                    }
                })
                .stopAndAdd(arm.armBusy())
                .lineToY(-17.5)
                .strafeTo(toDrop)
                .stopAndAdd(arm.dropSpec())
                .strafeTo(toPlace)
                // second dropped
                .stopAndAdd(arm.lowerArm())
                .strafeToLinearHeading(new Vector2d(20, -23), -Math.PI / 2, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 8;
                    }
                })
                .strafeTo(new Vector2d(20, -27))
                .stopAndAdd(arm.armBusy())
                .stopAndAdd(arm.grabSpec())
                // third grabbed
                .stopAndAdd(arm.raiseArm())
                .strafeToSplineHeading(toPlace, Math.PI / 4, new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 7;
                    }
                })
                .stopAndAdd(arm.armBusy())
                .strafeTo(toDrop)
                .stopAndAdd(arm.dropSpec())
                .strafeTo(toPlace)
                // third dropped
                .stopAndAdd(arm.grabSpec())
                .stopAndAdd(arm.lowerArm())
                .strafeToLinearHeading(new Vector2d(23, -22), Math.toRadians(- 80), new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 7;
                    }
                })
                .strafeTo(new Vector2d(25.5, -30))
                // touching sample
                .strafeToLinearHeading(new Vector2d(20.5, -42), - Math.PI / 2)
                .stopAndAdd(arm.dropSpec())
                .strafeTo(new Vector2d(24, -39))
                .strafeTo(new Vector2d(24, -44))
                .stopAndAdd(arm.grabSpec())
                // fourth grabbed
                .stopAndAdd(arm.raiseArm())
                .strafeToLinearHeading(toPlace, Math.PI / 4)
                .strafeTo(toDrop)
                .stopAndAdd(arm.dropSpec())
                .strafeTo(toPlace)
                .stopAndAdd(arm.armAscend())
                .strafeToSplineHeading(new Vector2d(-3, -52), Math.PI)
                .strafeTo(new Vector2d(-15, -52))
                ;

        waitForStart();

        arm.grabSpec();
        if (isStopRequested()) return;


        Actions.runBlocking( mainTraj.build() );
    }
}
