package org.firstinspires.ftc.teamcode;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.module.ArtifactLocation;
import org.firstinspires.ftc.teamcode.module.FieldCentricDriveTrain;
import org.firstinspires.ftc.teamcode.module.Intake;
import org.firstinspires.ftc.teamcode.module.RobotLift;
import org.firstinspires.ftc.teamcode.module.Shooter;
import org.firstinspires.ftc.teamcode.module.Spindexer;
import org.firstinspires.ftc.teamcode.module.Turret;

import java.util.Arrays;

@Config
public class Robot {
    private static final String TAG = "Robot";

    public static double ROBOT_LENGTH = 17.5;
    public static double ROBOT_WIDTH = 17.25;
    public static double FIELD_LENGTH = 144.0;
    public static double FIELD_WIDTH = 144.0;

    public static double DEFAULT_ROBOT_X = 117.9;
    public static double DEFAULT_ROBOT_Y = 131.3;
    public static DistanceUnit DEFAULT_ROBOT_POSITION_UNIT = DistanceUnit.INCH;
    public static double DEFAULT_ROBOT_HEADING = -54;
    public static AngleUnit DEFAULT_ROBOT_HEADING_UNIT = AngleUnit.DEGREES;

    private static Pose2D getDefaultRedPose() {
        return new Pose2D(
            DEFAULT_ROBOT_POSITION_UNIT, DEFAULT_ROBOT_X, DEFAULT_ROBOT_Y,
            DEFAULT_ROBOT_HEADING_UNIT, DEFAULT_ROBOT_HEADING
        );
    }

    public Pose2D getDefaultRobotPose() {
        final Pose2D redDefault2D = getDefaultRedPose();
        if (allianceColor == AllianceColor.BLUE) {
            final Pose defaultPoseRed = PoseConverter.pose2DToPose(
                redDefault2D,
                PedroCoordinates.INSTANCE
            );
            return PoseConverter.poseToPose2D(defaultPoseRed.mirror(), PedroCoordinates.INSTANCE);
        }
        else {
            return redDefault2D;
        }
    }

    private static final class PersistentState {
        private static PersistentState saved = null;

        private final Pose2D robotPose;
        private final boolean[] artifactDetections;
        private final double turretHeading;

        private PersistentState(
            Pose2D robotPose,
            boolean[] artifactDetections,
            double turretHeading
        ) {
            this.robotPose = robotPose;
            this.artifactDetections = artifactDetections;
            this.turretHeading = turretHeading;
        }

        public static void saveRobotState(Robot robot) {
            if (robot == null) {
                return;
            }

            saved = new PersistentState(
                robot.driveTrain.getRobotPose(),
                robot.spindexer.getArtifactDetections(),
                robot.turret.getCurrentHeading(AngleUnit.DEGREES)
            );

            Log.i(TAG, "Saved robot state: " + saved);
        }

        public static boolean tryLoadRobotState(Robot robot) {
            if (saved == null || robot == null) {
                return false;
            }

            Log.i(TAG, "Loading robot state: " + saved);

            robot.driveTrain.setRobotPose(saved.robotPose);
            robot.spindexer.setArtifactDetections(saved.artifactDetections);
            robot.turret.setCurrentHeading(saved.turretHeading, AngleUnit.DEGREES);

            saved = null;
            return true;
        }

        public static void clearPersistentState() {
            Log.i(TAG, "Cleared robot state");
            saved = null;
        }

        @NonNull
        @Override
        public String toString() {
            return "PersistentState{ pose = " +
                robotPose +
                ", turret heading = " +
                turretHeading +
                ", detections = " +
                Arrays.toString(artifactDetections) +
                " }";
        }
    }

    public static void clearPersistentState() {
        PersistentState.clearPersistentState();
    }

    public void savePersistentState() {
        PersistentState.saveRobotState(this);
    }

    public void tryLoadPersistentState() {
        if (!PersistentState.tryLoadRobotState(this)) {
            loadDefaultState();
        }
    }

    public void loadDefaultState() {
        driveTrain.setRobotPose(getDefaultRobotPose());
    }

    public enum RobotState {
        INTAKE,             // Robot is collecting artifacts on the field
        REVERSE_INTAKE,     // Robot is ejecting artifacts
        PRE_SHOOT,          // Robot is preparing to shoot, but cannot actually launch artifacts
        MANUAL_PRE_SHOOT,   // Robot is preparing to shoot at a hardcoded speed (used for tuning)
        SHOOT,              // Robot is shooting artifacts into the goal
        MANUAL_SHOOT,       // Robot is shooting using hardcoded speeds (used for tuning)
        NONE,               // No special function; robot is just moving
        PARK,               // Robot has parked (match is about to end)
    }

    private double fallbackRPM = 2900;
    private double fallbackHoodPosition = 0.5;

    private double moveScale = 1;
    private double headingScale = 1;
    private double turretAimOffset = 0;

    private boolean shotReady = false;
    private int shotsTaken = 0;

    /* Modules */
    private final FieldCentricDriveTrain driveTrain;
    private final Shooter shooter;
    private final Intake intake;
    private final Spindexer spindexer;
    private final Turret turret;
    private final RobotLift lift;

    private final Telemetry telemetry;

    private AllianceColor allianceColor;
    private RobotState currentState;

    private final ElapsedTime stateStopwatch;
    private final ElapsedTime timeSinceShotReady;
    private final ElapsedTime shotPrepTime;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor color) {
        this(hardwareMap, telemetry, color, false);
    }

    public Robot(
        HardwareMap hardwareMap,
        Telemetry telemetry,
        AllianceColor color,
        boolean preloadedArtifacts
    ) {
        driveTrain = new FieldCentricDriveTrain(hardwareMap, telemetry);
        driveTrain.resetOdometry();

        shooter = new Shooter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        spindexer = new Spindexer(hardwareMap, telemetry, preloadedArtifacts);
        turret = new Turret(hardwareMap, telemetry);
        lift = new RobotLift(hardwareMap, telemetry);

        this.telemetry = telemetry;

        setAllianceColor(color);

        stateStopwatch = new ElapsedTime();
        timeSinceShotReady = new ElapsedTime();
        shotPrepTime = new ElapsedTime();

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        setState(RobotState.NONE);

        Log.i(TAG, "Robot initialized");
    }

    public void setAllianceColor(AllianceColor color) {
        allianceColor = color;
    }

    public void swapAllianceColor() {
        switch (allianceColor) {
            case RED:
                setAllianceColor(AllianceColor.BLUE);
                break;

            case BLUE:
                setAllianceColor(AllianceColor.RED);
                break;
        }
    }

    private void drawRobot() {
        final FtcDashboard dashboard = FtcDashboard.getInstance();
        if (!dashboard.isEnabled()) {
            return;
        }
        final Pose2D robotPose = getRobotPose();
        final Pose2D turretPose = turret.getTurretPose(robotPose);

        final double robotHeading = robotPose.getHeading(AngleUnit.RADIANS);
        final double robotX = robotPose.getX(DistanceUnit.INCH) - (FIELD_WIDTH / 2);
        final double robotY = robotPose.getY(DistanceUnit.INCH) - (FIELD_LENGTH / 2);
        final double turretX = turretPose.getX(DistanceUnit.INCH) - (FIELD_WIDTH / 2);
        final double turretY = turretPose.getY(DistanceUnit.INCH) - (FIELD_LENGTH / 2);

        final double fieldX = FieldCentricDriveTrain.rotX(robotX, robotY, Math.PI / 2);
        final double fieldY = FieldCentricDriveTrain.rotY(robotX, robotY, Math.PI / 2);
        final double fieldHeading = robotHeading + Math.PI / 2;

        final double turretHeading = turret.getCurrentHeading(AngleUnit.RADIANS) + fieldHeading;
        final double turretLength = Math.sqrt(
            ROBOT_LENGTH * ROBOT_LENGTH + ROBOT_WIDTH * ROBOT_WIDTH) / 2;
        final double turretStartX = FieldCentricDriveTrain.rotX(turretX, turretY, Math.PI / 2);
        final double turretStartY = FieldCentricDriveTrain.rotY(turretX, turretY, Math.PI / 2);
        final double turretEndX = turretStartX + turretLength * Math.cos(turretHeading);
        final double turretEndY = turretStartY + turretLength * Math.sin(turretHeading);

        final TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay()
            .setFill("red")
            .fillPolygon(
                new double[]{
                    fieldX + FieldCentricDriveTrain.rotX(
                        ROBOT_WIDTH / 2, ROBOT_LENGTH / 2, fieldHeading),
                    fieldX + FieldCentricDriveTrain.rotX(
                        ROBOT_WIDTH / 2, -ROBOT_LENGTH / 2, fieldHeading),
                    fieldX + FieldCentricDriveTrain.rotX(
                        -ROBOT_WIDTH / 2, -ROBOT_LENGTH / 2, fieldHeading),
                    fieldX + FieldCentricDriveTrain.rotX(
                        -ROBOT_WIDTH / 2, ROBOT_LENGTH / 2, fieldHeading)
                },
                new double[]{
                    fieldY + FieldCentricDriveTrain.rotY(
                        ROBOT_WIDTH / 2, ROBOT_LENGTH / 2, fieldHeading),
                    fieldY + FieldCentricDriveTrain.rotY(
                        ROBOT_WIDTH / 2, -ROBOT_LENGTH / 2, fieldHeading),
                    fieldY + FieldCentricDriveTrain.rotY(
                        -ROBOT_WIDTH / 2, -ROBOT_LENGTH / 2, fieldHeading),
                    fieldY + FieldCentricDriveTrain.rotY(
                        -ROBOT_WIDTH / 2, ROBOT_LENGTH / 2, fieldHeading)
                }
            )
            .setStroke("black")
            .strokeLine(
                fieldX + FieldCentricDriveTrain.rotX(0, 0, fieldHeading),
                fieldY + FieldCentricDriveTrain.rotY(0, 0, fieldHeading),
                fieldX + FieldCentricDriveTrain.rotX(ROBOT_WIDTH / 2, 0, fieldHeading),
                fieldY + FieldCentricDriveTrain.rotY(ROBOT_WIDTH / 2, 0, fieldHeading)
            )
            .strokeLine(
                fieldX + FieldCentricDriveTrain.rotX(
                    ROBOT_WIDTH / 2, ROBOT_LENGTH / 2, fieldHeading),
                fieldY + FieldCentricDriveTrain.rotY(
                    ROBOT_WIDTH / 2, ROBOT_LENGTH / 2, fieldHeading),
                fieldX + FieldCentricDriveTrain.rotX(
                    ROBOT_WIDTH / 2, -ROBOT_LENGTH / 2, fieldHeading),
                fieldY + FieldCentricDriveTrain.rotY(
                    ROBOT_WIDTH / 2, -ROBOT_LENGTH / 2, fieldHeading)
            )
            .strokeLine(turretStartX, turretStartY, turretEndX, turretEndY);
        dashboard.sendTelemetryPacket(packet);
    }

    public void logInfo() {
        telemetry.addData("Fallback Shooter RPM", fallbackRPM);
        telemetry.addData("Fallback Hood Position", fallbackHoodPosition);

        driveTrain.logInfo();
        shooter.logInfo();
        turret.logInfo();
        spindexer.logInfo();
        lift.logInfo();

        drawRobot();
    }

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }

    public int getShotsTaken() {
        if (currentState != RobotState.SHOOT) {
            return 0;
        }
        return shotsTaken;
    }

    private void prepareToShoot(double goalOffsetX, double goalOffsetY) {
        if (currentState == RobotState.MANUAL_SHOOT || currentState == RobotState.MANUAL_PRE_SHOOT) {
            shooter.setRPM(fallbackRPM);
            shooter.setHoodPosition(fallbackHoodPosition);
        }
        else {
            shooter.setRPMForGoal(Math.sqrt(goalOffsetX * goalOffsetX + goalOffsetY * goalOffsetY));
            shooter.adjustHood();
        }
    }


    public void setState(RobotState newState) {
        if (newState == currentState) {
            return;
        }

        Log.v(
            TAG, "State " + currentState + " lasted for " + stateStopwatch.seconds() + " seconds");
        Log.i(TAG, "Switched to new state: " + newState);
        stateStopwatch.reset();

        switch (newState) {
            case INTAKE:
                spindexer.intakeIntoEmptySlot();
                shooter.disengageKicker();
                intake.startIntake();
                lift.lowerRobot();
                shooter.setRPM(0);
                break;

            case REVERSE_INTAKE:
                spindexer.beginIntaking();
                shooter.disengageKicker();
                intake.reverseIntake();
                lift.lowerRobot();
                shooter.setRPM(0);
                break;

            case MANUAL_PRE_SHOOT:
            case PRE_SHOOT:
                intake.stopIntake();
                shooter.engageKicker();
                lift.lowerRobot();
                break;

            case MANUAL_SHOOT:
            case SHOOT:
                lift.lowerRobot();
                intake.stopIntake();
                shooter.engageKicker();

                shotReady = false;
                shotPrepTime.reset();
                shotsTaken = 0;

                Log.d(
                    TAG, "enter shoot {" +
                        (spindexer.hasArtifact(ArtifactLocation.SLOT_ONE) ? "1 | " : "  | ") +
                        (spindexer.hasArtifact(ArtifactLocation.SLOT_TWO) ? "2 | " : "  | ") +
                        (spindexer.hasArtifact(ArtifactLocation.SLOT_THREE) ? "3" : " ") +
                        "}"
                );

                spindexer.setArtifactDetections(new boolean[ArtifactLocation.values().length]);
                break;

            case NONE:
                spindexer.beginIntaking();
                shooter.disengageKicker();
                shooter.setRPM(0);
                intake.stopIntake();
                lift.lowerRobot();
                break;

            case PARK:
                spindexer.setPower(0);
                shooter.disengageKicker();
                shooter.setRPM(0);
                intake.stopIntake();
                lift.raiseRobot();
                break;
        }
        currentState = newState;
    }

    public RobotState getState() {
        return currentState;
    }

    public void loop(Gamepad gamepad1) {
        loop(
            -gamepad1.left_stick_y * moveScale, gamepad1.left_stick_x * moveScale,
            gamepad1.right_stick_x * headingScale
        );
    }

    public void loop(double drivePower, double strafePower, double turnPower) {
        loopWithoutMovement();
        if (currentState != RobotState.PARK) {
            setDrivePowers(drivePower, strafePower, turnPower);
        }
    }

    public void loopWithoutMovement() {
        try {
            driveTrain.updatePinpoint();
            if (currentState == RobotState.PARK) {
                return;
            }

            final Pose2D robotPose = driveTrain.getRobotPose();
            final Pose2D turretPose = turret.getTurretPose(robotPose);
            final double goalOffsetX = allianceColor.goalPositionX - turretPose.getX(
                DistanceUnit.INCH);
            final double goalOffsetY = allianceColor.goalPositionY - turretPose.getY(
                DistanceUnit.INCH);
            turret.aimAtGoal(
                goalOffsetX,
                goalOffsetY,
                robotPose.getHeading(AngleUnit.RADIANS) + turretAimOffset,
                AngleUnit.RADIANS
            );

            switch (currentState) {
                case MANUAL_SHOOT:
                case SHOOT:
                    prepareToShoot(goalOffsetX, goalOffsetY);

                    if (!isShotReady()) {
                        spindexer.setPower(0);
                        if (shotReady) {
                            shotsTaken++;
                            Log.d(
                                TAG,
                                "Shot #" + shotsTaken + " completed in " + timeSinceShotReady.milliseconds() + " millis"
                            );

                        }
                        shotReady = false;
                        break;
                    }

                    spindexer.setPower(-1);
                    if (!shotReady) {
                        Log.d(
                            TAG, "Ready to shoot after " + shotPrepTime.milliseconds() + " millis");
                        shotPrepTime.reset();
                        shotReady = true;
                        timeSinceShotReady.reset();
                    }
                    break;

                case MANUAL_PRE_SHOOT:
                case PRE_SHOOT:
                    prepareToShoot(goalOffsetX, goalOffsetY);
                    holdUpBall();
                    break;

                case INTAKE:
                    spindexer.intakeIntoEmptySlot();
                    if (spindexer.hasAllArtifacts()) {
                        intake.idleWithBall();
                    }
                    else {
                        intake.startIntake();
                    }
                    break;

                case REVERSE_INTAKE:
                case PARK:
                    break;

                case NONE:
                    holdUpBall();
                    break;
            }

            spindexer.updateSpindexer();
            turret.update();

            logInfo();
        } catch (Throwable e) {
            Log.e(TAG, "An exception was encountered; saving state", e);
            // save state on a crash to (hopefully) recover on next run
            savePersistentState();

            // actually crash
            throw e;
        }
    }

    private void holdUpBall() {
        intake.idleWithBall();
    }

    public void setTurretAimOffset(double offset) {
        turretAimOffset = offset;
    }

    public void resetRobotPosition() {
        driveTrain.setRobotPose(getDefaultRobotPose());
    }

    public boolean isShotReady() {
        return shooter.isReady() &&
            turret.isReady();
    }

    /* Module-specific methods */

    public Pose2D getRobotPose() {
        return driveTrain.getRobotPose();
    }

    public void setRobotPose(Pose2D pose) {
        driveTrain.setRobotPose(pose);
    }

    public void setDrivePowers(double drive, double strafe, double turn) {
        driveTrain.setPower(drive, strafe, turn);
    }

    public void stopRobotMovement() {
        driveTrain.stopRobotMovement();
    }

    public void resetFieldCentricHeading() {
        driveTrain.resetFieldCentricHeading();
    }

    public void increaseFallbackShooterRPM() {
        fallbackRPM += 50;
    }

    public void decreaseFallbackShooterRPM() {
        fallbackRPM -= 50;
    }

    public double getFallbackShooterRPM() {
        return fallbackRPM;
    }

    public void setFallbackShooterRPM(double rpm) {
        fallbackRPM = rpm;
    }

    public double getFallbackHoodPosition() {
        return fallbackHoodPosition;
    }

    public void setFallbackHoodPosition(double position) {
        fallbackHoodPosition = position;
    }

    public void setMoveScale(double moveScale) {
        this.moveScale = moveScale;
    }

    public void setHeadingScale(double newHeadingScale) {
        headingScale = newHeadingScale;
    }

    public int getHeldArtifactCount() {
        return spindexer.getArtifactCount();
    }

    public void rotateSpindexerToNextSlot() {
        spindexer.rotateToNextSlot();
    }

    public void rotateSpindexerToPreviousSlot() {
        spindexer.rotateToPreviousSlot();
    }
}