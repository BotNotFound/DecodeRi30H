package org.firstinspires.ftc.teamcode.module;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.IndicatorColorValues;
import org.firstinspires.ftc.teamcode.controller.PositionalPIDFController;

import java.util.Arrays;
import java.util.Objects;


@Config
public class Spindexer {
    private static final String TAG = "Spindexer";

    private enum SpindexerState {
        MANUAL_ROTATION,
        INTAKING,
        LOADING,
    }

    private static final String SPINDEXER_SERVO_ONE = "Spindexer 1";
    private static final String SPINDEXER_SERVO_TWO = "Spindexer 2";
    private static final String SPINDEXER_SERVO_THREE = "Spindexer 3";
    private static final String SPINDEXER_SERVO_FOUR = "Spindexer 4";
    private static final String SENS_ORANGE_ENCODER = "Spindexer Encoder";
    private static final String FRONT_COLOR_SENSOR = "Front Color Sensor";
    public static final String INDICATOR_LIGHT_NAME = "LED Light";

    public static double LOADING_ANGLE_OFFSET = 180.0;

    private final CRServo spindexerServoOne;
    private final CRServo spindexerServoTwo;
    private final CRServo spindexerServoThree;
    private final CRServo spindexerServoFour;
    private final RevColorSensorV3 frontColorSensor;
    private final Servo indicatorLight;

    private final AnalogInput spindexerEncoder;

    private final Telemetry telemetry;

    private boolean[] artifactDetections;
    private ArtifactLocation activeLocation;
    private SpindexerState curState;

    public static double ARTIFACT_DISTANCE_THRESHOLD_CM = 5;
    public static double OFFSET_ANGLE = -38;
    public static double[] INDICATOR_COLORS = {
        IndicatorColorValues.OFF,
        IndicatorColorValues.VIOLET,
        IndicatorColorValues.YELLOW,
        IndicatorColorValues.GREEN,
    };

    private final PositionalPIDFController spindexerController;
    public static double kP = 0.0013;
    public static double kI = 0;
    public static double kD = 0.0002;
    public static double kF = 0.0445;
    public static double tolerance = 8;
    public static double feedForwardThreshold = 1;

    private boolean couldBeJammed;
    private double jamAngle;
    private final ElapsedTime timeInJam;
    public static double JAM_MOVEMENT_POWER_THRESHOLD = 0.8;
    public static double JAM_ANGLE_THRESHOLD = 2;
    public static double JAM_TIME_THRESHOLD = 1;
    public static double JAM_EXIT_POWER = 1;

    private double targetAngle = 0;

    public Spindexer(HardwareMap hardwareMap, Telemetry telemetry, boolean preloaded) {
        this.telemetry = telemetry;
        artifactDetections = new boolean[ArtifactLocation.values().length];
        if (preloaded) {
            Arrays.fill(artifactDetections, true);
        }
        activeLocation = null;
        setCurState(SpindexerState.MANUAL_ROTATION);

        spindexerServoOne = hardwareMap.get(CRServo.class, SPINDEXER_SERVO_ONE);
        spindexerServoTwo = hardwareMap.get(CRServo.class, SPINDEXER_SERVO_TWO);
        spindexerServoThree = hardwareMap.get(CRServo.class, SPINDEXER_SERVO_THREE);
        spindexerServoFour = hardwareMap.get(CRServo.class, SPINDEXER_SERVO_FOUR);

        frontColorSensor = hardwareMap.get(RevColorSensorV3.class, FRONT_COLOR_SENSOR);

        spindexerEncoder = hardwareMap.get(AnalogInput.class, SENS_ORANGE_ENCODER);

        indicatorLight = hardwareMap.get(Servo.class, INDICATOR_LIGHT_NAME);

        spindexerController = new PositionalPIDFController(kP, kI, kD, kF);
        spindexerController.setTolerance(tolerance);
        spindexerController.setFeedforwardThreshold(feedForwardThreshold);

        timeInJam = new ElapsedTime();
        jamAngle = 0;
        couldBeJammed = false;

        // set all servo directions for spindexer to turn counterclockwise
        spindexerServoOne.setDirection(CRServo.Direction.FORWARD);
        spindexerServoTwo.setDirection(CRServo.Direction.REVERSE);
        spindexerServoThree.setDirection(CRServo.Direction.REVERSE);
        spindexerServoFour.setDirection(CRServo.Direction.FORWARD);
    }

    public void setArtifactDetections(boolean[] detections) {
        Objects.requireNonNull(detections, "detections cannot be null!");
        if (detections.length != ArtifactLocation.values().length) {
            throw new IllegalArgumentException(
                "detections must be of length " + ArtifactLocation.values().length);
        }
        artifactDetections = detections;
    }

    public boolean[] getArtifactDetections() {
        return artifactDetections;
    }

    private void updateDetectionFromSensor() {
        if (!spindexerController.atSetPoint() || activeLocation == null || curState != SpindexerState.INTAKING) {
            return; // not at an artifact location; color sensor won't give the right data
        }

        double dist = frontColorSensor.getDistance(DistanceUnit.CM);
        boolean detected = dist <= ARTIFACT_DISTANCE_THRESHOLD_CM;
        if (detected != artifactDetections[activeLocation.ordinal()]) {
            Log.d(TAG, "Detection update: " + activeLocation + " = " + detected);
        }
        artifactDetections[activeLocation.ordinal()] = detected;
    }

    public ArtifactLocation getActiveLocation() {
        return activeLocation;
    }

    private void setCurState(SpindexerState newState) {
        if (newState != curState) {
            Log.v(TAG, "Set new state: " + newState);
        }
        curState = newState;
    }

    public boolean hasArtifact(ArtifactLocation location) {
        return artifactDetections[location.ordinal()];
    }

    public boolean hasAllArtifacts() {
        for (boolean ballDetection : artifactDetections) {
            if (!ballDetection) {
                return false;
            }
        }
        return true;
    }

    public boolean hasNoArtifacts() {
        for (boolean detection : artifactDetections) {
            if (detection) {
                return false;
            }
        }
        return true;
    }

    public int getArtifactCount() {
        int artifacts = 0;
        for (boolean detection : artifactDetections) {
            if (detection) {
                artifacts++;
            }
        }
        return artifacts;
    }

    private void setPowerInternal(double power) {
        spindexerServoOne.setPower(power);
        spindexerServoTwo.setPower(power);
        spindexerServoThree.setPower(power);
        spindexerServoFour.setPower(power);
    }

    public void setPower(double power) {
        setCurState(SpindexerState.MANUAL_ROTATION);
        activeLocation = null;
        setPowerInternal(power);
    }

    public double getPower() {
        return spindexerServoOne.getPower();
    }

    /**
     * Given 3 values, returns the one with the lowest magnitude.
     *
     * @return The value with the lowest magnitude
     */
    private static double signedMin(double a, double b, double c) {
        if (Math.abs(a) <= Math.abs(b) && Math.abs(a) <= Math.abs(c)) {
            return a;
        }
        if (Math.abs(b) <= Math.abs(a) && Math.abs(b) <= Math.abs(c)) {
            return b;
        }
        return c;
    }

    /**
     * Calculates the shortest displacement of {@code angle1} relative to {@code angle2}. This is
     * similar to a simple {@code angle1 - angle2}, with the exception that the calculated
     * displacement will not be unnecessarily large when the angles are across the normalization
     * boundary (e.g. {@code angle1 = 170} and {@code angle2 = -175})
     *
     * @param angle1 The angle to get the displacement of
     * @param angle2 The 'source' angle -- displacement is calculated relative to this value
     * @param unit   The unit of the given angles
     * @return The shortest displacement of {@code angle1} relative to {@code angle2}
     */
    private static double getShortestDisplacement(double angle1, double angle2, AngleUnit unit) {
        final double ROTATION;
        switch (unit) {
            case DEGREES:
                ROTATION = 360;
                break;

            case RADIANS:
            default:
                ROTATION = 2.0 * Math.PI;
                break;
        }

        angle1 = unit.normalize(angle1);
        angle2 = unit.normalize(angle2);

        return signedMin(
            angle1 - angle2,
            angle1 + ROTATION - angle2,
            angle1 - angle2 - ROTATION
        );
    }

    /**
     * Calculates the closest angle to {@code closeToAngle} that is equivalent to {@code angle}
     *
     * @param angle        The angle to transform
     * @param closeToAngle The target angle -- the calculated angle will be as close as possible to
     *                     this
     * @param unit         The unit both angles are in
     * @return The closest angle to {@code closeToAngle} that is equivalent to {@code angle}
     */
    private static double closestEquivalentAngle(
        double angle,
        double closeToAngle,
        AngleUnit unit
    ) {
        return closeToAngle + getShortestDisplacement(angle, closeToAngle, unit);
    }

    private double getTargetAngle() {
        return targetAngle;
    }

    private void setTargetAngle(double angle) {
        targetAngle = angle;
    }

    public void rotateToAngle(double angle) {
        activeLocation = null;
        setCurState(SpindexerState.MANUAL_ROTATION);
        setTargetAngle(angle);
    }

    private void checkForJam() {
        if (getPower() < JAM_MOVEMENT_POWER_THRESHOLD) {
            couldBeJammed = false;
            return;
        }

        if (!couldBeJammed) {
            couldBeJammed = true;
            jamAngle = getAngle();
            timeInJam.reset();
        }

        final double curAngle = getAngle();
        if (Math.abs(curAngle - jamAngle) > JAM_ANGLE_THRESHOLD) {
            couldBeJammed = false;
        }
    }

    private boolean isJammed() {
        return couldBeJammed && timeInJam.seconds() >= JAM_TIME_THRESHOLD;
    }

    private void handleJams() {
        checkForJam();
        if (JAM_EXIT_POWER != 0 && isJammed()) {
            setPowerInternal(JAM_EXIT_POWER * -Math.signum(getPower()));
            couldBeJammed = false;
        }
    }

    public void updateSpindexer() {
        if (curState != SpindexerState.MANUAL_ROTATION) {
            final double curError = getShortestDisplacement(
                getAngle(), getTargetAngle(), AngleUnit.DEGREES);

            spindexerController.setPIDF(kP, kI, kD, kF);
            spindexerController.setTolerance(tolerance);
            spindexerController.setFeedforwardThreshold(feedForwardThreshold);

            setPowerInternal(spindexerController.calculate(curError));
        }

        handleJams();

        updateDetectionFromSensor();
        indicatorLight.setPosition(
            INDICATOR_COLORS[Math.min(getArtifactCount(), INDICATOR_COLORS.length)]);
    }

    private void setActiveLocation(ArtifactLocation location) {
        if (location == null) {
            location = ArtifactLocation.SLOT_ONE;
        }
        if (location != activeLocation) {
            Log.v(TAG, "Moving to " + location);
        }

        activeLocation = location;
        switch (curState) {
            case MANUAL_ROTATION:
                setCurState(SpindexerState.INTAKING);
                // no break because we should mimic the behavior of the state we changed to
            case INTAKING:
                setTargetAngle(location.angle);
                break;
            case LOADING:
                setTargetAngle(location.angle + LOADING_ANGLE_OFFSET);
                break;
        }
    }

    public void intakeIntoLocation(ArtifactLocation location) {
        setCurState(SpindexerState.INTAKING);
        setActiveLocation(location);
    }

    public void loadFromLocation(ArtifactLocation location) {
        setCurState(SpindexerState.LOADING);
        setActiveLocation(location);
    }

    public boolean atTargetRotation() {
        return spindexerController.atSetPoint();
    }

    public void rotateToNextSlot() {
        if (activeLocation == null) {
            setActiveLocation(ArtifactLocation.SLOT_ONE);
        }
        else {
            setActiveLocation(activeLocation.getNextLocation());
        }
    }

    public void rotateToPreviousSlot() {
        if (activeLocation == null) {
            setActiveLocation(ArtifactLocation.SLOT_ONE);
        }
        else {
            setActiveLocation(activeLocation.getPreviousLocation());
        }
    }

    public void intakeIntoEmptySlot() {
        if (hasAllArtifacts()) {
            return;
        }

        ArtifactLocation location = activeLocation;
        if (location == null) {
            location = ArtifactLocation.SLOT_ONE;
        }

        while (hasArtifact(location)) {
            location = location.getNextLocation();
        }

        intakeIntoLocation(location);
    }

    public void loadNextArtifact() {
        if (hasNoArtifacts()) {
            return;
        }

        ArtifactLocation location = activeLocation;
        if (location == null) {
            location = ArtifactLocation.SLOT_ONE;
        }

        while (!hasArtifact(location)) {
            location = location.getNextLocation();
        }

        loadFromLocation(location);
    }

    public void beginIntaking() {
        intakeIntoLocation(activeLocation);
    }

    public void beginLoading() {
        loadFromLocation(activeLocation);
    }

    public void shootLoadedArtifact() {
        if (curState == SpindexerState.LOADING && activeLocation != null) {
            artifactDetections[activeLocation.ordinal()] = false;
            rotateToNextSlot();
        }
    }

    private double getAngle() {
        // see https://docs.sensorangerobotics.com/encoder/#analog-usage
        return AngleUnit.normalizeDegrees(
            (spindexerEncoder.getVoltage() - 0.043) / 3.1 * 360 + OFFSET_ANGLE);
    }

    private String getStateInfo() {
        final StringBuilder builder = new StringBuilder(curState.toString());
        switch (curState) {
            case LOADING:
            case INTAKING:
                builder.append(" { location = ").append(activeLocation).append(" }");
                break;
            case MANUAL_ROTATION:
                builder.append(" { power = ").append(spindexerServoOne.getPower()).append(" }");
                break;
        }
        return builder.toString();
    }

    private String getDetectionInfo() {
        assert ArtifactLocation.values().length > 0;

        final StringBuilder builder = new StringBuilder("{ ");

        for (ArtifactLocation location : ArtifactLocation.values()) {
            builder.append(location)
                .append(": ")
                .append(hasArtifact(location))
                .append(", ");
        }

        builder.replace(builder.length() - 2, builder.length() - 1, " }");

        return builder.toString();
    }

    public void logInfo() {
        telemetry.addData("Spindexer State", getStateInfo());
        telemetry.addData("Spindexer Angle (degrees)", getAngle());
        telemetry.addData("Spindexer Target Angle (degrees)", getTargetAngle());
        telemetry.addData(
            "Spindexer Close Angle (degrees)",
            closestEquivalentAngle(getAngle(), getTargetAngle(), AngleUnit.DEGREES)
        );
        telemetry.addData("Detections", getArtifactCount() + " " + getDetectionInfo());
        telemetry.addData("Detected Distance", this.frontColorSensor.getDistance(DistanceUnit.CM));
    }
}
