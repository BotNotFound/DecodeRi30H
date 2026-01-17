package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmode.teleop.TwoPersonTeleOpBlue;

// arithmetic is applied uniformly to ensure everything is mirrored
@SuppressWarnings("PointlessArithmeticExpression")
@Autonomous(name = "12 Artifact Blue (no gate)", group = "blue", preselectTeleOp = TwoPersonTeleOpBlue.OP_MODE_NAME)
public class SimpleTwelveArtifactBlue extends AutonomousBase {

    public static Pose startPosition = new Pose(117.9, 131.3, Math.toRadians(-54)).mirror();
    public static Pose shootPosition = new Pose(110, 115).mirror();
    public static double headingToShoot = Math.toRadians(180 + 40);

    public SimpleTwelveArtifactBlue() {
        super(startPosition, AllianceColor.BLUE);
    }

    @Override
    protected AutonomousStage[] buildStageSequence() {
        Paths autoPath = new Paths(follower);
        return new AutonomousStage[]{
            new AutonomousStage(autoPath.shootPreloads, Robot.RobotState.NONE),
            new AutonomousStage(
                follower.pathBuilder()
                    .addPath(new BezierPoint(autoPath.shootPreloads.endPose()))
                    .setConstantHeadingInterpolation(headingToShoot)
                    .build(), Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPath.moveToFirstRow, Robot.RobotState.NONE),
            new AutonomousStage(autoPath.intakeFirstRow, Robot.RobotState.INTAKE),
            new AutonomousStage(autoPath.shootFirstRow, Robot.RobotState.NONE),
            new AutonomousStage(
                follower.pathBuilder()
                    .addPath(new BezierPoint(autoPath.shootFirstRow.endPose()))
                    .setConstantHeadingInterpolation(headingToShoot)
                    .build(), Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPath.moveToSecondRow, Robot.RobotState.NONE),
            new AutonomousStage(autoPath.intakeSecondRow, Robot.RobotState.INTAKE),
            new AutonomousStage(autoPath.shootSecondRow, Robot.RobotState.NONE),
            new AutonomousStage(
                follower.pathBuilder()
                    .addPath(new BezierPoint(autoPath.shootSecondRow.endPose()))
                    .setConstantHeadingInterpolation(Math.toRadians(180 - 35))
                    .build(), Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPath.moveToThirdRow, Robot.RobotState.NONE),
            new AutonomousStage(autoPath.intakeThirdRow, Robot.RobotState.INTAKE),
            new AutonomousStage(autoPath.shootThirdRow, Robot.RobotState.NONE),
            new AutonomousStage(
                follower.pathBuilder()
                    .addPath(new BezierPoint(autoPath.shootThirdRow.endPose()))
                    .setConstantHeadingInterpolation(Math.toRadians(180 - 35))
                    .build(), Robot.RobotState.SHOOT
            ),
            new AutonomousStage(autoPath.leaveLaunchZone, Robot.RobotState.NONE),

        };
    }

    public static class Paths {

        public PathChain shootPreloads;
        public PathChain moveToFirstRow;
        public PathChain intakeFirstRow;
        public PathChain shootFirstRow;
        public PathChain moveToSecondRow;
        public PathChain intakeSecondRow;
        public PathChain shootSecondRow;
        public PathChain moveToThirdRow;
        public PathChain intakeThirdRow;
        public PathChain shootThirdRow;
        public PathChain leaveLaunchZone;

        public Paths(Follower follower) {
            shootPreloads = follower.pathBuilder()
                .addPath(new BezierLine(startPosition, shootPosition))
                .setLinearHeadingInterpolation(startPosition.getHeading(), headingToShoot)
                .build();

            moveToFirstRow = follower.pathBuilder()
                .addPath(new BezierCurve(
                    shootPosition,
                    new Pose(98, 87).mirror(),
                    new Pose(100.9081, 87.3834).mirror()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180 - 0))
                .build();

            intakeFirstRow = follower.pathBuilder()
                .addPath(
                    new BezierLine(
                        new Pose(100.9081, 87.3834).mirror(),
                        new Pose(128, 87.267).mirror()
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180 - 0))
                .build();

            shootFirstRow = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(128, 78).mirror(), shootPosition))
                .setLinearHeadingInterpolation(Math.toRadians(180 - 0), headingToShoot)
                .build();

            moveToSecondRow = follower.pathBuilder()
                .addPath(new BezierCurve(
                    shootPosition,
                    new Pose(96.42, 64.07).mirror(),
                    new Pose(96.42, 64.07).mirror()
                ))
                .setLinearHeadingInterpolation(headingToShoot, Math.toRadians(180 - 0))
                .build();

            intakeSecondRow = follower.pathBuilder()
                .addPath(new BezierLine(
                    new Pose(96.42, 64.07).mirror(),
                    new Pose(134, 62.08).mirror()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180 - 0))
                .build();

            shootSecondRow = follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Pose(134, 62.08).mirror(),
                    new Pose(86.95, 95).mirror(),
                    shootPosition
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180 - 0), headingToShoot)
                .build();

            moveToThirdRow = follower.pathBuilder()
                .addPath(new BezierCurve(
                    shootPosition,
                    new Pose(97.06, 41).mirror(),
                    new Pose(97.06, 41.13).mirror()
                ))
                .setLinearHeadingInterpolation(headingToShoot, Math.toRadians(180 - 0))
                .build();

            intakeThirdRow = follower.pathBuilder()
                .addPath(new BezierLine(
                    new Pose(97.06, 41.13).mirror(),
                    new Pose(134, 39.93).mirror()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180 - 0), Math.toRadians(180 - 0))
                .build();

            shootThirdRow = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(134, 39.93).mirror(), shootPosition))
                .setLinearHeadingInterpolation(Math.toRadians(180 - 0), headingToShoot)
                .build();

            leaveLaunchZone = follower.pathBuilder()
                .addPath(new BezierCurve(
                    shootPosition,
                    new Pose(111.52, 78.55).mirror(),
                    new Pose(111.52, 78.55).mirror()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180 - 0))
                .build();
        }
    }
}

