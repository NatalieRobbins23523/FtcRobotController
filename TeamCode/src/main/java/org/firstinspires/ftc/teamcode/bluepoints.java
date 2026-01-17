package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class bluepoints {
    final Pose2D ballsInfrontFar = new Pose2D(DistanceUnit.INCH, -12, -36, AngleUnit.DEGREES, 180);
    final Pose2D ballsInfrontMiddle = new Pose2D(DistanceUnit.INCH, 12, -36, AngleUnit.DEGREES, 180);
    final Pose2D ballsInfrontClose1 = new Pose2D(DistanceUnit.INCH, 36, -31, AngleUnit.DEGREES, 180);
    final Pose2D ballsInfrontClose2 = new Pose2D(DistanceUnit.INCH, 36, -36, AngleUnit.DEGREES, 180);
    final Pose2D ballsAfterFar = new Pose2D(DistanceUnit.INCH, -12, -57.5, AngleUnit.DEGREES, 180);
    final Pose2D ballsAfterMiddle = new Pose2D(DistanceUnit.INCH, 12, -57.5, AngleUnit.DEGREES, 180);
    final Pose2D ballsAfterClose = new Pose2D(DistanceUnit.INCH, 36, -57.5, AngleUnit.DEGREES, 180);
    final Pose2D shooterPoint = new Pose2D(DistanceUnit.INCH, -24, -24, AngleUnit.DEGREES, -45);
    //    final Pose2D redAprilTagPosition = new Pose2D(DistanceUnit.INCH, -58.37, 55.64, AngleUnit.DEGREES, 46.4+90); // +90 because this is used to set odometry pose
//    final Pose2D blueAprilTagPosition = new Pose2D(DistanceUnit.INCH, -58.37, -55.64, AngleUnit.DEGREES, 133.6+90);
    final Pose2D initialBackupPoint = new Pose2D(DistanceUnit.INCH, 36, 0, AngleUnit.DEGREES,-90); //-90 because heading
    //is in field coordinates
    final Pose2D gatePoint1 = new Pose2D(DistanceUnit.INCH, -5, -50, AngleUnit.DEGREES,90);
    final Pose2D gatePoint2 = new Pose2D(DistanceUnit.INCH, -5, -56.5, AngleUnit.DEGREES,90);
    final Pose2D finalParkingPoint = new Pose2D(DistanceUnit.INCH, -6.5, -36, AngleUnit.DEGREES,0);

}
