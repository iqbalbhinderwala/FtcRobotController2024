package org.firstinspires.ftc.teamcode.Barry.Pedro;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Converts from the FTC field frame to the Pedro frame (inches, radians).
 *
 * FTC:   origin at the field center, +X toward the audience wall, +Y toward the blue side.
 * Pedro: origin at the red / audience wall corner, +X toward the blue side, +Y away from the
 *        audience wall (0 to 144).
 * Pedro is FTC rotated -90 degrees and shifted by half a field:
 *   pedroX = ftcY + 72,  pedroY = 72 - ftcX,  pedroHeading = ftcHeading - pi/2
 */
public final class FtcToPedro {

    private static final double HALF_FIELD = 72.0;

    private FtcToPedro() {}

    /** FTC point -> Pedro point {x, y}. */
    public static double[] point(double ftcX, double ftcY) {
        return new double[]{ftcY + HALF_FIELD, HALF_FIELD - ftcX};
    }

    /** FTC direction or displacement -> Pedro {x, y} (rotation only, no shift). */
    public static double[] direction(double ftcX, double ftcY) {
        return new double[]{ftcY, -ftcX};
    }

    /** FTC heading -> Pedro heading, radians. */
    public static double heading(double ftcHeading) {
        return ftcHeading - Math.PI / 2.0;
    }

    /** FTC pose -> Pedro pose. */
    public static Pose pose(Pose2D ftcPose) {
        double[] p = point(ftcPose.getX(DistanceUnit.INCH), ftcPose.getY(DistanceUnit.INCH));
        return new Pose(p[0], p[1], heading(ftcPose.getHeading(AngleUnit.RADIANS)));
    }
}
