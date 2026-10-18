package org.firstinspires.ftc.teamcode.Barry.Auto;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Barry.Field.HiveTagGeometry;
import org.firstinspires.ftc.teamcode.Barry.Pedro.FtcToPedro;
import org.firstinspires.ftc.teamcode.Barry.Vision.CameraMount;
import org.firstinspires.ftc.teamcode.Barry.Vision.ClusterDetection;

/**
 * Start position on the wall, shared by HiveAuto and the test OpModes.
 *
 * Red starts with its right side flush on the audience wall at heading 0, camera / shooter out the
 * left toward the HIVE. Blue is the same reflected through the field center (far wall, heading 180).
 * Positions are given as the "red x" along the wall (Pedro x on red; blue uses 144 - x).
 */
public final class StartPosition {

    private StartPosition() {}

    /** First tag ID of the alliance's audience-side cluster: the one the robot sees at the start. */
    public static int audienceClusterId(boolean blue) {
        return blue ? 42 : 34;
    }

    /**
     * Red x along the wall from the audience-side cluster seen while flush on the wall. The robot's
     * forward axis is field +x (red frame), so the robot sits the cluster's forward offset behind the
     * cluster's x, which is fixed by the HIVE geometry (across the arm, it does not change with tilt).
     */
    public static double redXFromCamera(ClusterDetection audience) {
        double forward = CameraMount.toRobot(audience.x, audience.y, audience.z)[0];
        HiveTagGeometry.TagPose nominal = HiveTagGeometry.clusterPose(true, true, 30);
        return FtcToPedro.point(nominal.x, nominal.y)[0] - forward;
    }

    /** Pedro start pose flush on the wall at the given red x. */
    public static Pose pose(double redX, boolean blue) {
        double y = HiveAuto.ROBOT_HALF_WIDTH;
        return blue
                ? new Pose(144.0 - redX, 144.0 - y, Math.PI)
                : new Pose(redX, y, 0.0);
    }
}
