package org.firstinspires.ftc.teamcode.John.Auto;

import org.firstinspires.ftc.teamcode.John.Hardware.Shooter;

/**
 * [John] Auto with the features that are not field-tested yet switched on, for a practice field
 * where the Dashboard cannot be used (and for the matches, once one of them has proven itself:
 * select it on the Driver Station, no rebuild). Each variant sets its values for its own run only
 * and puts the competition defaults back afterwards (the values are static and outlive an OpMode).
 *
 *  - [John] Auto TEST square (JohnAutoTestSquare): the GARDEN square (0 deg) with the diagonal wall touch, one back-off
 *    and second push on a stall, and the gradual flywheel raise for volley 2 (ball 2 waits less).
 *  - [John] Auto TEST 15 deg (JohnAutoTest15): the same with the GARDEN angled 15 deg.
 *
 * The variant is the first event in the log ("variant: ...").
 */
public abstract class JohnAutoTest extends JohnAuto {

    /** Sets this variant's values (called before the run). */
    protected abstract void apply();

    @Override
    public void runOpMode() {
        double angle = GARDEN_ANGLE_DEG;
        boolean touch = GARDEN_TOUCH;
        int retries = GARDEN_END_RETRIES;
        double ramp = Shooter.PRESPIN_RAMP_RPM_PER_S;
        String variant = VARIANT;
        try {
            apply();
            super.runOpMode();
        } finally {
            GARDEN_ANGLE_DEG = angle;
            GARDEN_TOUCH = touch;
            GARDEN_END_RETRIES = retries;
            Shooter.PRESPIN_RAMP_RPM_PER_S = ramp;
            VARIANT = variant;
        }
    }

    /** The new GARDEN and volley-2 features, square. */
    protected static void newFeatures(double angleDeg, String name) {
        GARDEN_ANGLE_DEG = angleDeg;
        GARDEN_TOUCH = true;
        GARDEN_END_RETRIES = 1;
        Shooter.PRESPIN_RAMP_RPM_PER_S = 150;
        VARIANT = name;
    }
}
