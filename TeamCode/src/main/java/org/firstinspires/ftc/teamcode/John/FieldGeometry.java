package org.firstinspires.ftc.teamcode.John;

/**
 * Where the field walls are, FTC field frame (inches, origin at the field center). Every pose taken
 * from a wall uses it: the start pose (camera wall flush on the start wall), the GARDEN run along
 * the audience wall, the park along the red / blue wall, TeleOp's wall re-positioning.
 *
 * 2026-11-15: the inside of the competition field measured 142" x 142" (the nominal 144" includes
 * the walls), so the walls are at +-71, not +-72: every wall pose was 1" off, which is most of the
 * 0.6-2.1" the Auto's GARDEN wall touch found at the practice field. If another field measures
 * differently, change WALL here (half the inside width).
 */
public final class FieldGeometry {

    /** Distance from the field center to the inside face of each wall. */
    public static final double WALL = 71.0;

    private FieldGeometry() {}
}
