package org.firstinspires.ftc.teamcode.game.robot;

/**
 * Created by 20smcnamara on 12/17/18.
 */

public class yeesusConverstion {
    public static final double inchesInAYeet = 0.825; // Maybe
    public static final double tileSize = 23;

    public static double tileToYeet(double input) { return input * tileSize / inchesInAYeet; }

    public static double inchesToYeet(double input) { return input / inchesInAYeet; }
}
