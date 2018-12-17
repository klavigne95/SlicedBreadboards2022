package org.firstinspires.ftc.teamcode.game.robot;

/**
 * Created by urmom on 12/17/18.
 */

public class yeesusConverstion {
    public static final double inchesInAYeet = 0.825; // Maybe
    public static final double tileSize = 23.0;

    public static int tileToYeet(double input) { return (int)(input * tileSize / inchesInAYeet); }

    public static int inchesToYeet(double input) { return (int)(input / inchesInAYeet); }
}
