package org.firstinspires.ftc.teamcode.game.robot;

/**
 * Created by urmom on 12/17/18.
 */

public class Convert {
    public static final double inchesInAYeet = 2.00; // GYRO
    public static final double inchesInAYeet2 = 0.825; // NORMAL
    public static final double inchesInAYeet3 = 0.825; // GYRO HORIZONTAL
    public static final double tileSize = 23.5;

    public static int inchesToYeet(double input) { return (int)(input / inchesInAYeet); }

    public static int inchesToYeet2(double input) { return (int)(input / inchesInAYeet2); }

    public static int inchesToYeet3(double input) { return (int)(input / inchesInAYeet3); }

    public static int tileToYeet(double input) { return (int)(input * tileSize / inchesInAYeet); }

    public static int tileToYeet2(double input) { return (int)(input * tileSize / inchesInAYeet2); }

    public static int tileToYeet3(double input) { return (int)(input * tileSize / inchesInAYeet3); }
}
