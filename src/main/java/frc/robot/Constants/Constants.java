package frc.robot.Constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Constants {
    public static InterpolatingDoubleTreeMap metersToRPM = new InterpolatingDoubleTreeMap();
    
    static {
        metersToRPM.put(0.05, 2000.0);
        metersToRPM.put(2.65, 2700.0);
        metersToRPM.put(1.87, 2270.0);
        metersToRPM.put(2.35, 2550.0);
        metersToRPM.put(3.4, 2750.0);
        metersToRPM.put(3.6, 2850.0);
        metersToRPM.put(5.1, 4400.0);
        metersToRPM.put(3.0, 2900.0);
        metersToRPM.put(4.0, 3450.0);
        metersToRPM.put(2.0, 2500.0);
    }

    public static double getRPM(double meters)
    {
        return metersToRPM.get(meters);
    }
}
