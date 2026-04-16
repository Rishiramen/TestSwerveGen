package frc.robot.Constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Constants {
    public static InterpolatingDoubleTreeMap metersToRPM = new InterpolatingDoubleTreeMap();
    
    static {
        metersToRPM.put(0.05, 3000.0);
        metersToRPM.put(2.65, 4700.0);
        metersToRPM.put(1.87, 3800.0);
        metersToRPM.put(2.446, 4450.0);
        metersToRPM.put(2.35, 4430.0);
        metersToRPM.put(3.4, 5000.0);
        metersToRPM.put(3.6, 5150.0);
        metersToRPM.put(5.1, 5700.0);
        metersToRPM.put(3.0, 4800.0);
        metersToRPM.put(4.0, 5450.0);
        metersToRPM.put(2.0, 4000.0);
    }

    public static double getRPM(double meters)
    {
        return metersToRPM.get(meters);
    }
}
