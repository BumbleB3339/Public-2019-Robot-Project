/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.bumblelib.util;

import java.util.HashMap;
import java.util.Iterator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class BumbleTimer {
    private static boolean enabled = false;
    private static HashMap<String, Double> prevTimes = new HashMap<>();
    private static HashMap<String, Double> timeDiffs = new HashMap<>();

    public static void setEnabled(boolean enable) {
        enabled = enable;
    }
  
    public static Double getTimeDiff(String key) {
        return timeDiffs.get(key);
    }

    private static double getTimeMs() {
        // multiply by 1000 to convert seconds to millisceonds
        return Timer.getFPGATimestamp() * 1000;
    }

    /** 
     * Calculate dt from the last call to time() or to start()
     */
    public static double time(String key) {
        if (!enabled) return 0;

        double currentTime = getTimeMs();
        Double prevTime = prevTimes.get(key);

        double dt = 0;
        if (prevTime != null) {
            dt = currentTime - prevTime;

            timeDiffs.put(key, dt);
            SmartDashboard.putNumber("BumbleTimer/" + key, dt);
        }

        prevTimes.put(key, currentTime);

        return dt;
    }

    public static void start(String key) {
        if (!enabled) return;
        prevTimes.put(key, getTimeMs());
    }

    public static void sendAsArray() {
        if (!enabled) return;

        double[] arr = new double[timeDiffs.size()];
        Iterator<Double> vals = timeDiffs.values().iterator();
        for (int i = 0; i < arr.length; i++) {
            arr[i] = (double) vals.next();
        }

        SmartDashboard.putNumberArray("BumbleTimer/Array", arr);
        
    }
}
