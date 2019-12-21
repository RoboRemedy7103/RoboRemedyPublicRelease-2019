/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import edu.wpi.first.wpilibj.Timer;

/**
 * This creates a generic log form to be followed. Simplifies/cleans up our log
 * files
 */
public class RobotLog {
    DateFormat dtf = new SimpleDateFormat("MM/dd/yyyy HH:mm:ss");
    DateFormat timeDTF = new SimpleDateFormat("HH:mm:ss.SS");
    int count = 0;

    RobotLog() {
        String realTime = dtf.format(new Date());
        System.out.println("Public Release: " + realTime);
    }

    void incrmentCount() {
        count++;
    }

    void print(String message) {
        double matchTime = Timer.getMatchTime();
        if (matchTime != -1) {
            System.out.println(matchTime + ": " + message);
        } else {
            String realTime = timeDTF.format(new Date());
            System.out.println(realTime + " Pit: " + message);
        }
    }

    void printPeriodic(String message, int frequency) {
        if (count % frequency == 0) {
            print(message);
        }
    }

}
