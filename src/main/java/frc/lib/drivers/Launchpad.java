// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.drivers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Launchpad extends SubsystemBase{
    public boolean btns[][] = new boolean[9][9];
    public NetworkTableInstance nt;
    public NetworkTable table;
    
    public Launchpad()
    {
        nt = NetworkTableInstance.getDefault();
        table = nt.getTable("Launchpad");
    }
    
    public boolean isPressed(int row, int col)
    {
        SmartDashboard.putBoolean("Pressed", btns[row][col]);
        return btns[row][col];
    }

    @Override
    public void periodic()
    {
        for(int i = 0; i < 9; i++)
        {
            for(int j = 0; j < 9; j++)
            {
                String key = i +":"+j;
                btns[i][j] = table.getEntry(key).getBoolean(false);
            }
        }
    }

}