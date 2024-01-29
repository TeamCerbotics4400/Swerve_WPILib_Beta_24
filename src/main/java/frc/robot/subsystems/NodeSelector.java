// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class NodeSelector extends SubsystemBase {
  /** Creates a new NodeSelector. */
  private int currentSelection;
  private ArrayList<String> pointsOfInterest = new ArrayList<String>();
  Joystick joy;

  public static PathPlannerPath povToAlign;

  public NodeSelector(Joystick joy) {
    this.joy = joy;
    this.currentSelection = 0;

    pointsOfInterest.add("Speaker Orbit Align");
    pointsOfInterest.add("Amp Align");
    pointsOfInterest.add("Source Align");
    pointsOfInterest.add("Left Stage Align");
    pointsOfInterest.add("Center Stage Align");
    pointsOfInterest.add("Right Stage Align");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    displaySelection();
  }

  public void updateSelectionUp(){
    int pov = joy.getPOV();

    if(pov == 0){
      currentSelection++;
      if(currentSelection >= pointsOfInterest.size()){
        currentSelection = 0;
      }
    }  
  }

  public void updateSelectionDown(){
    int pov = joy.getPOV();

    if(pov == 180){
      currentSelection--;
      if(currentSelection < 0){
        currentSelection = pointsOfInterest.size() - 1;
      }
    }  
  }

  public void displaySelection(){
    
    String currentKeyLevels = pointsOfInterest.get(currentSelection);

    if (currentKeyLevels != null) {
      // Get the string representation of the selected entry
  
      // Display the selected entry on the SmartDashboard
      SmartDashboard.putString("Selected POV", currentKeyLevels);
      povToAlign = PathPlannerPath.fromPathFile(currentKeyLevels);

      //nodeToAlign = pose_map.get(currentKeyNodes);
    } else {
      // Display a message indicating that the selected entry is null
      SmartDashboard.putString("Selected Entry", "No node selected");
    }
  }

  public PathPlannerPath getPOVToAlign(){
    return povToAlign;
  }

  public String getAlignName(){
    String currentKey = pointsOfInterest.get(currentSelection);
    return currentKey;
  }

  public void selectPOV(int pov){
    currentSelection = pov;
  }
}
