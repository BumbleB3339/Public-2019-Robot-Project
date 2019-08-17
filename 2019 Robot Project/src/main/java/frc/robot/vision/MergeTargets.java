/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;
import frc.bumblelib.util.hardware.pixy.PixyObject;
import frc.bumblelib.util.hardware.pixy.PixyObjectList;
import frc.bumblelib.util.hardware.pixy.PixyTarget;
import frc.bumblelib.util.hardware.pixy.PixyTargetList;
import frc.robot.RobotGameState.FieldObjective;
import frc.robot.RobotGameState.GamePiece;
import frc.robot.RobotGameState.RobotAction;
import frc.robot.RobotGameStateManager;

/**
 * This class is responsible of merging objects detected by the vision target into "vision targets".
 * As a reminder, the vision targets in DESTINATION: DEEP SPACE are built from TWO seperate pieces of retroreflective tape.
 * As Pixy doesn't provide data on the angle of the vision rectangles releative to the ground, targets were merged based on
 * their distance from each other, their height from the ground and et cetere...
 * This class also contains some basic algorithems to help cope with the LED ring reflecting from the material from which the 
 * FEEDER is made.
 */
public class MergeTargets {

    private static final double HEIGHT_WIDTH_RATIO_TOLERANCE = 0.25;
    private PixyObject[] objectsArr = new PixyObject[6];
    private PixyTargetList mergedTargetsList = new PixyTargetList();
    private final int DISTANCE_Y_DIFFERENCE = 22; // In pixels

    public static Side forcedSideMergingSide = Side.LEFT;
    public static boolean isForcedSideMerging = false;

    private double LEDRingRatio = 1.0; // H/W

    public MergeTargets() {
    }

    private PixyTarget mergeBetweenTwoObjects(PixyObject objectOne, PixyObject objectTwo) {
        double x = (objectOne.getX() + objectTwo.getX()) / 2;
        double y = (objectOne.getY() + objectTwo.getY()) / 2;
        double width = (int) (Math.abs(objectOne.getX() - objectTwo.getX())
                + 0.5 * (objectOne.getWidth() + objectTwo.getWidth()));
        double height = (objectOne.getHeight() + objectTwo.getHeight()) / 2;
        return new PixyTarget(x, y, width, height);
    }

    private void updateObjects(PixyObjectList objectList) {
        int listSize = objectList.size();
        int arrIndex = 0;
        for (int i = 0; i < objectsArr.length; i++) {
            if (i < listSize) {
                if ((double) (objectList.get(i).getHeight()) / objectList.get(i).getWidth() > LEDRingRatio
                        + HEIGHT_WIDTH_RATIO_TOLERANCE) {
                    objectsArr[arrIndex] = objectList.get(i);
                    arrIndex++;
                } else {
                    objectList.remove(objectList.get(i));
                    i--;
                    listSize--;
                }
            } else {
                objectsArr[arrIndex] = null;
                arrIndex++;
            }
        }
        for (int i = arrIndex; i < objectsArr.length; i++) {
            objectsArr[i] = null;
        }
        if (listSize == 3) { // avoid LED ring detection in feeder
            if (objectsArr[0].getArea() * 3.5 < (objectsArr[1].getArea() + objectsArr[2].getArea() / 2.0)) {
                objectsArr[0] = objectsArr[1];
                objectsArr[1] = objectsArr[2];
                objectsArr[2] = null;
                objectList.remove(0);
            } else if (objectsArr[1].getArea() * 3.5 < (objectsArr[0].getArea() + objectsArr[2].getArea() / 2.0)) {
                objectsArr[1] = objectsArr[2];
                objectsArr[2] = null;
                objectList.remove(1);
            } else if (objectsArr[2].getArea() * 3.5 < (objectsArr[0].getArea() + objectsArr[1].getArea() / 2.0)) {
                objectsArr[2] = null;
                objectList.remove(2);
            }
        }
    }

    public PixyTargetList getMergedTargets(PixyObjectList objectList) {
        updateObjects(objectList);

        mergedTargetsList.clear();

        if (isForcedSideMerging && objectList.size() >= 2) {
            switch (forcedSideMergingSide) {
            case RIGHT:
                mergedTargetsList.add(
                        mergeBetweenTwoObjects(objectsArr[objectList.size() - 2], objectsArr[objectList.size() - 1]));
                break;
            case LEFT:
                mergedTargetsList.add(mergeBetweenTwoObjects(objectsArr[0], objectsArr[1]));

                break;
            }
            return mergedTargetsList;
        }

        switch (objectList.size()) {
        case 1:
            if (((RobotGameStateManager.nextGameState.fieldObjective == FieldObjective.ROCKET
                    && RobotGameStateManager.nextGameState.gamePiece == GamePiece.CARGO)
                    || RobotGameStateManager.currentGameState.robotAction == RobotAction.FEEDER_COLLECT)
                    && PixyReflectiveHeightToDistance
                            .getDistance(objectsArr[0].getHeight()) > AlignToTarget.MIN_DISTANCE_TO_ALLOW_ONE_TARGET) {
                PixyTarget target = new PixyTarget(objectsArr[0]);
                target.setSingleReflective(true);
                mergedTargetsList.add(target);
            }
            break;
        case 2:
            int object0BottomY = objectsArr[0].getY() - objectsArr[0].getHeight() / 2;
            int object0UpperY = objectsArr[0].getY() + objectsArr[0].getHeight() / 2;
            int object1BottomY = objectsArr[1].getY() - objectsArr[1].getHeight() / 2;
            int object1UpperY = objectsArr[1].getY() + objectsArr[1].getHeight() / 2;

            if (Math.abs(object0BottomY - object1BottomY) < DISTANCE_Y_DIFFERENCE
                    || Math.abs(object0UpperY - object1UpperY) < DISTANCE_Y_DIFFERENCE) {
                if (Math.abs(object0BottomY - object1BottomY) > DISTANCE_Y_DIFFERENCE) {
                    // cut borrom part of lower target
                    PixyObject objectToCut;
                    PixyObject objectToKeep;
                    if (object0BottomY < object1BottomY) {
                        objectToCut = objectsArr[0];
                        objectToKeep = objectsArr[1];
                    } else {
                        objectToCut = objectsArr[1];
                        objectToKeep = objectsArr[0];
                    }
                    objectToCut.setY(objectToKeep.getY());
                    objectToCut.setHeight(objectToKeep.getHeight() + 2 * Math.abs(object1UpperY - object0UpperY));
                } else if (Math.abs(object0UpperY - object1UpperY) > DISTANCE_Y_DIFFERENCE) {
                    // cut upper part of higher target
                    PixyObject objectToCut;
                    PixyObject objectToKeep;
                    if (object0UpperY > object1UpperY) {
                        objectToCut = objectsArr[0];
                        objectToKeep = objectsArr[1];
                    } else {
                        objectToCut = objectsArr[1];
                        objectToKeep = objectsArr[0];
                    }
                    objectToCut.setY(objectToKeep.getY());
                    objectToCut.setHeight(objectToKeep.getHeight() + 2 * Math.abs(object1BottomY - object0BottomY));
                }

                mergedTargetsList.add(mergeBetweenTwoObjects(objectsArr[0], objectsArr[1]));
            }
            break;
        case 3:
            if (Math.abs(objectsArr[0].getY() - objectsArr[1].getY()) < DISTANCE_Y_DIFFERENCE
                    && Math.abs(objectsArr[1].getY() - objectsArr[2].getY()) < DISTANCE_Y_DIFFERENCE) {// Cargo ship
                if (Math.abs(objectsArr[0].getX() - objectsArr[1].getX()) > Math
                        .abs(objectsArr[1].getX() - objectsArr[2].getX())) {
                    mergedTargetsList.add(mergeBetweenTwoObjects(objectsArr[0], objectsArr[1]));
                } else {
                    mergedTargetsList.add(mergeBetweenTwoObjects(objectsArr[1], objectsArr[2]));
                }
            } else {// Rocket
                if (Math.abs(objectsArr[0].getY() - objectsArr[1].getY()) < Math
                        .abs(objectsArr[1].getY() - objectsArr[2].getY())) {
                    mergedTargetsList.add(mergeBetweenTwoObjects(objectsArr[0], objectsArr[1]));
                    if (RobotGameStateManager.nextGameState.gamePiece == GamePiece.CARGO) {
                        PixyTarget target = new PixyTarget(objectsArr[2]);
                        target.setSingleReflective(true);
                        mergedTargetsList.add(target);
                    }
                } else {
                    mergedTargetsList.add(mergeBetweenTwoObjects(objectsArr[1], objectsArr[2]));
                    if (RobotGameStateManager.nextGameState.gamePiece == GamePiece.CARGO) {
                        PixyTarget target = new PixyTarget(objectsArr[0]);
                        target.setSingleReflective(true);
                        mergedTargetsList.add(target);
                    }
                }
            }
            break;
        case 4:
            if (Math.abs(objectsArr[1].getY() - objectsArr[2].getY()) < DISTANCE_Y_DIFFERENCE) {// Cargo ship
                if (Math.abs(objectsArr[1].getX()
                        - objectsArr[2].getX()) > ((Math.abs(objectsArr[0].getX() - objectsArr[1].getX())))) {
                    mergedTargetsList.add(mergeBetweenTwoObjects(objectsArr[1], objectsArr[2]));
                } else {
                    mergedTargetsList.add(mergeBetweenTwoObjects(objectsArr[0], objectsArr[1]));
                    mergedTargetsList.add(mergeBetweenTwoObjects(objectsArr[2], objectsArr[3]));
                }
            } else {// Rocket
                mergedTargetsList.add(mergeBetweenTwoObjects(objectsArr[0], objectsArr[1]));
                mergedTargetsList.add(mergeBetweenTwoObjects(objectsArr[2], objectsArr[3]));
            }
            break;
        case 5:
            if (Math.abs(objectsArr[0].getX() - objectsArr[1].getX()) > Math
                    .abs(objectsArr[1].getX() - objectsArr[2].getX())) {
                mergedTargetsList.add(mergeBetweenTwoObjects(objectsArr[0], objectsArr[1]));
                mergedTargetsList.add(mergeBetweenTwoObjects(objectsArr[2], objectsArr[3]));
            } else {
                mergedTargetsList.add(mergeBetweenTwoObjects(objectsArr[1], objectsArr[2]));
                mergedTargetsList.add(mergeBetweenTwoObjects(objectsArr[3], objectsArr[4]));
            }
            break;
        case 6:
            mergedTargetsList.add(mergeBetweenTwoObjects(objectsArr[0], objectsArr[1]));
            mergedTargetsList.add(mergeBetweenTwoObjects(objectsArr[2], objectsArr[3]));
            mergedTargetsList.add(mergeBetweenTwoObjects(objectsArr[4], objectsArr[5]));
            break;
        }
        return mergedTargetsList;
    }
}
