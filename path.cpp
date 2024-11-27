

// Directions defined are from the robot's perspective
// Magnetic dropoff is node 9, NonMagnetic dropoff is node 10

struct PathStep {
    int nodeId;                  // Unique ID for each node
    String exitDirection;        // Direction the robot should take when leaving ('straight','left','right','turnaround')
    String conditionType;        // Decide which dropoff to go to
};

PathStep box1_magnetic[] = {
    {7, 'right', ''},
    {11, 'right', ''},
    {9, 'turn around', ''},
    {11, 'right', ''},
};

PathStep box1_nonMagnetic[] = {
    {7, 'straight', ''},
    {8, 'right', ''},
    {10, 'turn around', ''},
    {8, 'left', ''},
    {7, 'left', ''},
    {11, 'straight', ''}
};

PathStep box2_magnetic[] = {
    {11, 'left', ''},
    {9, 'turn around', ''},
    {11, 'left', ''},
    {7, 'right', ''},
    {8, 'straight', ''},
};

PathStep box2_nonMagnetic[] = {
    {11, 'straight', ''},
    {7, 'right', ''},
    {8, 'right', ''},
    {10, 'turn around', ''},
    {8, 'right', ''},
};

PathStep box3_magnetic[] = {
    {11, 'left', ''},
    {9, 'turn around', ''},
    {11, 'left', ''},
    {7, 'right', ''},
    {8, 'straight', ''},
    {13, 'right', ''},
};

PathStep box3_nonMagnetic[] = {
    {11, 'straight', ''},
    {7, 'right', ''},
    {8, 'right', ''},
    {10, 'turn around', ''},
    {8, 'right', ''},
};

PathStep box4_magnetic[] = {
    {7, 'right', ''},
    {11, 'right', ''},
    {9, 'turn around', ''},
    {11, 'left', ''},
    {7, 'right', ''},
    {8, 'straight', ''},
};

PathStep box4_nonMagnetic[] = {
    {7, 'straight', ''},
    {8, 'right', ''},
    {10, 'turn around', ''},
    {8, 'right', ''},
};

PathStep box5_magnetic[] = {
    {11, 'left', ''},
    {9, 'turn around', ''},
    {11, 'left', ''},
    {7, 'left', ''},
};

PathStep box5_nonMagnetic[] = {
    {11, 'straight', ''},
    {7, 'right', ''},
    {8, 'right', ''},
    {10, 'turn around', ''},
    {8, 'left', ''},
    {7, 'straight', ''},
};

PathStep box6_magnetic[] = {
    {6, 'left', ''},
    {5, 'right', ''},
    {11, 'left', ''},
    {9, 'turn around', ''},
    {11, 'right', ''},
    {5, 'right', ''},
    {4, 'left', ''},
    {1, 'left', ''},
    {2, 'right', ''},
};

PathStep box6_nonMagnetic[] = {
    {6, 'straight', ''},
    {13, 'left', ''},
    {8, 'left', ''},
    {10, 'turn around', ''},
    {8, 'right', ''},
    {13, 'right', ''},
    {6, 'straight', ''},
    {3, 'right', ''},
    {2, 'left', ''},
};



PathStep main_path[] = {
    // BOX1 NODE 2
    {0, 'straight', ''},
    // PICKUP BOX FUNCTION
    {2, 'left', ''},
    {1, 'right', ''},
    {4, 'straight', ''},
    {12, 'right', 'checkBox1Type'},
    // Magnetic (9) from 12
    // Non Magnetic (10) from 12
    // Drop Box


    // BOX2 NODE 5
    // PICKUP BOX FUNCTION
    {5, 'turnaround', 'checkBox2Type'},
    // Magnetic (9) from 5
    // Non Magnetic (10) from 5
    // Drop Box


    // BOX3 NODE 6-5
    {13, 'right', ''},
    {6, 'right', ''},
    // PICKUP BOX FUNCTION  
    {5, 'right', 'checkBox3Type'},
    // Magnetic (9) from 5
    // Non Magnetic (10) from 5
    // Drop Box


    // BOX4 NODE 2-3
    {13, 'right', ''},
    {6, 'straight', ''},
    {3, 'right', ''},
    // PICKUP BOX FUNCTION
    {2, 'straight', ''}
    {1, 'right', ''},
    {4, 'straight', ''},
    {12, 'right', 'checkBox4Type'}
    // Magnetic (9) from 12
    // Non Magnetic (10) from 12
    // Drop Box


    // BOX5 OFF NODE 6-5 LINE
    {13, 'right', ''},
    {6, 'right', ''},
    // PICKUP BOX OFF LINE FUNCTION 
    {5, 'right', 'checkBox5Type'},
    // Magnetic (9) from 5
    // Non Magnetic (10) from 5
    // Drop Box


    // BOX6 OFF NODE 1-2 LINE
    {12, 'left', ''},
    {4, 'straight', ''},
    {1, 'left', ''},
    // PICKUP BOX FUNCTION
    {2, 'straight', ''},
    {3, 'left', 'checkBox6Type'},
    // Magnetic (9) from 3 and back to start
    // Non Magnetic (9) from 3 and back to start
    // Drop Box
};


// TODO: replace all repeated code (if straight moveStraight() etc.) with followPath() functions)

void followPath(PathStep main_path[]) {

    int pathLengthMain = sizeof(main_path) / sizeof(PathStep);
    bool magnetic = false;

    for (int i=0; i < pathLengthMain; i++) {
        PathStep step = main_path[i];

        if (step.conditionType == 'checkBox1Type') {
            if (magnetic) {
                int pathLength1M = sizeof(box1_magnetic) / sizeof(PathStep);
                for (int j=0; j < pathLength1M; j++) {
                    PathStep step = box1_magnetic[j];
                    if (step.exitDirection == 'straight') {
                        moveStraight();
                    }
                    else if (step.exitDirection == 'left') {
                        turnLeft();
                    }
                    else if (step.exitDirection == 'right') {
                        turnRight();
                    }
                    else {
                        turnAround();
                    }
                    delay(500);
                }
            }
            else {
                int pathLength1NM = sizeof(box1_nonMagnetic) / sizeof(PathStep);
                for (int j=0; j < pathLength1NM; j++) {
                    PathStep step = box1_nonMagnetic[j];
                    if (step.exitDirection == 'straight') {
                        moveStraight();
                    }
                    else if (step.exitDirection == 'left') {
                        turnLeft();
                    }
                    else if (step.exitDirection == 'right') {
                        turnRight();
                    }
                    else {
                        turnAround();
                    }
                    delay(500);
                }

            }
        }

        if (step.conditionType == 'checkBox2Type') {
            if (magnetic) {
                int pathLength2M = sizeof(box2_magnetic) / sizeof(PathStep);
                for (int j=0; j < pathLength2M; j++) {
                    PathStep step = box2_magnetic[j];
                    if (step.exitDirection == 'straight') {
                        moveStraight();
                    }
                    else if (step.exitDirection == 'left') {
                        turnLeft();
                    }
                    else if (step.exitDirection == 'right') {
                        turnRight();
                    }
                    else {
                        turnAround();
                    }
                    delay(500);
                }
            }
            else {
                int pathLength2NM = sizeof(box2_nonMagnetic) / sizeof(PathStep);
                for (int j=0; j < pathLength2NM; j++) {
                    PathStep step = box2_nonMagnetic[j];
                    if (step.exitDirection == 'straight') {
                        moveStraight();
                    }
                    else if (step.exitDirection == 'left') {
                        turnLeft();
                    }
                    else if (step.exitDirection == 'right') {
                        turnRight();
                    }
                    else {
                        turnAround();
                    }
                    delay(500);
                }

            }
        }

        if (step.conditionType == 'checkBox3Type') {
            if (magnetic) {
                int pathLength3M = sizeof(box3_magnetic) / sizeof(PathStep);
                for (int j=0; j < pathLength3M; j++) {
                    PathStep step = box1_magnetic[j];
                    if (step.exitDirection == 'straight') {
                        moveStraight();
                    }
                    else if (step.exitDirection == 'left') {
                        turnLeft();
                    }
                    else if (step.exitDirection == 'right') {
                        turnRight();
                    }
                    else {
                        turnAround();
                    }
                    delay(500);
                }
            }
            else {
                int pathLength3NM = sizeof(box3_nonMagnetic) / sizeof(PathStep);
                for (int j=0; j < pathLength3NM; j++) {
                    PathStep step = box3_nonMagnetic[j];
                    if (step.exitDirection == 'straight') {
                        moveStraight();
                    }
                    else if (step.exitDirection == 'left') {
                        turnLeft();
                    }
                    else if (step.exitDirection == 'right') {
                        turnRight();
                    }
                    else {
                        turnAround();
                    }
                    delay(500);
                }

            }
        }

        if (step.conditionType == 'checkBox4Type') {
            if (magnetic) {
                int pathLength4M = sizeof(box4_magnetic) / sizeof(PathStep);
                for (int j=0; j < pathLength4M; j++) {
                    PathStep step = box4_magnetic[j];
                    if (step.exitDirection == 'straight') {
                        moveStraight();
                    }
                    else if (step.exitDirection == 'left') {
                        turnLeft();
                    }
                    else if (step.exitDirection == 'right') {
                        turnRight();
                    }
                    else {
                        turnAround();
                    }
                    delay(500);
                }
            }
            else {
                int pathLength4NM = sizeof(box4_nonMagnetic) / sizeof(PathStep);
                for (int j=0; j < pathLength4NM; j++) {
                    PathStep step = box4_nonMagnetic[j];
                    if (step.exitDirection == 'straight') {
                        moveStraight();
                    }
                    else if (step.exitDirection == 'left') {
                        turnLeft();
                    }
                    else if (step.exitDirection == 'right') {
                        turnRight();
                    }
                    else {
                        turnAround();
                    }
                    delay(500);
                }

            }
        }

        if (step.conditionType == 'checkBox5Type') {
            if (magnetic) {
                int pathLength5M = sizeof(box5_magnetic) / sizeof(PathStep);
                for (int j=0; j < pathLength5M; j++) {
                    PathStep step = box5_magnetic[j];
                    if (step.exitDirection == 'straight') {
                        moveStraight();
                    }
                    else if (step.exitDirection == 'left') {
                        turnLeft();
                    }
                    else if (step.exitDirection == 'right') {
                        turnRight();
                    }
                    else {
                        turnAround();
                    }
                    delay(500);
                }
            }
            else {
                int pathLength5NM = sizeof(box5_nonMagnetic) / sizeof(PathStep);
                for (int j=0; j < pathLength5NM; j++) {
                    PathStep step = box5_nonMagnetic[j];
                    if (step.exitDirection == 'straight') {
                        moveStraight();
                    }
                    else if (step.exitDirection == 'left') {
                        turnLeft();
                    }
                    else if (step.exitDirection == 'right') {
                        turnRight();
                    }
                    else {
                        turnAround();
                    }
                    delay(500);
                }

            }
        }

        if (step.conditionType == 'checkBox6Type') {
            if (magnetic) {
                int pathLength6M = sizeof(box6_magnetic) / sizeof(PathStep);
                for (int j=0; j < pathLength6M; j++) {
                    PathStep step = box6_magnetic[j];
                    if (step.exitDirection == 'straight') {
                        moveStraight();
                    }
                    else if (step.exitDirection == 'left') {
                        turnLeft();
                    }
                    else if (step.exitDirection == 'right') {
                        turnRight();
                    }
                    else {
                        turnAround();
                    }
                    delay(500);
                }
            }
            else {
                int pathLength6NM = sizeof(box6_nonMagnetic) / sizeof(PathStep);
                for (int j=0; j < pathLength6NM; j++) {
                    PathStep step = box6_nonMagnetic[j];
                    if (step.exitDirection == 'straight') {
                        moveStraight();
                    }
                    else if (step.exitDirection == 'left') {
                        turnLeft();
                    }
                    else if (step.exitDirection == 'right') {
                        turnRight();
                    }
                    else {
                        turnAround();
                    }
                    delay(500);
                }

            }
        }
  

        if (step.exitDirection == 'straight') {
            moveStraight();
        }
        else if (step.exitDirection == 'left') {
            turnLeft();
        }
        else if (step.exitDirection == 'right') {
            turnRight();
        }
        else {
            turnAround();
        }
        delay(500);
    }
}
