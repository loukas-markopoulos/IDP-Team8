#include <Arduino.h>

struct PathStep {
    int nodeId;                  // Unique ID for each node
    String exitDirection;        // Direction the robot should take when leaving ('straight','left','right','turnaround')
};

PathStep sample_path[] = {
    {2, 'straight'},
    {1, 'right'},
    {4, 'right'},
    {5, 'left'},
    {11, 'straight'},
    {7, 'right'},
    {8, 'straight'},
    {13, 'right'},
    {6, 'right'},
    {5, 'straight'},
    {4, 'left'},
    {1, 'left'},
    {2, 'straight'},
}

void followPath(PathStep path[]) {
    int pathLength = 13;

    for (int i=0; i < pathLength; i++) {
        PathStep step = path[i];

        if (step.exitDirection == 'straight') {
            moveStraight();
        }
        else if (step.exitDirection == 'left') {
            turnLeft();
        }
        else if (step.exitDirection == 'right') {
            turnRight();
        }

        delay(500);
    }
}