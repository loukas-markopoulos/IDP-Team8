

// // Directions defined are from the robot's perspective
// // Magnetic dropoff is node 9, NonMagnetic dropoff is node 10

// struct PathStep {
//     int nodeId;                  // Unique ID for each node
//     String exitDirection;        // Direction the robot should take when leaving ('straight','left','right','turnaround')
// };

// PathStep main_path[] = {
//     // BOX NODE 2
//     {0, 'straight'},
//     {2, 'left'},
//     {1, 'right'},
//     {4, 'straight'},
//     {12, 'right'},
//     // Magnetic (9) from 12
//     if (magnetic) {
//         {7, 'right'},
//         {11, 'right'},
//         {9, 'turnaround'}, 
//         {11, 'right'},
//     }
//     // Non Magnetic (10) from 12
//     else {
//         {7, 'straight'},
//         {8, 'right'},
//         {10, 'turnaround'},
//         {8, 'left'},
//         {7, 'left'},
//         {11, 'straight'},
//     }

//     // BOX NODE 5
//     {5, 'turnaround'},
//     // Magnetic (9) from 5
//     if (magnetic) {
//         {11, 'left'},
//         {9, 'turnaround'},
//         {11, 'left'},
//         {7, 'right'},
//         {8, 'straight'},
//     }
//     // Non Magnetic (10) from 5
//     else {
//         {11, 'straight'},
//         {7, 'right'},
//         {8, 'right'},
//         {10, 'turnaround'},
//         {8, 'right'},
//     }

//     // BOX NODE 6-5
//     {13, 'right'},
//     {6, 'right'},
//     {5, 'right'},
//     // Magnetic (9) from 5
//     if (magnetic) {
//         {11, 'left'},
//         {9, 'turnaround'},
//         {11, 'right'},
//     }
//     // Non Magnetic (10) from 5
//     else {
//         {11, 'straight'},
//         {7, 'right'},
//         {8, 'right'},
//         {10, 'turnaround'},
//         {8, 'left'},
//         {7, 'left'},
//         {11, 'straight'},
//     }

//     // BOX NODE 2-3
//     {5, 'right'},
//     {4, 'left'},
//     {1, 'left'},
//     {2, 'straight'},
//     {3, 'left'},
//     {6, 'left'},
//     {5, 'right'},
//     // Magnetic (9) from 5
//     if (magnetic) {
//         {11, 'left'},
//         {9, 'turnaround'},
//         {11, 'left'},
//         {7, 'right'},
//         {8, 'straight'},
//     }
//     // Non Magnetic (10) from 5
//     else {
//         {11, 'straight'},
//         {7, 'right'},
//         {8, 'right'},
//         {10, 'turnaround'},
//         {8, 'right'},
//     }

//     // BOX OFF NODE 6-5 LINE

//     {, ''},
//     {, ''},
//     {, ''},
//     {, ''},
//     {, ''},
//     {, ''},
//     {, ''},
//     {, ''},
//     {, ''},
//     {, ''},
//     {, ''},
//     {, ''},
//     {, ''},
//     {, ''},
//     {, ''},
//     {, ''},
//     {, ''},
//     {, ''},
// };