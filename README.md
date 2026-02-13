# Blackboard Spline Tracing with Inverse Kinematics

## Implementation Line Numbers

### Specific Requirements

- The end effector is the end point of the right hand (see the figure below).

    - L

- The character will remain aligned with the surface of the blackboard.

    - L

- At the start of the animation, the human character is in its rest pose, and you should put the blackboard and starting point of the spline near its right hand.

    - L

- The character should move its right hand in a straight line to touch the blackboard and then draw the spline, all with a natural speed of movement.

    - L

- The required shape of the spline is specified below

    - L


### Grading Scheme

1. Display a "classroom" with a floor plane and a blackboard on a planar wall parallel to the x-y plane (perpendicular to the floor).

    - L

2. Display the spline on the blackboard. You can either display the spline from the beginning or gradually display it during the drawing animation.

    - L

3. Model the human character using ellipsoids (you can use the Sphere primitive with non-uniform scaling).

    - L

4. Implement the inverse kinematics solver using the pseudoinverse approach.

    - L

5. Move the right hand from its initial position such that the end effector touches the board (initial animation).

    - L

6. Then, move the hand to draw the spline, looping repeatedly, since the spline is a closed shape.

    - L


## DEMO

(See /DEMO.mov)