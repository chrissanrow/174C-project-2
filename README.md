# Blackboard Spline Tracing with Inverse Kinematics

## Implementation Line Numbers

### Specific Requirements

- The end effector is the end point of the right hand (see the figure below).

    - `human.js:L223-L229` - implemented as a single end effector on the human class, with the associated arc having a variable denoting whether the end effector is attached to it.

    - `End_Effector` class contains offset which represents its location relative to its parent's origin (which in this case is the right wrist)

    - `human.js:L290-L319` - `_get_effector` end position calls `_rec_update` which performs forward kinematics until encountering the arc with the end effector and applying the aformentioned offset to give the end effector's global location

- The character will remain aligned with the surface of the blackboard.

    - The character's body is aligned with the surface through the root only having translational DOF `human.js:L235`

- At the start of the animation, the human character is in its rest pose, and you should put the blackboard and starting point of the spline near its right hand.

    - `assignment2.js:L54-L62` - position of spline is on the blackboard, centered @ 3,6,-.88

    - `human.js:L96-L229` - human's default position is near the blackboard, as setup by the link hierarchy and no joint angles are applied initially to create default rest pose

- The character should move its right hand in a straight line to touch the blackboard and then draw the spline, all with a natural speed of movement.

    - `assignment2.js:L165-L176` - character's iterative IK method is set to move end effector smoothly to blackboard using lerp

    - `assignment2.js:L178-L183` - character traces spline using the spline's `get_position` function and a local_t parameter based on the animation's t value

- The required shape of the spline is specified below

    - `assignment2.js:L54-L62` - spline is implemented with 8 control points (center, left top loop, right top loop, top top loop, left bottom loop, right bottom loop, bottom bottom loop)

### Grading Scheme

1. Display a "classroom" with a floor plane and a blackboard on a planar wall parallel to the x-y plane (perpendicular to the floor).

    - `assignment2.js:L153-L159` - ground, wall, and board placement / draw

2. Display the spline on the blackboard. You can either display the spline from the beginning or gradually display it during the drawing animation.

    - `assignment2.js:L52-L65` - spline is implemented with 8 control points (center, left top loop, right top loop, top top loop, left bottom loop, right bottom loop, bottom bottom loop)
    
        - `Curve_shape` class extends base Shape class to enable hermite spline drawing (same as assignment 1)

    - `assignment2.js:L179` - only drawn once the blackboard is touched by human

3. Model the human character using ellipsoids (you can use the Sphere primitive with non-uniform scaling).

    - `human.js:L96-L229` - link hierarchy made with scaled Sphere primitives


4. Implement the inverse kinematics solver using the pseudoinverse approach.

    - `human.js:L256-281` - `iterative_inverse_kinematics(pg)` uses loop given in discussion to minimize error between current end effector position p and target pg

        - `human.js:L290-L319` - end effector position is obtained with these two functions

        - `human.js:L323-L344` - jacobian is estimated using derivative approximation as described in wikipedia page: https://en.wikipedia.org/wiki/Inverse_kinematics#The_Jacobian_inverse_technique

        - `human.jsL395-L410` - theta dot is computed using pseudinverse approach, optimizing by solving directly for theta dot with LU decomposition instead of plugging back in J+

5. Move the right hand from its initial position such that the end effector touches the board (initial animation).

    - `assignment2.js:L165-L176` - character's iterative IK method is set to move end effector smoothly to blackboard using lerp


6. Then, move the hand to draw the spline, looping repeatedly, since the spline is a closed shape.

    - `assignment2.js:L178-L183` - character traces spline using the spline's `get_position` function and a local_t parameter based on the animation's t value



## DEMO

(See /DEMO.mov)