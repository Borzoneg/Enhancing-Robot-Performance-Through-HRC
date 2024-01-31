# Enhancing robot performance through HRC

## Case:
The robot and the human collaborate to join together two part through a plaque that will be put in the middle

## Scene description
- The parts lies on the table ready to be picked up
- The screws are inside a box
- The screwdriver is inside the same box
- The robot knows the location of the box from a camera
- The robot knows the location of the aprts from ...
- The part is placed twice:
    - When the human is ready click something on the GUI and the robot pick up a part, and position itself over the track
    - The human can fine tune the alignment through teach mode or moving the tcp from the GUI
    - When the human thinks the two are aligned it presses a button and the robot insert the part into the track
- *The robot move the box close to the human (maybe)*
- When the human is ready click something on the GUI and the robot pick up the joining plaque and hold it in place
- From the GUI the human can put the robot in teach mode or move it with arrows
- While the robot hold this in place the human screw it
- Once the job is done the robot moves the finished part away

## Robot 
### Parameters
- Joint position: nparray((1, 6))
- Holding an object: boolean
- TCP pose: nparray((4, 4))
- Gripper status: boolean
### Actions
#### Public
- Grab object (pick location)
- Hold object in place (pick location, hold location): Grab obj + Move TCP 
#### Private
- Move TCP (pose)
## GUI
### Buttons
- Hold part over the rail
    - Enter teach mode
    - Move TCP up, down, ...
- Move box close
- Hold plaque between the two parts
    - Enter teach mode
    - Move TCP up, down, ...
### Monitoring
- Tcp xyz, rpy of the real robot will be updated in the GUI
- Status of the task: showing which of the two parts are holding(blue) placed(green), missing(gray), misplaced(red), same for the plaque with just missing(gray), holding(blue)
