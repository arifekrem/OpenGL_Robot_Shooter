To compile the program you will need Visual Studio (2022) for Windows, the files Robot3D.cpp, QuadMesh.cpp, QuadMesh.h, and VECTOR3D.h.
Because we are using VS you will also need the .sln and .vcxproj makefiles. 

IMPORTANT: To compile the project you may need the x64 folder which has freeglut.dll, this has been provided in the .zip submission.
On our end this folder in addition to the usual Dependencies folder is what was necessary to compile.

User inputs:
" " spacebar to fire the defensive cannon projectile
"R" key to reset the game, score, number of enemy robots, and re-enable the defensive cannon
"horizontal mouse movement" aim defensive camera left or right, along with the camera
"vertical mouse movement" aim defensive camera up or down, along with the camera

Working:
- 3 texture-mapped robots (number increases every level) walking around with animations, firing projectiles at the defensive cannon
- Texture-mapped defensive cannon firing projectiles using the space key
- Projectiles rendered and animated
- Successful detection of collisions between defensive cannon projectiles and enemy robots, and projectiles hitting the defensive cannon
- Enemy robots shrink and dissapear when hit
- Defensive cannon fades to black and no longer works when hit, ability to hit "r" to reset
- Mouse-controlled defensive cannon and camera
Bonuses implemented:
- Enemy robots and defensive cannon shoot lasers
- New levels with a score updated when all robots are shot, each level adds a new enemy robot that will shoot the defensive cannon

Not implemented:
- Custom mesh generated via A2
- Vertex/Fragment shaders to render the scene, only texture mapping included
