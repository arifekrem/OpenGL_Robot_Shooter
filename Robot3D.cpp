/*******************************************************************
  Hierarchical Multi-Part Model Example
********************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <gl/glut.h>
#include <utility>
#include <vector>
#include "VECTOR3D.h"
#include "QuadMesh.h"
#define M_PI 3.14159265358979323846

const int vWidth = 650;    // Viewport width in pixels
const int vHeight = 500;    // Viewport height in pixels

// Note how everything depends on robot body dimensions so that can scale entire robot proportionately
// just by changing robot body scale
float robotBodyWidth = 12.0;
float robotBodyLength = 10.0;
float robotBodyDepth = 8.0;
float headWidth = 0.3 * robotBodyWidth;
float headLength = headWidth;
float headDepth = headWidth;
float upperArmLength = robotBodyLength;
float upperArmWidth = 0.2 * robotBodyWidth;
float gunWidth = upperArmWidth;
float gunLength = upperArmLength / 2.0;
float gunDepth = upperArmWidth;
float stanchionLength = robotBodyLength;
float stanchionRadius = 0.1 * robotBodyDepth;
float baseWidth = 1.5 * robotBodyWidth;
float baseLength = 0.4 * stanchionLength;
float legAngle = 0.0f;        // Controls the angle of the leg during stepping
bool spinCannon = false;      // Flag to control cannon spinning
float cannonSpinAngle = 0.0f; // Angle for cannon spinning

// Joint angles for walking
float hipAngleLeft = 0.0f;  // Angle for left hip joint
float kneeAngleLeft = 0.0f; // Angle for left knee joint
float ankleAngleLeft = 0.0f; // Angle for left ankle joint
float hipAngleRight = 0.0f;  // Angle for right hip joint
float kneeAngleRight = 0.0f; // Angle for right knee joint
float ankleAngleRight = 0.0f; // Angle for right ankle joint
float lowerLegAngleLeft = 0.0f;  // Angle for rotating lower part of the left leg
float lowerLegAngleRight = 0.0f; // Angle for rotating lower part of the right leg

// Flag to control walking state
bool walking = false;
bool stepBackwards = false;  // Controls whether the leg is stepping forward or backward
bool walkingForward = true;   // Track whether we're walking forward
int selectedJoint = 0; // 0 for none, 1 for knee, 2 for hip, 3 for body
int cameraView = 0; // 0 = default, 1 = front, 2 = side, 3 = top-down

// Control Robot body rotation on base
float robotAngle = 0.0;
float neckAngle = 0.0f;   // Neck rotation

// Control arm rotation
float shoulderAngle = -40.0;
float gunAngle = -25.0;

// Variable to control the position offset along the direction of movement
float robotZOffset = 0.0f; // Movement offset along the z-axis
bool movingForward = true; // Flag to control movement direction

float barrelYawAngle = 0.0f;  // Rotation left/right
float barrelTiltAngle = 0.0f; // Tilt up/down

float cameraX = 0.0f, cameraY = 15.0f, cameraZ = 100.0f; // Initial camera position
float cameraYaw = 0.0f;   // Horizontal rotation
float cameraPitch = 0.0f; // Vertical rotation
float cameraDistance = 100.0f; // Distance from the target (cannon)
bool cannonDisabled = false; // Initially, the cannon is enabled
float cannonFadeProgress = 0.0f;      // Progress of the cannon fade to black (0.0 to 1.0)
float cannonColor[4] = { 0.0f, 1.0f, 0.0f, 1.0f }; // Initial green RGBA for the cannon

// Lighting/shading and material properties for robot - upcoming lecture - just copy for now
// Robot RGBA material properties (NOTE: we will learn about this later in the semester)
GLfloat robotBody_mat_ambient[] = { 0.0f,0.0f,0.0f,1.0f };
GLfloat robotBody_mat_specular[] = { 0.45f,0.55f,0.45f,1.0f };
GLfloat robotBody_mat_diffuse[] = { 0.8f, 0.7f, 0.5f, 1.0f };
GLfloat robotBody_mat_shininess[] = { 32.0F };

GLfloat robotArm_mat_ambient[] = { 0.0215f, 0.1745f, 0.0215f, 0.55f };
GLfloat robotArm_mat_diffuse[] = { 0.4f, 0.5f, 0.2f, 1.0f };
GLfloat robotArm_mat_specular[] = { 0.7f, 0.6f, 0.6f, 1.0f };
GLfloat robotArm_mat_shininess[] = { 32.0F };

GLfloat dark_grey_ambient[] = { 0.1f, 0.1f, 0.1f, 1.0f };
GLfloat dark_grey_diffuse[] = { 0.15f, 0.15f, 0.15f, 1.0f };
GLfloat dark_grey_specular[] = { 0.2f, 0.2f, 0.2f, 1.0f };
GLfloat dark_grey_shininess[] = { 50.0f };

GLfloat robotLowerBody_mat_ambient[] = { 0.25f, 0.25f, 0.25f, 1.0f };
GLfloat robotLowerBody_mat_diffuse[] = { 0.4f, 0.4f, 0.4f, 1.0f };
GLfloat robotLowerBody_mat_specular[] = { 0.774597f, 0.774597f, 0.774597f, 1.0f };
GLfloat robotLowerBody_mat_shininess[] = { 76.8F };

GLfloat green_mat_ambient[] = { 0.02f, 0.15f, 0.02f, 1.0f };
GLfloat green_mat_diffuse[] = { 0.05f, 0.2f, 0.05f, 0.1f };
GLfloat green_mat_specular[] = { 0.2f, 0.2f, 0.2f, 1.0f };
GLfloat green_mat_shininess[] = { 100.0f };

GLfloat beige_mat_ambient[] = { 0.6f, 0.5f, 0.3f, 1.0f };
GLfloat beige_mat_specular[] = { 0.1f, 0.1f, 0.1f, 1.0f };
GLfloat beige_mat_diffuse[] = { 0.7f, 0.6f, 0.4f, 1.0f };
GLfloat beige_mat_shininess[] = { 30.0f };

GLfloat light_brown_mat_ambient[] = { 0.3f, 0.2f, 0.1f, 1.0f };
GLfloat light_brown_mat_specular[] = { 0.1f, 0.1f, 0.1f, 1.0f };
GLfloat light_brown_mat_diffuse[] = { 0.4f, 0.3f, 0.2f, 1.0f };
GLfloat light_brown_mat_shininess[] = { 30.0f };

GLfloat red_orange_ambient[] = { 0.8f, 0.2f, 0.0f, 1.0f };
GLfloat red_orange_diffuse[] = { 0.9f, 0.3f, 0.1f, 1.0f };
GLfloat red_orange_specular[] = { 0.8f, 0.2f, 0.1f, 1.0f };
GLfloat red_orange_shininess[] = { 32.0F };

// Light properties
GLfloat light_position0[] = { -4.0F, 8.0F, 8.0F, 1.0F };
GLfloat light_position1[] = { 4.0F, 8.0F, 8.0F, 1.0F };
GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat light_ambient[] = { 0.2F, 0.2F, 0.2F, 1.0F };

GLfloat neon_green_ambient[] = { 0.0f, 0.8f, 0.0f, 1.0f };
GLfloat neon_green_diffuse[] = { 0.0f, 1.0f, 0.0f, 1.0f };
GLfloat neon_green_specular[] = { 0.5f, 1.0f, 0.5f, 1.0f };
GLfloat neon_green_shininess[] = { 100.0f };

// Mouse button
int currentButton;

// A flat open mesh
QuadMesh* groundMesh = NULL;

// Structure defining a bounding box, currently unused
typedef struct BoundingBox {
	VECTOR3D min;
	VECTOR3D max;
} BBox;

struct Robot {
	float xOffset;
	float zOffset;
	float speed;
	float direction;
	bool disabled;
	int breakingTimer;
};

struct Projectile {
	float x, y, z;     // Position
	float speed;       // Speed of the projectile
	float directionX, directionY, directionZ; // Direction vector
	bool active;       // Is the projectile currently active?
};

const int maxProjectiles = 10; // Max number of projectiles
Projectile projectiles[maxProjectiles];

std::vector<Projectile> defensiveProjectiles; // For defensive cannon projectiles

float spacing = 20.0f;
const int numRobots = 3;
Robot robots[numRobots];

// Default Mesh Size
int meshSize = 16;

// Prototypes for functions in this module
void initOpenGL(int w, int h);
void display(void);
void reshape(int w, int h);
void handleMouseClick(int button, int state, int x, int y);
void handleMouseMotion(int x, int y);
void keyboard(unsigned char key, int x, int y);
void functionKeys(int key, int x, int y);
void animationHandler(int param);
void drawRobot();
void drawBody();
void drawHead();
void drawLowerBody();
void drawLeftArm();
void drawRightArm();
void moveRobots(int value);
void stepAnimation(int value);
void initializeRobots();
void initializeEnemyProjectiles();
void fireEnemyProjectile(float startX, float startY, float startZ, float targetX, float targetY, float targetZ);
void fireRandomEnemyProjectiles(int value);
void updateEnemyProjectiles(int value);
void drawEnemyProjectiles();
VECTOR3D getCannonWorldPosition(Robot robot);
void drawDefensiveCannon();
void fireDefensiveCannonProjectile();
void updateDefensiveProjectiles();
void drawDefensiveProjectiles();
void checkCannonHit();
void updateDefensiveProjectilesTimer(int value);
bool detectDefensiveLaserCollisionWithBot(const Projectile& defensiveLaser, const Robot& enemyBot);
void drawRobotWithBreakingAnimation(const Robot& enemyBot);
void updateCannonFade(int value);

int main(int argc, char** argv)
{
	// Initialize GLUT
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(vWidth, vHeight);
	glutInitWindowPosition(200, 30);
	glutCreateWindow("3D Hierarchical Example");

	// Initialize robots & projectiles
	initializeRobots();
	initializeEnemyProjectiles();

	// Initialize GL
	initOpenGL(vWidth, vHeight);

	// Register callback functions
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutMouseFunc(handleMouseClick);
	glutPassiveMotionFunc(handleMouseMotion);
	glutKeyboardFunc(keyboard);
	glutSpecialFunc(functionKeys);

	// Start movement animations
	walking = true;  // Start walking animation automatically
	glutTimerFunc(16, moveRobots, 0);  // Start robot movement animation
	glutTimerFunc(10, stepAnimation, 0);  // Start walking animation
	glutTimerFunc(16, updateEnemyProjectiles, 0);  // Start updating all projectiles
	glutTimerFunc(16, updateDefensiveProjectilesTimer, 0);
	glutTimerFunc(500, fireRandomEnemyProjectiles, 0);

	// Start event loop, never returns
	glutMainLoop();

	return 0;
}

void initializeRobots() {
	for (int i = 0; i < numRobots; i++) {
		robots[i].xOffset = (i - (numRobots - 1) * 0.5f) * 20.0f; // Spaced along x-axis
		robots[i].zOffset = 0.0f; // Start at the same z-axis position
		robots[i].speed = 0.1f + 0.05f * i; // Slightly different speeds
		robots[i].direction = 0.0f; // All initially walking straight
		robots[i].disabled = false; // Not disabled initially
		robots[i].breakingTimer = 0; // No animation initially
	}
}

void initializeEnemyProjectiles() {
	for (int i = 0; i < maxProjectiles; i++) {
		projectiles[i].active = false; // Initially, all projectiles are inactive
	}
}

void fireEnemyProjectile(float startX, float startY, float startZ, float camDirX, float camDirY, float camDirZ) {
	for (int i = 0; i < maxProjectiles; i++) {
		if (!projectiles[i].active) {
			// Set the initial position of the projectile
			projectiles[i].x = startX;
			projectiles[i].y = startY;
			projectiles[i].z = startZ;

			// Add slight random inaccuracy to the camera direction
			float inaccuracy = 0.1f; // Degree of inaccuracy
			float dirX = camDirX + (rand() % 100 / 500.0f - inaccuracy); // Random offset
			float dirY = camDirY + (rand() % 100 / 500.0f - inaccuracy);
			float dirZ = camDirZ + (rand() % 100 / 500.0f - inaccuracy);

			// Negate the direction to ensure the projectiles move towards the front view camera
			dirX = -dirX;
			dirY = -dirY;
			dirZ = -dirZ;

			// Normalize the direction vector
			float magnitude = sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ);
			if (magnitude > 0.0f) {
				projectiles[i].directionX = dirX / magnitude;
				projectiles[i].directionY = dirY / magnitude;
				projectiles[i].directionZ = dirZ / magnitude;
			}
			else {
				// Default fallback direction if magnitude is zero
				projectiles[i].directionX = 0.0f;
				projectiles[i].directionY = 0.0f;
				projectiles[i].directionZ = -1.0f;
			}

			projectiles[i].speed = 0.5f; // Set speed for the projectile
			projectiles[i].active = true;

			break; // Use only one projectile at a time
		}
	}
}

bool detectDefensiveLaserCollisionWithBot(const Projectile& defensiveLaser, const Robot& enemyBot) {
	float botRadius = robotBodyWidth * 0.5f; // Approximate robot as a circle
	float laserRadius = 0.5f; // Approximate laser size

	float dx = defensiveLaser.x - enemyBot.xOffset;
	float dz = defensiveLaser.z - enemyBot.zOffset;

	float distance = sqrt(dx * dx + dz * dz);

	return distance < (botRadius + laserRadius);
}

VECTOR3D getCannonWorldPosition(Robot robot) {
	float rad = robot.direction * (M_PI / 180.0f);
	float baseX = robot.xOffset;
	float baseZ = robot.zOffset;

	// Body rotation
	float bodyRotationRad = robotAngle * (M_PI / 180.0f);

	// Cannon's position relative to the body
	float cannonLocalX = -(0.5 * robotBodyWidth + gunWidth);
	float cannonLocalY = 0.3 * robotBodyLength;
	float cannonLocalZ = 0.5 * robotBodyDepth;

	// Apply body rotation
	float cannonRotatedX = cannonLocalX * cos(bodyRotationRad) - cannonLocalZ * sin(bodyRotationRad);
	float cannonRotatedZ = cannonLocalX * sin(bodyRotationRad) + cannonLocalZ * cos(bodyRotationRad);

	// Compute world position
	float cannonWorldX = baseX + cannonRotatedX;
	float cannonWorldY = cannonLocalY;
	float cannonWorldZ = baseZ + cannonRotatedZ;

	return VECTOR3D(cannonWorldX, cannonWorldY, cannonWorldZ);
}

void fireRandomEnemyProjectiles(int value) {
	float projectileDirX = 0.0f, projectileDirY = 0.0f, projectileDirZ = -1.0f;

	for (int i = 0; i < numRobots; i++) {
		if (robots[i].disabled) {
			continue; // Skip disabled robots
		}

		if (rand() % 100 < 50) { // Increase chance to fire from 20% to 50%
			VECTOR3D cannonWorldPos = getCannonWorldPosition(robots[i]);
			fireEnemyProjectile(
				cannonWorldPos.x, cannonWorldPos.y, cannonWorldPos.z,
				projectileDirX, projectileDirY, projectileDirZ
			);
		}
	}

	glutTimerFunc(200, fireRandomEnemyProjectiles, 0); // Lower interval to 200ms
}

void updateEnemyProjectiles(int value) {
	for (int i = 0; i < maxProjectiles; i++) {
		if (projectiles[i].active) {
			projectiles[i].x += projectiles[i].directionX * projectiles[i].speed;
			projectiles[i].y += projectiles[i].directionY * projectiles[i].speed;
			projectiles[i].z += projectiles[i].directionZ * projectiles[i].speed;

			// Deactivate if projectile goes out of bounds
			if (projectiles[i].z < -100 || projectiles[i].z > 100 ||
				projectiles[i].x < -100 || projectiles[i].x > 100 ||
				projectiles[i].y < -10 || projectiles[i].y > 50) {
				projectiles[i].active = false;
			}
		}
	}
	glutPostRedisplay();
	glutTimerFunc(16, updateEnemyProjectiles, 0);
}

void updateDefensiveProjectilesTimer(int value) {
	updateDefensiveProjectiles();  // Update positions of defensive projectiles
	glutPostRedisplay();           // Trigger a redraw
	glutTimerFunc(16, updateDefensiveProjectilesTimer, 0);  // Schedule the next update
}

void drawEnemyProjectiles() {
	for (int i = 0; i < maxProjectiles; i++) {
		if (projectiles[i].active) {
			glPushMatrix();

			// Position the projectile
			glTranslatef(projectiles[i].x, projectiles[i].y, projectiles[i].z);

			// Calculate the projectile's direction vector
			VECTOR3D direction(
				projectiles[i].directionX,
				projectiles[i].directionY,
				projectiles[i].directionZ
			);

			// Normalize the direction vector
			float magnitude = sqrt(direction.x * direction.x +
				direction.y * direction.y +
				direction.z * direction.z);
			if (magnitude > 0.0f) {
				direction.x /= magnitude;
				direction.y /= magnitude;
				direction.z /= magnitude;
			}

			// Define an up vector and calculate the right vector
			VECTOR3D up(0.0f, 1.0f, 0.0f);
			if (fabs(direction.x) < 1e-6 && fabs(direction.z) < 1e-6) {
				up = VECTOR3D(1.0f, 0.0f, 0.0f); // Prevent degenerate up vector
			}
			VECTOR3D right = cross(up, direction);
			right = normalize(right);
			up = cross(direction, right);

			// Create a rotation matrix to orient the projectile
			float rotationMatrix[16] = {
				right.x,   right.y,   right.z,   0.0f,
				up.x,      up.y,      up.z,      0.0f,
				-direction.x, -direction.y, -direction.z, 0.0f,
				0.0f,      0.0f,      0.0f,      1.0f
			};

			// Apply the rotation matrix
			glMultMatrixf(rotationMatrix);

			// Set material properties for enemy projectiles (red)
			glMaterialfv(GL_FRONT, GL_AMBIENT, red_orange_ambient);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, red_orange_diffuse);
			glMaterialfv(GL_FRONT, GL_SPECULAR, red_orange_specular);
			glMaterialfv(GL_FRONT, GL_SHININESS, red_orange_shininess);

			// Draw the projectile as a cylinder
			GLUquadric* quad = gluNewQuadric();
			gluCylinder(quad, 0.1f, 0.1f, 3.0f, 16, 16); // Thin, elongated cylinder
			gluDeleteQuadric(quad);

			glPopMatrix();
		}
	}
}

void drawRobotWithBreakingAnimation(const Robot& enemyBot) {
	if (enemyBot.disabled && enemyBot.breakingTimer <= 0) {
		return; // Don't render robots that have finished breaking animation
	}

	if (enemyBot.disabled) {
		// Breaking animation: Shrink the bot over time
		glPushMatrix();
		glTranslatef(enemyBot.xOffset, 0.0f, enemyBot.zOffset);
		float scale = enemyBot.breakingTimer / 50.0f; // Shrink over time
		glScalef(scale, scale, scale);
		drawRobot(); // Draw robot body in reduced size
		glPopMatrix();
		return;
	}

	// Normal bot drawing
	glPushMatrix();
	glTranslatef(enemyBot.xOffset, 0.0, enemyBot.zOffset);
	drawRobot();
	glPopMatrix();
}

void drawDefensiveCannon() {
	// If the cannon is fully disabled and fully black, render it as black
	glPushMatrix();

	// Adjust position based on camera's position and direction
	float cannonOffsetDistance = 10.0f; // Distance in front of the camera
	float offsetX = sin(cameraYaw * M_PI / 180.0) * cos(cameraPitch * M_PI / 180.0);
	float offsetY = sin(cameraPitch * M_PI / 180.0);
	float offsetZ = -cos(cameraYaw * M_PI / 180.0) * cos(cameraPitch * M_PI / 180.0);

	float cannonX = cameraX + offsetX * cannonOffsetDistance;
	float cannonY = cameraY + offsetY * cannonOffsetDistance - 5.0f; // Slightly lower than the camera
	float cannonZ = cameraZ + offsetZ * cannonOffsetDistance;

	glTranslatef(cannonX, cannonY, cannonZ);

	// Align the cannon with the camera's orientation
	glRotatef(-cameraYaw, 0.0, 1.0, 0.0);  // Horizontal alignment
	glRotatef(cameraPitch, 1.0, 0.0, 0.0); // Vertical alignment

	// Base material: Black if fully disabled
	GLfloat blackAmbient[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
	GLfloat blackDiffuse[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
	GLfloat blackSpecular[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
	GLfloat blackShininess = 0.0f;

	if (cannonDisabled && cannonFadeProgress >= 1.0f) {
		// Apply fully black material
		glMaterialfv(GL_FRONT, GL_AMBIENT, blackAmbient);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, blackDiffuse);
		glMaterialfv(GL_FRONT, GL_SPECULAR, blackSpecular);
		glMaterialfv(GL_FRONT, GL_SHININESS, &blackShininess);
	}
	else {
		// Transition to black based on fade progress
		GLfloat currentBaseAmbient[4], currentBaseDiffuse[4], currentBaseSpecular[4];
		for (int i = 0; i < 4; i++) {
			currentBaseAmbient[i] = green_mat_ambient[i] * (1.0f - cannonFadeProgress);
			currentBaseDiffuse[i] = green_mat_diffuse[i] * (1.0f - cannonFadeProgress);
			currentBaseSpecular[i] = green_mat_specular[i] * (1.0f - cannonFadeProgress);
		}
		glMaterialfv(GL_FRONT, GL_AMBIENT, currentBaseAmbient);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, currentBaseDiffuse);
		glMaterialfv(GL_FRONT, GL_SPECULAR, currentBaseSpecular);
		glMaterialfv(GL_FRONT, GL_SHININESS, green_mat_shininess);
	}

	// Draw cannon base (scaled 2x)
	glPushMatrix();
	glScalef(6.0, 3.0, 6.0); // Base scaled to twice the original size
	glutSolidCube(1.0);
	glPopMatrix();

	// Barrel material
	if (cannonDisabled && cannonFadeProgress >= 1.0f) {
		// Apply fully black material for the barrel as well
		glMaterialfv(GL_FRONT, GL_AMBIENT, blackAmbient);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, blackDiffuse);
		glMaterialfv(GL_FRONT, GL_SPECULAR, blackSpecular);
		glMaterialfv(GL_FRONT, GL_SHININESS, &blackShininess);
	}
	else {
		glMaterialfv(GL_FRONT, GL_AMBIENT, dark_grey_ambient);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, dark_grey_diffuse);
		glMaterialfv(GL_FRONT, GL_SPECULAR, dark_grey_specular);
		glMaterialfv(GL_FRONT, GL_SHININESS, dark_grey_shininess);
	}

	// Draw cannon barrel (scaled 2x)
	glPushMatrix();
	glTranslatef(0.0, 1.0, -4.0); // Adjust barrel position to fit the larger base
	gluCylinder(gluNewQuadric(), 1.0, 1.0, 10.0, 16, 16); // Barrel scaled up
	glPopMatrix();

	glPopMatrix();
}

void fireDefensiveCannonProjectile() {
	if (cannonDisabled) return; // Skip if the cannon is disabled

	// Calculate cannon position
	float cannonOffsetDistance = 10.0f; // Distance in front of the camera
	float offsetX = sin(cameraYaw * M_PI / 180.0f) * cos(cameraPitch * M_PI / 180.0f);
	float offsetY = sin(cameraPitch * M_PI / 180.0f);
	float offsetZ = -cos(cameraYaw * M_PI / 180.0f) * cos(cameraPitch * M_PI / 180.0f);

	float cannonX = cameraX + offsetX * cannonOffsetDistance;
	float cannonY = cameraY + offsetY * cannonOffsetDistance - 5.0f;
	float cannonZ = cameraZ + offsetZ * cannonOffsetDistance;

	// Calculate projectile direction
	float dirX = offsetX;
	float dirY = offsetY;
	float dirZ = offsetZ;

	// Normalize the direction vector
	float magnitude = sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ);
	if (magnitude > 0.0f) {
		dirX /= magnitude;
		dirY /= magnitude;
		dirZ /= magnitude;
	}

	// Create new projectile
	Projectile newProjectile;
	newProjectile.x = cannonX;
	newProjectile.y = cannonY;
	newProjectile.z = cannonZ;
	newProjectile.directionX = dirX;
	newProjectile.directionY = dirY;
	newProjectile.directionZ = dirZ;
	newProjectile.speed = 1.0f; // Speed of the projectile
	newProjectile.active = true;

	defensiveProjectiles.push_back(newProjectile); // Add projectile to list
}

void updateDefensiveProjectiles() {
	for (size_t i = 0; i < defensiveProjectiles.size(); ++i) {
		if (defensiveProjectiles[i].active) {
			// Update projectile position
			defensiveProjectiles[i].x += defensiveProjectiles[i].directionX * defensiveProjectiles[i].speed;
			defensiveProjectiles[i].y += defensiveProjectiles[i].directionY * defensiveProjectiles[i].speed;
			defensiveProjectiles[i].z += defensiveProjectiles[i].directionZ * defensiveProjectiles[i].speed;

			// Check for collisions with enemy robots
			for (int j = 0; j < numRobots; j++) {
				if (!robots[j].disabled &&
					detectDefensiveLaserCollisionWithBot(defensiveProjectiles[i], robots[j])) {
					// Handle collision
					robots[j].disabled = true; // Disable the enemy robot
					robots[j].breakingTimer = 50; // Trigger breaking animation
					defensiveProjectiles[i].active = false; // Deactivate projectile
					break; // Handle only one collision per projectile
				}
			}

			// Deactivate projectiles that go out of bounds
			if (fabs(defensiveProjectiles[i].x) > 100 ||
				fabs(defensiveProjectiles[i].y) > 50 ||
				fabs(defensiveProjectiles[i].z) > 100) {
				defensiveProjectiles[i].active = false;
			}
		}
	}

	// Remove inactive projectiles
	defensiveProjectiles.erase(
		std::remove_if(defensiveProjectiles.begin(), defensiveProjectiles.end(),
			[](const Projectile& p) { return !p.active; }),
		defensiveProjectiles.end());
}

void drawDefensiveProjectiles() {
	for (const auto& proj : defensiveProjectiles) {
		if (proj.active) {
			glPushMatrix();

			// Position the projectile
			glTranslatef(proj.x, proj.y, proj.z);

			// Calculate the projectile's direction vector
			VECTOR3D direction(
				proj.directionX,
				proj.directionY,
				proj.directionZ
			);

			// Normalize the direction vector
			float magnitude = sqrt(direction.x * direction.x +
				direction.y * direction.y +
				direction.z * direction.z);
			if (magnitude > 0.0f) {
				direction.x /= magnitude;
				direction.y /= magnitude;
				direction.z /= magnitude;
			}

			// Define an up vector and calculate the right vector
			VECTOR3D up(0.0f, 1.0f, 0.0f);
			if (fabs(direction.x) < 1e-6 && fabs(direction.z) < 1e-6) {
				up = VECTOR3D(1.0f, 0.0f, 0.0f); // Prevent degenerate up vector
			}
			VECTOR3D right = cross(up, direction);
			right = normalize(right);
			up = cross(direction, right);

			// Create a rotation matrix to orient the projectile
			float rotationMatrix[16] = {
				right.x,   right.y,   right.z,   0.0f,
				up.x,      up.y,      up.z,      0.0f,
				-direction.x, -direction.y, -direction.z, 0.0f,
				0.0f,      0.0f,      0.0f,      1.0f
			};

			// Apply the rotation matrix
			glMultMatrixf(rotationMatrix);

			// Set material properties for defensive projectiles (neon green)
			glMaterialfv(GL_FRONT, GL_AMBIENT, neon_green_ambient);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, neon_green_diffuse);
			glMaterialfv(GL_FRONT, GL_SPECULAR, neon_green_specular);
			glMaterialfv(GL_FRONT, GL_SHININESS, neon_green_shininess);

			// Draw the projectile as a neon green cylinder
			GLUquadric* quad = gluNewQuadric();
			gluCylinder(quad, 0.1f, 0.1f, 3.0f, 16, 16); // Thin, elongated cylinder
			gluDeleteQuadric(quad);

			glPopMatrix();
		}
	}
}

void checkCannonHit() {
	for (int i = 0; i < maxProjectiles; i++) {
		if (projectiles[i].active) {
			// Adjust collision bounds for the larger cannon
			if (fabs(projectiles[i].x - cameraX) < 4.0f &&   // X-axis range doubled
				fabs(projectiles[i].y - (cameraY - 5.0f)) < 2.0f && // Y-axis range doubled
				fabs(projectiles[i].z - (cameraZ - 10.0f)) < 2.0f) { // Z-axis range doubled
				cannonDisabled = true;  // Disable cannon
				glutTimerFunc(16, updateCannonFade, 0); // Start fading animation
				projectiles[i].active = false; // Deactivate the projectile
			}
		}
	}
}

void updateCannonFade(int value) {
	if (cannonDisabled && cannonFadeProgress < 1.0f) {
		// Increment fade progress
		cannonFadeProgress += 0.01f; // Adjust for slower or faster fading

		// Interpolate RGBA values to fade to black
		for (int i = 0; i < 3; i++) {
			cannonColor[i] = (1.0f - cannonFadeProgress) * green_mat_diffuse[i];
		}
		cannonColor[3] = 1.0f; // Maintain full opacity

		glutPostRedisplay();
		glutTimerFunc(16, updateCannonFade, 0); // Call this function every 16ms (60 FPS)
	}
}

// Set up OpenGL. For viewport and projection setup see reshape().
void initOpenGL(int w, int h)
{
	// Set up and enable lighting
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);

	glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
	glLightfv(GL_LIGHT1, GL_POSITION, light_position1);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);   // This second light is currently off

	// Other OpenGL setup
	glEnable(GL_DEPTH_TEST);   // Remove hidded surfaces
	glShadeModel(GL_SMOOTH);   // Use smooth shading, makes boundaries between polygons harder to see
	glClearColor(0.4F, 0.4F, 0.4F, 0.0F);  // Color and depth for glClear
	glClearDepth(1.0f);
	glEnable(GL_NORMALIZE);    // Renormalize normal vectors
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);   // Nicer perspective

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();


	// Other initializatuion
	// Set up ground quad mesh
	VECTOR3D origin = VECTOR3D(-160.0f, 0.0f, 160.0f); // Expand origin to match 10x scale
	VECTOR3D dir1v = VECTOR3D(1.0f, 0.0f, 0.0f);
	VECTOR3D dir2v = VECTOR3D(0.0f, 0.0f, -1.0f);
	groundMesh = new QuadMesh(meshSize, 3200.0); // Update mesh size to 3200.0 for 10x scale
	groundMesh->InitMesh(meshSize, origin, 3200.0, 3200.0, dir1v, dir2v); // 10x larger dimensions

	VECTOR3D ambient = VECTOR3D(0.0f, 0.05f, 0.0f);
	VECTOR3D diffuse = VECTOR3D(0.4f, 0.8f, 0.4f);
	VECTOR3D specular = VECTOR3D(0.04f, 0.04f, 0.04f);
	float shininess = 0.2;
	groundMesh->SetMaterial(ambient, diffuse, specular, shininess);

}

void display(void) {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	// Calculate the camera position and direction
	float camDirX = sin(cameraYaw * M_PI / 180.0) * cos(cameraPitch * M_PI / 180.0);
	float camDirY = sin(cameraPitch * M_PI / 180.0);
	float camDirZ = -cos(cameraYaw * M_PI / 180.0) * cos(cameraPitch * M_PI / 180.0);

	float cameraTargetX = cameraX + camDirX;
	float cameraTargetY = cameraY + camDirY;
	float cameraTargetZ = cameraZ + camDirZ;

	// Set the camera's view
	gluLookAt(cameraX, cameraY, cameraZ, cameraTargetX, cameraTargetY, cameraTargetZ, 0.0f, 1.0f, 0.0f);

	// Check if the cannon is hit and start fading animation if necessary
	checkCannonHit();
	if (cannonDisabled && cannonFadeProgress < 1.0f) {
		glutTimerFunc(16, updateCannonFade, 0); // Start fading animation
	}

	// Draw robots
	for (int i = 0; i < numRobots; i++) {
		if (robots[i].disabled) {
			drawRobotWithBreakingAnimation(robots[i]); // Draw breaking animation for disabled robots
		}
		else {
			glPushMatrix();
			glTranslatef(robots[i].xOffset, 0.0, robots[i].zOffset);
			drawRobot(); // Draw active robots normally
			glPopMatrix();
		}
	}

	// Draw defensive projectiles
	drawDefensiveProjectiles();

	// Draw enemy projectiles
	drawEnemyProjectiles();

	// Draw defensive cannon
	drawDefensiveCannon();

	// Draw ground mesh
	glPushMatrix();
	glTranslatef(0.0, -25.0, 0.0); // Lower the ground slightly
	groundMesh->DrawMesh(meshSize);
	glPopMatrix();

	// Additional rendering elements (if any)
	// Add any further components to be rendered here, if needed.

	glutSwapBuffers();
}

void drawRobot()
{
	// 1. Draw the lower body (green part and legs) separately with no rotation
	glPushMatrix();  // Save current matrix state
	drawLowerBody();  // Draw the lower body first without any transformations
	glPopMatrix();  // Restore the matrix

	// 2. Draw the upper body with rotation
	glPushMatrix();  // Save current matrix state

	// Rotate only the upper body parts
	glRotatef(robotAngle, 0.0, 1.0, 0.0);  // Apply rotation for the upper body

	// Draw the upper body components: torso, head, arms
	drawBody();      // Beige, black parts (upper torso)
	drawHead();      // Head
	drawLeftArm();   // Left arm
	drawRightArm();  // Right arm

	glPopMatrix();  // End upper body rotation, restore the matrix
}

void drawBody()
{
	// Top Part (Beige, wide)
	glPushMatrix();
	// Set material properties for the beige top part
	glMaterialfv(GL_FRONT, GL_AMBIENT, beige_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, beige_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, beige_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, beige_mat_shininess);

	// Position and scale the top part
	glTranslatef(0.0, 0.5 * robotBodyLength, 0.0);  // Top part is at the top
	glScalef(robotBodyWidth, robotBodyLength / 3.0, robotBodyDepth);  // Wide part, 1/3 of total height
	glutSolidCube(1.0);
	glPopMatrix();

	// Middle Part (Black, thin and long)
	glPushMatrix();
	// Set material properties for the black middle part
	glMaterialfv(GL_FRONT, GL_AMBIENT, dark_grey_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, dark_grey_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, dark_grey_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, dark_grey_shininess);

	// Position and scale the middle part
	glTranslatef(0.0, 0.0, 0.0);  // Middle part stays in the center
	glScalef(0.4 * robotBodyWidth, robotBodyLength / 2.0, 0.4 * robotBodyDepth);  // Thin and long part
	glutSolidCube(1.0);
	glPopMatrix();

	// No longer drawing the green bottom part here; it will now be drawn as part of the lower body.
}

void drawHead()
{
	// Set robot material properties for the head (white part)
	GLfloat white_ambient[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat white_diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat white_specular[] = { 0.5f, 0.5f, 0.5f, 1.0f };
	GLfloat white_shininess[] = { 50.0f };

	GLfloat cyan_ambient[] = { 0.0f, 1.0f, 1.0f, 1.0f };
	GLfloat cyan_diffuse[] = { 0.0f, 1.0f, 1.0f, 1.0f };
	GLfloat cyan_specular[] = { 0.1f, 0.1f, 0.1f, 1.0f };
	GLfloat cyan_shininess[] = { 30.0f };

	glMaterialfv(GL_FRONT, GL_AMBIENT, dark_grey_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, dark_grey_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, dark_grey_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, dark_grey_shininess);

	glPushMatrix();
	// Apply neck rotation
	glRotatef(neckAngle, 0.0, 1.0, 0.0);  // Rotate neck along Y-axis
	glTranslatef(0, 0.5 * robotBodyLength + 1.0 * headLength, 0); // Move head above the body

	// Draw the head (white part)
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, white_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, white_diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, white_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, white_shininess);

	glScalef(0.4 * robotBodyWidth, 0.4 * robotBodyWidth, 0.4 * robotBodyWidth);
	glutSolidCube(1.0);
	glPopMatrix();

	glMaterialfv(GL_FRONT, GL_AMBIENT, green_mat_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, green_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, green_mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, green_mat_shininess);

	// Draw left side of the head
	glPushMatrix();
	glTranslatef(-0.2 * robotBodyWidth, 0, 0);  // Move to left
	glScalef(0.01 * robotBodyWidth, 0.4 * robotBodyWidth, 0.4 * robotBodyWidth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Draw right side of the head
	glPushMatrix();
	glTranslatef(0.2 * robotBodyWidth, 0, 0);  // Move to right
	glScalef(0.01 * robotBodyWidth, 0.4 * robotBodyWidth, 0.4 * robotBodyWidth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Grey stripe positioned just behind the blue and in front of the white, move very slightly down
	glMaterialfv(GL_FRONT, GL_AMBIENT, dark_grey_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, dark_grey_diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, dark_grey_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, dark_grey_shininess);

	// Move the front grey part very slightly down
	glPushMatrix();
	glTranslatef(0.0, 0.06 * robotBodyWidth, 0.20 * robotBodyWidth);  // Very slight downward adjustment
	glScalef(0.12 * robotBodyWidth, 0.3 * robotBodyWidth, 0.03 * robotBodyWidth);  // Taller and wider
	glutSolidCube(1.0);
	glPopMatrix();

	// Add grey part to the top of the head
	glPushMatrix();
	glTranslatef(0.0, 0.2 * robotBodyWidth, 0.01 * robotBodyWidth);  // Small forward adjustment
	glScalef(0.12 * robotBodyWidth, 0.02 * robotBodyWidth, 0.42 * robotBodyWidth);  // Long and thin grey stripe on top
	glutSolidCube(1.0);
	glPopMatrix();

	// Blue eye (upright visor-like stripe)
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, cyan_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, cyan_diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, cyan_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, cyan_shininess);

	// Position the blue visor/eye on the front
	glTranslatef(0.0, 0.1 * robotBodyWidth, 0.22 * robotBodyWidth);  // Position it on the front
	glScalef(0.05 * robotBodyWidth, 0.2 * robotBodyWidth, 0.02 * robotBodyWidth);  // Flip dimensions to make it upright
	glutSolidCube(1.0); // Blue eye
	glPopMatrix();

	glPopMatrix();  // End head drawing
}

void drawLowerBody()
{
	// Lower body green section (stationary part)
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, green_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, green_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, green_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, green_mat_shininess);

	// Position the green section (between the legs) under the upper body but not affected by rotation
	glTranslatef(0.0, -0.5 * robotBodyLength, 0.0);  // Move to where the green section is positioned
	glScalef(0.8 * robotBodyWidth, robotBodyLength / 3.0, 0.8 * robotBodyDepth);  // Scale to match the body proportions
	glutSolidCube(1.0);  // Draw the green section (stationary)
	glPopMatrix();

	// Left leg
	glPushMatrix();
	// Move the leg lower to connect better to the body
	glTranslatef(0.5 * robotBodyWidth, -0.7 * robotBodyLength, 0.0); // Adjust leg height

	// Hip rotation for walking
	glRotatef(hipAngleLeft, 1.0, 0.0, 0.0); // Rotate hip joint

	// Upper leg segment - beige
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, beige_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, beige_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, beige_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, beige_mat_shininess);

	// Rotate and scale the upper leg
	glRotatef(-15, 1.0, 0.0, 0.0); // Slight rotation for a zig-zag pose
	glScalef(0.2 * robotBodyWidth, 0.5 * robotBodyLength, 0.2 * robotBodyDepth);
	glutSolidCube(1.0);
	glPopMatrix(); // End upper leg segment

	// Add kneecap before moving down for the lower leg
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, light_brown_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, light_brown_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, light_brown_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, light_brown_mat_shininess);

	// Adjust kneecap placement (slightly forward on the Z-axis)
	glTranslatef(0.0, -0.25 * robotBodyLength, 0.10 * robotBodyDepth);  // Move kneecap forward slightly
	glScalef(0.25 * robotBodyWidth, 0.1 * robotBodyLength, 0.25 * robotBodyDepth);  // Scale the kneecap
	glutSolidCube(1.0);  // Draw kneecap
	glPopMatrix();

	// Move down for knee joint
	glTranslatef(0.0, -0.5 * robotBodyLength, 0.0);
	// Knee rotation
	glRotatef(kneeAngleLeft, 1.0, 0.0, 0.0); // Rotate knee

	// Lower leg segment - green
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, green_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, green_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, green_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, green_mat_shininess);

	glRotatef(15, 1.0, 0.0, 0.0); // Rotate to maintain zig-zag pose
	glScalef(0.2 * robotBodyWidth, 0.5 * robotBodyLength, 0.2 * robotBodyDepth);
	glutSolidCube(1.0);
	glPopMatrix(); // End lower leg segment

	// New kneecap between the two green parts
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, light_brown_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, light_brown_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, light_brown_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, light_brown_mat_shininess);

	// Translate to position the kneecap between the two green parts
	glTranslatef(0.0, -0.25 * robotBodyLength, 0.0); // Adjust based on the spacing between the two green parts
	glScalef(0.25 * robotBodyWidth, 0.1 * robotBodyLength, 0.25 * robotBodyDepth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Move down for the second (third part) green leg
	glTranslatef(0.0, -0.5 * robotBodyLength, 0.0);
	// Lower leg rotation
	glRotatef(lowerLegAngleLeft, 1.0, 0.0, 0.0);

	// Second green part - same size as the previous green part
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, green_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, green_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, green_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, green_mat_shininess);

	glRotatef(-15, 1.0, 0.0, 0.0); // Continue zig-zag pose
	glScalef(0.2 * robotBodyWidth, 0.5 * robotBodyLength, 0.2 * robotBodyDepth);
	glutSolidCube(1.0);
	glPopMatrix(); // End second green part

	// Move down for ankle (adjusted to move feet up)
	glTranslatef(0.0, -0.3 * robotBodyLength, 0.0);  // Reduced from -0.5 to -0.3 for closer connection

	// Ankle rotation
	glRotatef(ankleAngleLeft, 1.0, 0.0, 0.0); // Rotate ankle

	// Foot segment - light brown
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, light_brown_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, light_brown_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, light_brown_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, light_brown_mat_shininess);

	// Foot base
	glPushMatrix();
	glScalef(0.4 * robotBodyDepth, 0.1 * robotBodyLength, 0.6 * robotBodyWidth); // Foot dimensions
	glutSolidCube(1.0);
	glPopMatrix(); // End foot base

	// Add two dents in front of the foot
	// First front dent
	glPushMatrix();
	glTranslatef(-0.15 * robotBodyDepth, 0.0, 0.4 * robotBodyWidth); // Move to the front-left
	glScalef(0.1 * robotBodyDepth, 0.1 * robotBodyLength, 0.2 * robotBodyWidth); // Small cube for the dent
	glutSolidCube(1.0);
	glPopMatrix();

	// Second front dent
	glPushMatrix();
	glTranslatef(0.15 * robotBodyDepth, 0.0, 0.4 * robotBodyWidth); // Move to the front-right
	glScalef(0.1 * robotBodyDepth, 0.1 * robotBodyLength, 0.2 * robotBodyWidth); // Small cube for the dent
	glutSolidCube(1.0);
	glPopMatrix();

	// Add two dents in back of the foot
	// First back dent
	glPushMatrix();
	glTranslatef(-0.15 * robotBodyDepth, 0.0, -0.4 * robotBodyWidth); // Move to the back-left
	glScalef(0.1 * robotBodyDepth, 0.1 * robotBodyLength, 0.2 * robotBodyWidth); // Small cube for the dent
	glutSolidCube(1.0);
	glPopMatrix();

	// Second back dent
	glPushMatrix();
	glTranslatef(0.15 * robotBodyDepth, 0.0, -0.4 * robotBodyWidth); // Move to the back-right
	glScalef(0.1 * robotBodyDepth, 0.1 * robotBodyLength, 0.2 * robotBodyWidth); // Small cube for the dent
	glutSolidCube(1.0);
	glPopMatrix();

	glPopMatrix(); // End left foot

	glPopMatrix(); // End left leg

	// Right leg (copy of the left leg but mirrored)
	glPushMatrix();
	// Move the leg lower to connect better to the body
	glTranslatef(-0.5 * robotBodyWidth, -0.7 * robotBodyLength, 0.0); // Adjust leg height

	// Hip rotation
	glRotatef(hipAngleRight, 1.0, 0.0, 0.0); // Rotate hip

	// Upper leg segment - beige
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, beige_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, beige_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, beige_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, beige_mat_shininess);

	glRotatef(-15, 1.0, 0.0, 0.0); // Zig-zag pose
	glScalef(0.2 * robotBodyWidth, 0.5 * robotBodyLength, 0.2 * robotBodyDepth);
	glutSolidCube(1.0);
	glPopMatrix(); // End upper leg segment

	// Add kneecap
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, light_brown_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, light_brown_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, light_brown_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, light_brown_mat_shininess);

	// Adjust kneecap placement
	glTranslatef(0.0, -0.25 * robotBodyLength, 0.10 * robotBodyDepth);  // Move kneecap forward
	glScalef(0.25 * robotBodyWidth, 0.1 * robotBodyLength, 0.25 * robotBodyDepth);  // Adjust kneecap scale
	glutSolidCube(1.0);
	glPopMatrix();

	// Move down for knee
	glTranslatef(0.0, -0.5 * robotBodyLength, 0.0);
	// Knee rotation
	glRotatef(kneeAngleRight, 1.0, 0.0, 0.0); // Rotate knee

	// Lower leg segment - green
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, green_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, green_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, green_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, green_mat_shininess);

	glRotatef(15, 1.0, 0.0, 0.0); // Zig-zag rotation
	glScalef(0.2 * robotBodyWidth, 0.5 * robotBodyLength, 0.2 * robotBodyDepth);
	glutSolidCube(1.0);
	glPopMatrix(); // End lower leg segment

	// New kneecap between the two green parts (right leg)
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, light_brown_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, light_brown_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, light_brown_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, light_brown_mat_shininess);

	glTranslatef(0.0, -0.25 * robotBodyLength, 0.0); // Adjust for kneecap position
	glScalef(0.25 * robotBodyWidth, 0.1 * robotBodyLength, 0.25 * robotBodyDepth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Move down for the second green part (right leg)
	glTranslatef(0.0, -0.5 * robotBodyLength, 0.0);
	// Lower leg rotation
	glRotatef(lowerLegAngleRight, 1.0, 0.0, 0.0);

	// Second green part (right leg)
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, green_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, green_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, green_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, green_mat_shininess);

	glRotatef(-15, 1.0, 0.0, 0.0); // Continue zig-zag pose
	glScalef(0.2 * robotBodyWidth, 0.5 * robotBodyLength, 0.2 * robotBodyDepth);
	glutSolidCube(1.0);
	glPopMatrix(); // End second green part

	// Move down for ankle (adjusted to move feet up)
	glTranslatef(0.0, -0.3 * robotBodyLength, 0.0);  // Reduced from -0.5 to -0.3 for closer connection

	// Ankle rotation
	glRotatef(ankleAngleRight, 1.0, 0.0, 0.0); // Rotate ankle

	// Foot segment - light brown
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, light_brown_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, light_brown_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, light_brown_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, light_brown_mat_shininess);

	// Foot base
	glPushMatrix();
	glScalef(0.4 * robotBodyDepth, 0.1 * robotBodyLength, 0.6 * robotBodyWidth); // Foot dimensions
	glutSolidCube(1.0);
	glPopMatrix(); // End foot base

	// Add two dents in front of the foot
	// First front dent
	glPushMatrix();
	glTranslatef(-0.15 * robotBodyDepth, 0.0, 0.4 * robotBodyWidth); // Move to the front-left
	glScalef(0.1 * robotBodyDepth, 0.1 * robotBodyLength, 0.2 * robotBodyWidth); // Small cube for the dent
	glutSolidCube(1.0);
	glPopMatrix();

	// Second front dent
	glPushMatrix();
	glTranslatef(0.15 * robotBodyDepth, 0.0, 0.4 * robotBodyWidth); // Move to the front-right
	glScalef(0.1 * robotBodyDepth, 0.1 * robotBodyLength, 0.2 * robotBodyWidth); // Small cube for the dent
	glutSolidCube(1.0);
	glPopMatrix();

	// Add two dents in back of the foot
	// First back dent
	glPushMatrix();
	glTranslatef(-0.15 * robotBodyDepth, 0.0, -0.4 * robotBodyWidth); // Move to the back-left
	glScalef(0.1 * robotBodyDepth, 0.1 * robotBodyLength, 0.2 * robotBodyWidth); // Small cube for the dent
	glutSolidCube(1.0);
	glPopMatrix();

	// Second back dent
	glPushMatrix();
	glTranslatef(0.15 * robotBodyDepth, 0.0, -0.4 * robotBodyWidth); // Move to the back-right
	glScalef(0.1 * robotBodyDepth, 0.1 * robotBodyLength, 0.2 * robotBodyWidth); // Small cube for the dent
	glutSolidCube(1.0);
	glPopMatrix();

	glPopMatrix(); // End right foot

	glPopMatrix(); // End right leg
}

void drawLeftArm()
{
	// Set the material for the arm (green)
	glMaterialfv(GL_FRONT, GL_AMBIENT, green_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, green_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, green_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, green_mat_shininess);

	glPushMatrix();
	// Position upper arm higher to connect with the body
	glTranslatef(0.5 * robotBodyWidth + 0.5 * upperArmWidth, 0.3 * robotBodyLength, 0.0); // Adjusted Y position to connect with body

	// Draw upper arm (green part)
	glPushMatrix();
	glScalef(upperArmWidth, 0.6 * upperArmLength, upperArmWidth); // Upper part is shorter
	glutSolidCube(1.0);
	glPopMatrix();

	// Add the elbow joint (grey part)
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, dark_grey_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, dark_grey_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, dark_grey_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, dark_grey_shininess);

	// Position and scale the elbow
	glTranslatef(0.0, -0.5 * 0.6 * upperArmLength, 0.0); // Adjust based on upper arm length
	glScalef(1.2 * upperArmWidth, 0.1 * upperArmLength, 1.2 * upperArmWidth); // Slightly larger elbow joint
	glutSolidCube(1.0);  // Draw elbow
	glPopMatrix();

	// Move down for the lower arm and translate further forward along Z-axis
	glTranslatef(0.0, -0.9 * 0.6 * upperArmLength, 1.1); // Slightly increased forward translation along the Z-axis

	// Apply rotation to the lower arm for an angled effect
	glRotatef(-30.0, 1.0, 0.0, 0.0); // Rotate around the X-axis to make the lower arm angled

	// Draw lower arm (green part)
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, green_mat_ambient);  // Set back to green for the lower arm
	glMaterialfv(GL_FRONT, GL_SPECULAR, green_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, green_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, green_mat_shininess);

	glScalef(upperArmWidth, 0.6 * upperArmLength, upperArmWidth); // Lower part is also shorter
	glutSolidCube(1.0);
	glPopMatrix();

	// Now draw the hand
	glMaterialfv(GL_FRONT, GL_AMBIENT, dark_grey_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, dark_grey_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, dark_grey_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, dark_grey_shininess);

	// Position the hand slightly above and more inside the lower arm
	glPushMatrix();
	glTranslatef(0.0, -0.35 * 0.6 * upperArmLength - 0.15, 0.0);  // Adjusted Y position to bring the hand inside the lower arm
	glScalef(0.7 * upperArmWidth, 0.5 * upperArmLength, 0.7 * upperArmWidth);  // Scale for the hand
	glutSolidCube(1.0);  // Hand (palm)

	// Add the fingers
	float fingerWidth = 0.06 * upperArmWidth; // Thicker fingers
	float fingerLength = 0.05 * upperArmLength; // Shorter length for fingers

	// Draw 5 fingers
	for (int i = -2; i <= 2; i++) {
		glPushMatrix();
		glTranslatef(i * (0.12 * upperArmWidth), -0.2 * (0.3 * upperArmLength), 0.0);  // Move fingers up
		glScalef(fingerWidth, fingerLength, fingerWidth);  // Scale fingers to thicker and shorter size
		glutSolidCube(1.0);
		glPopMatrix();
	}

	glPopMatrix();  // End of hand
	glPopMatrix();  // End of arm
}

void drawRightArm()
{
	// Set material properties for the arm (green)
	glMaterialfv(GL_FRONT, GL_AMBIENT, green_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, green_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, green_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, green_mat_shininess);


	glPushMatrix();

	// Adjust translation to mirror the left arm, move it up and forward slightly
	glTranslatef(-(0.5 * robotBodyWidth + 0.5 * upperArmWidth), 0.3 * robotBodyLength, 0.2 * robotBodyDepth); // Adjust Y-value for correct height

	glRotatef(-45.0, 1.0, 0.0, 0.0); // Tilt arm forward

	// Draw upper arm (green part)
	glPushMatrix();
	glScalef(upperArmWidth, 0.6 * upperArmLength, upperArmWidth); // Upper part is shorter
	glutSolidCube(1.0);
	glPopMatrix();

	// Add the brown elbow joint
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, dark_grey_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, dark_grey_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, dark_grey_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, dark_grey_shininess);

	// Position and scale the elbow
	glTranslatef(0.0, -0.5 * 0.6 * upperArmLength, 0.0); // Position the elbow under the upper arm
	glScalef(1.2 * upperArmWidth, 0.1 * upperArmLength, 1.2 * upperArmWidth); // Slightly larger elbow joint
	glutSolidCube(1.0);  // Draw elbow
	glPopMatrix();

	// Move down for the lower arm, starting from the elbow
	glTranslatef(0.0, -0.75 * 0.8 * upperArmLength, 1.3); // Move the lower arm further forward along Z-axis

	// Apply rotation to the lower arm for an angled effect
	glRotatef(-25.0, 1.0, 0.0, 0.0); // Rotate around the X-axis for angle

	// Draw lower arm (green part)
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, green_mat_ambient);  // Set back to green for the lower arm
	glMaterialfv(GL_FRONT, GL_SPECULAR, green_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, green_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, green_mat_shininess);

	glScalef(upperArmWidth, 0.7 * upperArmLength, upperArmWidth); // Same size of the lower arm
	glutSolidCube(1.0);
	glPopMatrix();

	// Now handle the cannon attached to the lower arm
	glMaterialfv(GL_FRONT, GL_AMBIENT, dark_grey_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, dark_grey_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, dark_grey_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, dark_grey_shininess);

	// Position the cannon at the end of the lower arm
	glPushMatrix();
	glTranslatef(0.0, -0.4 * upperArmLength - 0.4 * gunLength, 0.0);  // Position cannon directly at the lower arm's end

	// Apply cannon spin along its Y-axis (screw-like spin)
	if (spinCannon) {
		glRotatef(cannonSpinAngle, 0.0, 1.0, 0.0);  // Spin along the Y-axis
	}

	// Draw the gun (cannon body)
	glPushMatrix();
	glScalef(gunWidth, gunLength, gunDepth);
	glutSolidCube(1.0);  // Cannon body
	glPopMatrix();

	// Draw the cannon barrel (cylinder)
	glPushMatrix();
	glTranslatef(0.0, -0.5 * gunLength, 0.0);  // Move to the end of the cannon
	glRotatef(90.0, 1.0, 0.0, 0.0);  // Align the cylinder properly
	GLUquadric* quad = gluNewQuadric();
	gluCylinder(quad, 1.5, 1.5, 5.0, 40, 20);  // Barrel
	glPopMatrix();

	// Draw the orange projectile inside the cannon
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, red_orange_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, red_orange_diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, red_orange_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, red_orange_shininess);
	glTranslatef(0.0, -2.5 * gunLength, 0.0);  // Inside the cannon
	glScalef(gunWidth * 0.5, gunLength * 0.1, gunDepth * 0.5);
	glutSolidCube(1.0);  // The orange projectile
	glPopMatrix();

	// Draw the magazine under the cannon
	glPushMatrix();
	glTranslatef(0.0, -gunLength - 1.0, 0.0);  // Position magazine below the cannon
	glScalef(gunWidth * 0.8, gunLength * 0.4, gunDepth * 0.8);  // Scale the magazine
	glutSolidCube(1.0);
	glPopMatrix();

	glPopMatrix();  // End cannon drawing
	glPopMatrix();  // End arm drawing
}

void reshape(int w, int h)
{
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, (GLdouble)w / h, 0.2, 500.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// Set up the camera at position (0, 6, 35) looking at the origin
	gluLookAt(0.0, 6.0, 35.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
}

void moveRobots(int value) {
	for (int i = 0; i < numRobots; i++) {
		if (robots[i].disabled) {
			if (robots[i].breakingTimer > 0) {
				robots[i].breakingTimer--; // Count down breaking animation timer
			}
			else {
				robots[i].xOffset = 1000.0f; // Move robot far off-screen
				robots[i].zOffset = 1000.0f; // Prevent further interaction
			}
			continue; // Skip further processing for disabled robots
		}

		float rad = robots[i].direction * (M_PI / 180.0f);

		robots[i].zOffset += robots[i].speed * cos(rad);
		robots[i].xOffset += robots[i].speed * sin(rad);

		// Change direction randomly
		if (rand() % 100 < 5) { // 5% chance to change direction
			robots[i].direction += (rand() % 90 - 45); // Turn randomly between -45 to +45 degrees
		}

		if (robots[i].zOffset > 50.0f || robots[i].zOffset < -50.0f) {
			robots[i].direction += 180.0f; // Reverse direction
		}
		if (robots[i].xOffset > 50.0f || robots[i].xOffset < -50.0f) {
			robots[i].direction += 180.0f; // Reverse direction
		}
	}

	glutPostRedisplay();
	glutTimerFunc(16, moveRobots, 0);
}

bool stop = false;

void stepAnimation(int value)
{
	if (walking) {
		for (int i = 0; i < numRobots; i++) {
			float angleStep = 1.0f + i * 0.2f;
			float positionStep = 0.05f;

			if (walkingForward) {
				if (hipAngleLeft < 50.0f) {
					hipAngleLeft += angleStep;
					kneeAngleLeft -= angleStep * 0.75f;
					ankleAngleLeft += angleStep * 0.5f;
					lowerLegAngleLeft += angleStep;
				}

				if (hipAngleRight > -50.0f) {
					hipAngleRight -= angleStep;
					kneeAngleRight += angleStep * 0.75f;
					ankleAngleRight -= angleStep * 0.5f;
					lowerLegAngleRight -= angleStep;
				}

				if (hipAngleLeft >= 50.0f && hipAngleRight <= -50.0f) {
					walkingForward = false;
				}

				robotZOffset += positionStep;
			}
			else {
				if (hipAngleLeft > 0.0f) {
					hipAngleLeft -= angleStep;
					kneeAngleLeft += angleStep * 0.75f;
					ankleAngleLeft -= angleStep * 0.5f;
					lowerLegAngleLeft -= angleStep;
				}

				if (hipAngleRight < 0.0f) {
					hipAngleRight += angleStep;
					kneeAngleRight -= angleStep * 0.75f;
					ankleAngleRight += angleStep * 0.5f;
					lowerLegAngleRight += angleStep;
				}

				if (hipAngleLeft <= 0.0f && hipAngleRight >= 0.0f) {
					walkingForward = true;
				}
			}
		}

		glutPostRedisplay();

		if (walking) {
			glutTimerFunc(50, stepAnimation, 0);
		}
	}
}

void cannonAnimation(int value)
{
	if (spinCannon)
	{
		cannonSpinAngle += 5.0f;  // Increment the cannon spin angle to rotate the cannon
		if (cannonSpinAngle > 360.0f) {
			cannonSpinAngle -= 360.0f;  // Reset the angle after a full rotation
		}
		glutPostRedisplay();  // Redraw to show the updated cannon position
		glutTimerFunc(10, cannonAnimation, 0);  // Continue spinning the cannon
	}
}

void resetJointAngles() {
	hipAngleLeft = 0.0f;
	kneeAngleLeft = 0.0f;
	ankleAngleLeft = 0.0f;
	lowerLegAngleLeft = 0.0f;

	hipAngleRight = 0.0f;
	kneeAngleRight = 0.0f;
	ankleAngleRight = 0.0f;
	lowerLegAngleRight = 0.0f;
}

void keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'k':  // Control left knee
		selectedJoint = 1;  // Select knee joint
		break;
	case 'h':  // Control left hip
		selectedJoint = 2;  // Select hip joint
		break;
	case 'n':  // Control neck (head rotation)
		selectedJoint = 3;
		break;
	case 'b':  // Select body rotation
		selectedJoint = 4;
		break;
	case 'l':  // Control lower part of the left leg (between middle and lower sections)
		selectedJoint = 5;
		break;
	case 'a': // Control ankle
		selectedJoint = 6;
		break;
	case '1':  // Default view (isometric)
		cameraView = 0;
		break;
	case '2':  // Front view
		cameraView = 1;
		break;
	case '3':  // Side view
		cameraView = 2;
		break;
	case '4':  // Top-down view
		cameraView = 3;
		break;
	case 'w':  // Start/Stop walking
		walking = !walking;
		if (walking) {
			glutTimerFunc(10, stepAnimation, 0);  // Start walking animation
		}
		else {
			resetJointAngles();  // Reset joint angles when walking stops
			glutPostRedisplay(); // Trigger a redraw to reflect the reset angles
		}
		break;
	case 'c':  // Toggle cannon spinning
		spinCannon = !spinCannon;
		if (spinCannon) {
			glutTimerFunc(10, cannonAnimation, 0);
		}
		break;
	default:
		break;
	}
	glutPostRedisplay();
}

void animationHandler(int param)
{
	if (!stop)
	{
		shoulderAngle += 1.0;
		glutPostRedisplay();
		glutTimerFunc(10, animationHandler, 0);
	}
}

void functionKeys(int key, int x, int y)
{
	switch (key)
	{
	case GLUT_KEY_LEFT:
		// Control right knee when 'k' is pressed
		if (selectedJoint == 1) {
			kneeAngleRight += 2.0f;  // Rotate right knee
		}
		// Control right hip when 'h' is pressed
		else if (selectedJoint == 2) {
			hipAngleRight += 2.0f;   // Rotate right hip
		}
		// Control neck
		else if (selectedJoint == 3) {
			neckAngle += 2.0f;     // Rotate neck (left turn)
		}
		// Control body rotation
		else if (selectedJoint == 4) {
			robotAngle += 2.0f;    // Rotate body (left)
			if (robotAngle > 360.0f) {
				robotAngle -= 360.0f;
			}
		}
		// Rotate lower left leg
		else if (selectedJoint == 5) {
			lowerLegAngleLeft += 2.0f;  // Rotate lower left leg
		}
		// Rotate ankle
		else if (selectedJoint == 6) {
			ankleAngleLeft += 2.0f;
		}
		break;

	case GLUT_KEY_RIGHT:
		// Control right knee when 'k' is pressed
		if (selectedJoint == 1) {
			kneeAngleRight -= 2.0f;  // Rotate right knee in the opposite direction
		}
		// Control right hip when 'h' is pressed
		else if (selectedJoint == 2) {
			hipAngleRight -= 2.0f;   // Rotate right hip in the opposite direction
		}
		// Control neck
		else if (selectedJoint == 3) {
			neckAngle -= 2.0f;     // Rotate neck (right turn)
		}
		// Control body rotation
		else if (selectedJoint == 4) {
			robotAngle -= 2.0f;    // Rotate body (right)
			if (robotAngle < -360.0f) {
				robotAngle += 360.0f;
			}
		}
		// Rotate lower left leg in opposite direction
		else if (selectedJoint == 5) {
			lowerLegAngleLeft -= 2.0f;  // Rotate lower left leg in the opposite direction
		}
		// Rotate ankle
		else if (selectedJoint == 6) {
			ankleAngleLeft -= 2.0f;
		}
		break;

	case GLUT_KEY_UP:
		// Control left knee when 'k' is pressed
		if (selectedJoint == 1) {
			kneeAngleLeft += 2.0f;  // Rotate left knee
		}
		// Control left hip when 'h' is pressed
		else if (selectedJoint == 2) {
			hipAngleLeft += 2.0f;   // Rotate left hip
		}
		// Control neck
		else if (selectedJoint == 3) {
			neckAngle += 2.0f;     // Rotate neck (upward turn - simulating look up)
		}
		else if (selectedJoint == 4) {
			robotAngle += 2.0f;    // Optional: You can add more functionality for the body here
		}
		// Rotate lower left leg in opposite direction
		else if (selectedJoint == 5) {
			lowerLegAngleLeft += 2.0f;  // Rotate lower left leg in the opposite direction
		}
		// Rotate ankle
		else if (selectedJoint == 6) {
			ankleAngleLeft += 2.0f;
		}
		break;

	case GLUT_KEY_DOWN:
		// Control left knee when 'k' is pressed
		if (selectedJoint == 1) {
			kneeAngleLeft -= 2.0f;  // Rotate left knee in the opposite direction
		}
		// Control left hip when 'h' is pressed
		else if (selectedJoint == 2) {
			hipAngleLeft -= 2.0f;   // Rotate left hip in the opposite direction
		}
		// Control neck
		else if (selectedJoint == 3) {
			neckAngle -= 2.0f;     // Rotate neck (downward turn - simulating look down)
		}
		else if (selectedJoint == 4) {
			robotAngle -= 2.0f;    // Optional: You can add more functionality for the body here
		}
		// Rotate lower left leg
		else if (selectedJoint == 5) {
			lowerLegAngleLeft -= 2.0f;  // Rotate lower left leg
		}
		// Rotate ankle
		else if (selectedJoint == 6) {
			ankleAngleLeft -= 2.0f;
		}
		break;
	}
	glutPostRedisplay();   // Trigger redraw to apply changes
}

void handleMouseClick(int button, int state, int x, int y) {
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
		fireDefensiveCannonProjectile(); // Fire defensive projectile
	}
}

void handleMouseMotion(int x, int y) {
	static int lastX = x, lastY = y;

	// Adjust yaw (horizontal) and pitch (vertical) based on mouse movement
	float deltaX = (x - lastX) * 0.05f; // Reduced sensitivity
	float deltaY = (y - lastY) * 0.05f;

	cameraYaw += deltaX;
	cameraPitch -= deltaY;

	// Clamp the pitch to avoid excessive tilt
	const float maxPitch = 45.0f; // Maximum degrees to look up or down
	if (cameraPitch > maxPitch) cameraPitch = maxPitch;
	if (cameraPitch < -maxPitch) cameraPitch = -maxPitch;

	// Sync cannon's barrel angles with camera's orientation
	barrelYawAngle = -cameraYaw; // Reverse direction for intuitive alignment
	barrelTiltAngle = cameraPitch;

	// Save current mouse position
	lastX = x;
	lastY = y;

	glutPostRedisplay();
}
