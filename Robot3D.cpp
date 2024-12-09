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
#include <string>
#include <sstream>
#define M_PI 3.14159265358979323846

GLuint robotTexture, cannonBaseTexture, cannonBarrelTexture;

const int vWidth = 650;
const int vHeight = 500;

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
float legAngle = 0.0f;
bool spinCannon = true;
float cannonSpinAngle = 0.0f;

// Joint angles for walking
float hipAngleLeft = 0.0f;
float kneeAngleLeft = 0.0f;
float ankleAngleLeft = 0.0f;
float hipAngleRight = 0.0f;
float kneeAngleRight = 0.0f;
float ankleAngleRight = 0.0f;
float lowerLegAngleLeft = 0.0f;
float lowerLegAngleRight = 0.0f;

// Flag to control walking state
bool walking = false;
bool stepBackwards = false;
bool walkingForward = true;

// Control Robot body rotation on base
float robotAngle = 0.0;
float neckAngle = 0.0f;

// Control arm rotation
float shoulderAngle = -40.0;
float gunAngle = -25.0;

// Variable to control the position offset along the direction of movement
float robotZOffset = 0.0f;
bool movingForward = true;

float barrelYawAngle = 0.0f;
float barrelTiltAngle = 0.0f;

float cameraX = 0.0f, cameraY = 15.0f, cameraZ = 100.0f;
float cameraYaw = 0.0f;
float cameraPitch = 0.0f;
float cameraDistance = 100.0f;
bool cannonDisabled = false;
float cannonFadeProgress = 0.0f;
float cannonColor[4] = { 0.0f, 1.0f, 0.0f, 1.0f };

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

// Texture mapping enemy robot light properties
GLfloat light_position0[] = { -4.0F, 8.0F, 8.0F, 1.0F };
GLfloat light_position1[] = { 4.0F, 8.0F, 8.0F, 1.0F };
GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat light_ambient[] = { 0.2F, 0.2F, 0.2F, 1.0F };

GLfloat neon_green_ambient[] = { 0.0f, 0.8f, 0.0f, 1.0f };
GLfloat neon_green_diffuse[] = { 0.0f, 1.0f, 0.0f, 1.0f };
GLfloat neon_green_specular[] = { 0.5f, 1.0f, 0.5f, 1.0f };
GLfloat neon_green_shininess[] = { 100.0f };

GLfloat light_grey_ambient[] = { 0.5f, 0.5f, 0.5f, 1.0f };
GLfloat light_grey_diffuse[] = { 0.7f, 0.7f, 0.7f, 1.0f };
GLfloat light_grey_specular[] = { 0.8f, 0.8f, 0.8f, 1.0f };
GLfloat light_grey_shininess = 50.0f;

// Mouse button
int currentButton;

int score = 0;
int robotCount = 3;

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
	float x, y, z;
	float speed;
	float directionX, directionY, directionZ;
	bool active;
};

const int maxProjectiles = 10;
Projectile projectiles[maxProjectiles];

std::vector<Projectile> defensiveProjectiles;

float spacing = 20.0f;
Robot* robots = nullptr;

// Default Mesh Size
int meshSize = 16;

void initOpenGL(int w, int h);
void display(void);
void reshape(int w, int h);
void handleMouseMotion(int x, int y);
void keyboard(unsigned char key, int x, int y);
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
void resetRobots();
void cleanupRobots();
void resetApplication();
void cannonAnimation(int value);
bool gameDisabled = false;
GLuint createEnemyRobotTexture();
GLuint createMetallicTexture();
GLuint createDarkGrayTexture();

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
	glutPassiveMotionFunc(handleMouseMotion);
	glutKeyboardFunc(keyboard);

	// Start movement animations
	walking = true;
	glutTimerFunc(16, moveRobots, 0);
	glutTimerFunc(10, stepAnimation, 0);
	glutTimerFunc(16, updateEnemyProjectiles, 0);
	glutTimerFunc(16, updateDefensiveProjectilesTimer, 0);
	glutTimerFunc(500, fireRandomEnemyProjectiles, 0);
	glutTimerFunc(10, cannonAnimation, 0);

	// Start event loop, never returns
	glutMainLoop();
	cleanupRobots();

	return 0;
}

GLuint createEnemyRobotTexture() {
	const int textureSize = 512; // 512x512 texture
	GLuint textureID;
	unsigned char* textureData = new unsigned char[textureSize * textureSize * 3];

	// Generate procedural pattern, light gray with darker panel-like divisions
	for (int y = 0; y < textureSize; y++) {
		for (int x = 0; x < textureSize; x++) {
			int index = (y * textureSize + x) * 3;

			unsigned char baseColor = 200;

			// Panel effect, darker lines every 32 pixels
			if ((x % 32 == 0 || y % 32 == 0)) {
				baseColor = 160; // Panel line color
			}

			// Subtle random noise for realism
			unsigned char noise = rand() % 10;
			textureData[index] = baseColor - noise;
			textureData[index + 1] = baseColor - noise;
			textureData[index + 2] = baseColor - noise;
		}
	}

	// Create the OpenGL texture
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_2D, textureID);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, textureSize, textureSize, 0, GL_RGB, GL_UNSIGNED_BYTE, textureData);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	// Free memory and return texture ID
	delete[] textureData;
	return textureID;
}

GLuint createMetallicTexture() {
	const int width = 64, height = 64;
	unsigned char data[width * height * 3];

	for (int i = 0; i < width; ++i) {
		for (int j = 0; j < height; ++j) {
			int index = (i + j * width) * 3;
			unsigned char grayValue = (i + j) % 128 + 100;
			data[index] = grayValue;
			data[index + 1] = grayValue;
			data[index + 2] = grayValue;
		}
	}

	GLuint textureID;
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_2D, textureID);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glBindTexture(GL_TEXTURE_2D, 0);
	return textureID;
}

GLuint createDarkGrayTexture() {
	const int width = 64, height = 64;
	unsigned char data[width * height * 3];

	for (int i = 0; i < width; ++i) {
		for (int j = 0; j < height; ++j) {
			int index = (i + j * width) * 3;
			unsigned char grayValue = 50 + rand() % 20;
			data[index] = grayValue;
			data[index + 1] = grayValue;
			data[index + 2] = grayValue;
		}
	}

	GLuint textureID;
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_2D, textureID);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glBindTexture(GL_TEXTURE_2D, 0);
	return textureID;
}

void initializeRobots() {
	if (robots != nullptr) {
		delete[] robots;
	}

	robots = new Robot[robotCount];

	for (int i = 0; i < robotCount; i++) {
		robots[i].xOffset = (i - (robotCount - 1) * 0.5f) * spacing;
		robots[i].zOffset = 0.0f;
		robots[i].speed = 0.1f + 0.05f * i;
		robots[i].direction = 0.0f;
		robots[i].disabled = false;
		robots[i].breakingTimer = 0;
	}
}

void resetRobots() {
	robotCount++;
	score++;
	initializeRobots();
}

void cleanupRobots() {
	if (robots != nullptr) {
		delete[] robots;
		robots = nullptr;
	}
}

void resetApplication() {
	score = 0;
	robotCount = 3;

	// Reset the defensive cannon
	cannonDisabled = false;
	cannonFadeProgress = 0.0f;
	cannonColor[0] = green_mat_diffuse[0];
	cannonColor[1] = green_mat_diffuse[1];
	cannonColor[2] = green_mat_diffuse[2];
	cannonColor[3] = green_mat_diffuse[3];

	// Reinitialize robots and projectiles
	initializeRobots();
	initializeEnemyProjectiles();
	defensiveProjectiles.clear();
	glutPostRedisplay();
}

void initializeEnemyProjectiles() {
	for (int i = 0; i < maxProjectiles; i++) {
		projectiles[i].active = false;
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
			float dirX = camDirX + (rand() % 100 / 500.0f - inaccuracy);
			float dirY = camDirY + (rand() % 100 / 500.0f - inaccuracy);
			float dirZ = camDirZ + (rand() % 100 / 500.0f - inaccuracy);

			// Negate the direction to ensure the projectiles move towards defensive cannon
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

			projectiles[i].speed = 2.0f; // Set speed for the projectile
			projectiles[i].active = true;

			break; // Use only one projectile at a time
		}
	}
}

bool detectDefensiveLaserCollisionWithBot(const Projectile& defensiveLaser, const Robot& enemyBot) {
	float botRadius = robotBodyWidth * 0.5f;
	float laserRadius = 0.5f;

	float dx = defensiveLaser.x - enemyBot.xOffset;
	float dz = defensiveLaser.z - enemyBot.zOffset;

	float distance = sqrt(dx * dx + dz * dz);

	return distance < (botRadius + laserRadius);
}

VECTOR3D getCannonWorldPosition(Robot robot) {
	float rad = robot.direction * (M_PI / 180.0f);
	float baseX = robot.xOffset;
	float baseZ = robot.zOffset;

	float bodyRotationRad = robotAngle * (M_PI / 180.0f);

	float cannonLocalX = -(0.5 * robotBodyWidth + gunWidth);
	float cannonLocalY = 0.3 * robotBodyLength;
	float cannonLocalZ = 0.5 * robotBodyDepth;

	float cannonRotatedX = cannonLocalX * cos(bodyRotationRad) - cannonLocalZ * sin(bodyRotationRad);
	float cannonRotatedZ = cannonLocalX * sin(bodyRotationRad) + cannonLocalZ * cos(bodyRotationRad);

	float cannonWorldX = baseX + cannonRotatedX;
	float cannonWorldY = cannonLocalY;
	float cannonWorldZ = baseZ + cannonRotatedZ;

	return VECTOR3D(cannonWorldX, cannonWorldY, cannonWorldZ);
}

void fireRandomEnemyProjectiles(int value) {
	float projectileDirX = 0.0f, projectileDirY = 0.0f, projectileDirZ = -1.0f;

	for (int i = 0; i < robotCount; i++) {
		if (robots[i].disabled) {
			continue;
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

			// Check collision with the defensive cannon
			if (!cannonDisabled &&
				fabs(projectiles[i].x - cameraX) < 4.0f &&
				fabs(projectiles[i].y - (cameraY - 5.0f)) < 2.0f &&
				fabs(projectiles[i].z - (cameraZ - 10.0f)) < 2.0f) {
				cannonDisabled = true;  // Disable cannon
				glutTimerFunc(16, updateCannonFade, 0); // Disabled animation (fade to black)
				projectiles[i].active = false;
			}

			// Deactivate if projectile goes out of bounds
			if (fabs(projectiles[i].z) < -100 || projectiles[i].z > 100 ||
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
	updateDefensiveProjectiles();
	glutPostRedisplay();
	glutTimerFunc(16, updateDefensiveProjectilesTimer, 0);
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
				up = VECTOR3D(1.0f, 0.0f, 0.0f);
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

			// Draw the projectile
			GLUquadric* quad = gluNewQuadric();
			gluCylinder(quad, 0.1f, 0.1f, 3.0f, 16, 16);
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
		float scale = enemyBot.breakingTimer / 50.0f;
		glScalef(scale, scale, scale);
		drawRobot();
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
	glPushMatrix();

	// Adjust position based on camera's position and direction
	float cannonOffsetDistance = 10.0f;
	float offsetX = sin(cameraYaw * M_PI / 180.0) * cos(cameraPitch * M_PI / 180.0);
	float offsetY = sin(cameraPitch * M_PI / 180.0);
	float offsetZ = -cos(cameraYaw * M_PI / 180.0) * cos(cameraPitch * M_PI / 180.0);

	float cannonX = cameraX + offsetX * cannonOffsetDistance;
	float cannonY = cameraY + offsetY * cannonOffsetDistance - 5.0f;
	float cannonZ = cameraZ + offsetZ * cannonOffsetDistance;

	glTranslatef(cannonX, cannonY, cannonZ);

	// Align the cannon with the camera's orientation
	glRotatef(-cameraYaw, 0.0, 1.0, 0.0);
	glRotatef(cameraPitch, 1.0, 0.0, 0.0);

	// Texture Binding and Material Setup
	if (cannonDisabled && cannonFadeProgress >= 1.0f) {
		// Apply fully black material
		glDisable(GL_TEXTURE_2D);
		GLfloat blackAmbient[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
		GLfloat blackDiffuse[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
		GLfloat blackSpecular[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
		GLfloat blackShininess = 0.0f;

		glMaterialfv(GL_FRONT, GL_AMBIENT, blackAmbient);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, blackDiffuse);
		glMaterialfv(GL_FRONT, GL_SPECULAR, blackSpecular);
		glMaterialfv(GL_FRONT, GL_SHININESS, &blackShininess);
	}
	else {
		// Apply texture and material for the base
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, cannonBaseTexture); // Dark gray texture mapping for the base
		glMaterialfv(GL_FRONT, GL_AMBIENT, dark_grey_ambient);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, dark_grey_diffuse);
		glMaterialfv(GL_FRONT, GL_SPECULAR, dark_grey_specular);
		glMaterialfv(GL_FRONT, GL_SHININESS, dark_grey_shininess);
	}

	// Draw cannon base
	glPushMatrix();
	glScalef(6.0, 2.0, 6.0);
	glBegin(GL_QUADS);
	// Front face
	glTexCoord2f(0.0, 0.0); glVertex3f(-0.5, -0.5, 0.5);
	glTexCoord2f(1.0, 0.0); glVertex3f(0.5, -0.5, 0.5);
	glTexCoord2f(1.0, 1.0); glVertex3f(0.5, 0.5, 0.5);
	glTexCoord2f(0.0, 1.0); glVertex3f(-0.5, 0.5, 0.5);

	// Back face
	glTexCoord2f(0.0, 0.0); glVertex3f(-0.5, -0.5, -0.5);
	glTexCoord2f(1.0, 0.0); glVertex3f(0.5, -0.5, -0.5);
	glTexCoord2f(1.0, 1.0); glVertex3f(0.5, 0.5, -0.5);
	glTexCoord2f(0.0, 1.0); glVertex3f(-0.5, 0.5, -0.5);

	// Left face
	glTexCoord2f(0.0, 0.0); glVertex3f(-0.5, -0.5, -0.5);
	glTexCoord2f(1.0, 0.0); glVertex3f(-0.5, -0.5, 0.5);
	glTexCoord2f(1.0, 1.0); glVertex3f(-0.5, 0.5, 0.5);
	glTexCoord2f(0.0, 1.0); glVertex3f(-0.5, 0.5, -0.5);

	// Right face
	glTexCoord2f(0.0, 0.0); glVertex3f(0.5, -0.5, -0.5);
	glTexCoord2f(1.0, 0.0); glVertex3f(0.5, -0.5, 0.5);
	glTexCoord2f(1.0, 1.0); glVertex3f(0.5, 0.5, 0.5);
	glTexCoord2f(0.0, 1.0); glVertex3f(0.5, 0.5, -0.5);

	// Top face
	glTexCoord2f(0.0, 0.0); glVertex3f(-0.5, 0.5, -0.5);
	glTexCoord2f(1.0, 0.0); glVertex3f(0.5, 0.5, -0.5);
	glTexCoord2f(1.0, 1.0); glVertex3f(0.5, 0.5, 0.5);
	glTexCoord2f(0.0, 1.0); glVertex3f(-0.5, 0.5, 0.5);

	// Bottom face
	glTexCoord2f(0.0, 0.0); glVertex3f(-0.5, -0.5, -0.5);
	glTexCoord2f(1.0, 0.0); glVertex3f(0.5, -0.5, -0.5);
	glTexCoord2f(1.0, 1.0); glVertex3f(0.5, -0.5, 0.5);
	glTexCoord2f(0.0, 1.0); glVertex3f(-0.5, -0.5, 0.5);
	glEnd();
	glPopMatrix();

	// Draw barrel
	if (!(cannonDisabled && cannonFadeProgress >= 1.0f)) {
		glBindTexture(GL_TEXTURE_2D, cannonBarrelTexture); // Metallic texture mapping for the barrel
		glMaterialfv(GL_FRONT, GL_AMBIENT, robotLowerBody_mat_ambient);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, robotLowerBody_mat_diffuse);
		glMaterialfv(GL_FRONT, GL_SPECULAR, robotLowerBody_mat_specular);
		glMaterialfv(GL_FRONT, GL_SHININESS, robotLowerBody_mat_shininess);
	}

	glPushMatrix();
	glTranslatef(0.0, 1.0, -4.0);
	GLUquadric* quad = gluNewQuadric();
	gluQuadricTexture(quad, GL_TRUE); // Enable texture mapping for the barrel
	gluCylinder(quad, 1.0, 1.0, 10.0, 16, 16);
	gluDeleteQuadric(quad);
	glPopMatrix();

	glDisable(GL_TEXTURE_2D);
	glPopMatrix();
}

void fireDefensiveCannonProjectile() {
	if (cannonDisabled) return;

	// Calculate cannon position
	float cannonOffsetDistance = 10.0f;
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
	newProjectile.speed = 3.0f; // Speed of the projectile
	newProjectile.active = true;

	defensiveProjectiles.push_back(newProjectile);
}

void updateDefensiveProjectiles() {
	for (size_t i = 0; i < defensiveProjectiles.size(); ++i) {
		if (defensiveProjectiles[i].active) {
			// Update projectile position
			defensiveProjectiles[i].x += defensiveProjectiles[i].directionX * defensiveProjectiles[i].speed;
			defensiveProjectiles[i].y += defensiveProjectiles[i].directionY * defensiveProjectiles[i].speed;
			defensiveProjectiles[i].z += defensiveProjectiles[i].directionZ * defensiveProjectiles[i].speed;

			// Check for collisions with enemy robots
			for (int j = 0; j < robotCount; j++) {
				if (!robots[j].disabled &&
					detectDefensiveLaserCollisionWithBot(defensiveProjectiles[i], robots[j])) {
					// Handle collision
					robots[j].disabled = true;
					robots[j].breakingTimer = 50;
					defensiveProjectiles[i].active = false;
					break;
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
				up = VECTOR3D(1.0f, 0.0f, 0.0f);
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
			gluCylinder(quad, 0.1f, 0.1f, 3.0f, 16, 16);
			gluDeleteQuadric(quad);

			glPopMatrix();
		}
	}
}

void checkCannonHit() {
	for (int i = 0; i < maxProjectiles; i++) {
		if (projectiles[i].active) {
			// Adjust collision bounds for the larger cannon
			if (fabs(projectiles[i].x - cameraX) < 4.0f &&
				fabs(projectiles[i].y - (cameraY - 5.0f)) < 2.0f &&
				fabs(projectiles[i].z - (cameraZ - 10.0f)) < 2.0f) {
				cannonDisabled = true;
				glutTimerFunc(16, updateCannonFade, 0);
				projectiles[i].active = false;
			}
		}
	}
}

void updateCannonFade(int value) {
	if (cannonDisabled && cannonFadeProgress < 1.0f) {
		// Increment fade progress
		cannonFadeProgress += 0.01f;

		// Interpolate RGBA values to fade to black
		for (int i = 0; i < 3; i++) {
			cannonColor[i] = (1.0f - cannonFadeProgress) * green_mat_diffuse[i];
		}
		cannonColor[3] = 1.0f;

		glutPostRedisplay();
		glutTimerFunc(16, updateCannonFade, 0); // Call this function every 16ms (60 FPS)
	}
	else if (cannonDisabled && cannonFadeProgress >= 1.0f) {
		// If the cannon is fully faded to black, mark the game as disabled
		gameDisabled = true;
		glutPostRedisplay();
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
	glEnable(GL_LIGHT1);

	// Other OpenGL setup
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);
	glClearColor(0.4F, 0.4F, 0.4F, 0.0F);
	glClearDepth(1.0f);
	glEnable(GL_NORMALIZE);    // Renormalize normal vectors
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glEnable(GL_TEXTURE_2D);  // Enable texture mapping

	// Set up ground quad mesh
	VECTOR3D origin = VECTOR3D(-160.0f, 0.0f, 160.0f);
	VECTOR3D dir1v = VECTOR3D(1.0f, 0.0f, 0.0f);
	VECTOR3D dir2v = VECTOR3D(0.0f, 0.0f, -1.0f);
	groundMesh = new QuadMesh(meshSize, 3200.0);
	groundMesh->InitMesh(meshSize, origin, 3200.0, 3200.0, dir1v, dir2v);

	VECTOR3D ambient = VECTOR3D(0.0f, 0.05f, 0.0f);
	VECTOR3D diffuse = VECTOR3D(0.4f, 0.8f, 0.4f);
	VECTOR3D specular = VECTOR3D(0.04f, 0.04f, 0.04f);
	float shininess = 0.2;
	groundMesh->SetMaterial(ambient, diffuse, specular, shininess);

	// Create and bind procedural textures
	cannonBarrelTexture = createMetallicTexture();
	cannonBaseTexture = createDarkGrayTexture();

	// Generate textures for robot and cannon
	robotTexture = createEnemyRobotTexture();
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
		glutTimerFunc(16, updateCannonFade, 0);
	}

	// Draw robots
	for (int i = 0; i < robotCount; i++) {
		if (robots[i].disabled) {
			drawRobotWithBreakingAnimation(robots[i]);
		}
		else {
			glPushMatrix();
			glTranslatef(robots[i].xOffset, 0.0, robots[i].zOffset);
			drawRobot();
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
	glTranslatef(0.0, -25.0, 0.0);
	groundMesh->DrawMesh(meshSize);
	glPopMatrix();

	// Display the score
	glPushMatrix();
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, vWidth, 0, vHeight);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glColor3f(1.0f, 1.0f, 1.0f);

	glRasterPos2f(vWidth - 100, vHeight - 30);
	std::ostringstream scoreStream;
	scoreStream << "Score: " << score;
	std::string scoreText = scoreStream.str();
	for (char c : scoreText) {
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, c);
	}

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	// Display "Press R to Reset" if the game is disabled
	if (gameDisabled) {
		glPushMatrix();
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		gluOrtho2D(0, vWidth, 0, vHeight);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glColor3f(1.0f, 0.0f, 0.0f);

		std::string resetMessage = "Press R to Reset";
		glRasterPos2f(vWidth / 2 - 100, vHeight / 2);
		for (char c : resetMessage) {
			glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, c);
		}

		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
	}

	// Swap buffers
	glutSwapBuffers();
}

void drawRobot()
{
	// 1. Draw the lower body separately with no rotation
	glPushMatrix();
	drawLowerBody();
	glPopMatrix();

	// 2. Draw the upper body with rotation
	glPushMatrix();

	// Rotate only the upper body parts
	glRotatef(robotAngle, 0.0, 1.0, 0.0);

	// Draw the upper body components: torso, head, arms
	drawBody();
	drawHead();
	drawLeftArm();
	drawRightArm();

	glPopMatrix();
}

void drawBody() {
	// Top Part (Greyish, wide)
	glPushMatrix();
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, robotTexture);

	// Set material properties for the top part
	glMaterialfv(GL_FRONT, GL_AMBIENT, light_grey_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, light_grey_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, light_grey_diffuse);
	glMaterialf(GL_FRONT, GL_SHININESS, light_grey_shininess);

	// Position and scale the top part
	glTranslatef(0.0, 0.5 * robotBodyLength, 0.0);
	glScalef(robotBodyWidth, robotBodyLength / 3.0, robotBodyDepth);

	// Draw textured cube
	glBegin(GL_QUADS);

	// Front face
	glTexCoord2f(0.0, 0.0); glVertex3f(-0.5, -0.5, 0.5);
	glTexCoord2f(1.0, 0.0); glVertex3f(0.5, -0.5, 0.5);
	glTexCoord2f(1.0, 1.0); glVertex3f(0.5, 0.5, 0.5);
	glTexCoord2f(0.0, 1.0); glVertex3f(-0.5, 0.5, 0.5);

	// Back face
	glTexCoord2f(0.0, 0.0); glVertex3f(-0.5, -0.5, -0.5);
	glTexCoord2f(1.0, 0.0); glVertex3f(0.5, -0.5, -0.5);
	glTexCoord2f(1.0, 1.0); glVertex3f(0.5, 0.5, -0.5);
	glTexCoord2f(0.0, 1.0); glVertex3f(-0.5, 0.5, -0.5);

	// Left face
	glTexCoord2f(0.0, 0.0); glVertex3f(-0.5, -0.5, -0.5);
	glTexCoord2f(1.0, 0.0); glVertex3f(-0.5, -0.5, 0.5);
	glTexCoord2f(1.0, 1.0); glVertex3f(-0.5, 0.5, 0.5);
	glTexCoord2f(0.0, 1.0); glVertex3f(-0.5, 0.5, -0.5);

	// Right face
	glTexCoord2f(0.0, 0.0); glVertex3f(0.5, -0.5, -0.5);
	glTexCoord2f(1.0, 0.0); glVertex3f(0.5, -0.5, 0.5);
	glTexCoord2f(1.0, 1.0); glVertex3f(0.5, 0.5, 0.5);
	glTexCoord2f(0.0, 1.0); glVertex3f(0.5, 0.5, -0.5);

	// Top face
	glTexCoord2f(0.0, 0.0); glVertex3f(-0.5, 0.5, -0.5);
	glTexCoord2f(1.0, 0.0); glVertex3f(0.5, 0.5, -0.5);
	glTexCoord2f(1.0, 1.0); glVertex3f(0.5, 0.5, 0.5);
	glTexCoord2f(0.0, 1.0); glVertex3f(-0.5, 0.5, 0.5);

	// Bottom face
	glTexCoord2f(0.0, 0.0); glVertex3f(-0.5, -0.5, -0.5);
	glTexCoord2f(1.0, 0.0); glVertex3f(0.5, -0.5, -0.5);
	glTexCoord2f(1.0, 1.0); glVertex3f(0.5, -0.5, 0.5);
	glTexCoord2f(0.0, 1.0); glVertex3f(-0.5, -0.5, 0.5);

	glEnd();

	glPopMatrix();
	glDisable(GL_TEXTURE_2D);

	// Middle Part (Light gray with panel-like texture)
	glPushMatrix();
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, robotTexture);

	// Set material properties for the middle part
	glMaterialfv(GL_FRONT, GL_AMBIENT, light_grey_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, light_grey_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, light_grey_diffuse);
	glMaterialf(GL_FRONT, GL_SHININESS, light_grey_shininess);

	// Position and scale the middle part
	glTranslatef(0.0, 0.0, 0.0);
	glScalef(0.4 * robotBodyWidth, robotBodyLength / 2.0, 0.4 * robotBodyDepth);

	// Draw the textured middle part
	glutSolidCube(1.0);

	glPopMatrix();
	glDisable(GL_TEXTURE_2D);
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
	glRotatef(neckAngle, 0.0, 1.0, 0.0);
	glTranslatef(0, 0.5 * robotBodyLength + 1.0 * headLength, 0);

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
	glTranslatef(-0.2 * robotBodyWidth, 0, 0);
	glScalef(0.01 * robotBodyWidth, 0.4 * robotBodyWidth, 0.4 * robotBodyWidth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Draw right side of the head
	glPushMatrix();
	glTranslatef(0.2 * robotBodyWidth, 0, 0);
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
	glTranslatef(0.0, 0.06 * robotBodyWidth, 0.20 * robotBodyWidth);
	glScalef(0.12 * robotBodyWidth, 0.3 * robotBodyWidth, 0.03 * robotBodyWidth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Add grey part to the top of the head
	glPushMatrix();
	glTranslatef(0.0, 0.2 * robotBodyWidth, 0.01 * robotBodyWidth);
	glScalef(0.12 * robotBodyWidth, 0.02 * robotBodyWidth, 0.42 * robotBodyWidth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Blue eye (upright visor-like stripe)
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, cyan_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, cyan_diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, cyan_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, cyan_shininess);

	// Position the blue visor/eye on the front
	glTranslatef(0.0, 0.1 * robotBodyWidth, 0.22 * robotBodyWidth);
	glScalef(0.05 * robotBodyWidth, 0.2 * robotBodyWidth, 0.02 * robotBodyWidth);
	glutSolidCube(1.0);
	glPopMatrix();

	glPopMatrix();
}

void drawLowerBody() {
	// Lower body section
	glPushMatrix();
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, robotTexture);

	// Set material properties for the lower body
	glMaterialfv(GL_FRONT, GL_AMBIENT, light_grey_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, light_grey_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, light_grey_diffuse);
	glMaterialf(GL_FRONT, GL_SHININESS, light_grey_shininess);

	// Position the lower body (stationary part under the main body)
	glTranslatef(0.0, -0.5 * robotBodyLength, 0.0);
	glScalef(0.8 * robotBodyWidth, robotBodyLength / 3.0, 0.8 * robotBodyDepth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Left leg
	glPushMatrix();
	glTranslatef(0.5 * robotBodyWidth, -0.7 * robotBodyLength, 0.0);
	glRotatef(hipAngleLeft, 1.0, 0.0, 0.0);

	// Upper left leg
	glPushMatrix();
	glScalef(0.2 * robotBodyWidth, 0.5 * robotBodyLength, 0.2 * robotBodyDepth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Kneecap
	glPushMatrix();
	glTranslatef(0.0, -0.25 * robotBodyLength, 0.10 * robotBodyDepth);
	glScalef(0.25 * robotBodyWidth, 0.1 * robotBodyLength, 0.25 * robotBodyDepth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Lower left leg
	glTranslatef(0.0, -0.5 * robotBodyLength, 0.0);
	glRotatef(kneeAngleLeft, 1.0, 0.0, 0.0);
	glPushMatrix();
	glScalef(0.2 * robotBodyWidth, 0.5 * robotBodyLength, 0.2 * robotBodyDepth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Ankle
	glTranslatef(0.0, -0.5 * robotBodyLength, 0.0);
	glRotatef(ankleAngleLeft, 1.0, 0.0, 0.0);

	// Foot
	glPushMatrix();
	glScalef(0.4 * robotBodyDepth, 0.1 * robotBodyLength, 0.6 * robotBodyWidth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Foot dents
	// Front left dent
	glPushMatrix();
	glTranslatef(-0.15 * robotBodyDepth, 0.0, 0.4 * robotBodyWidth);
	glScalef(0.1 * robotBodyDepth, 0.1 * robotBodyLength, 0.2 * robotBodyWidth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Front right dent
	glPushMatrix();
	glTranslatef(0.15 * robotBodyDepth, 0.0, 0.4 * robotBodyWidth);
	glScalef(0.1 * robotBodyDepth, 0.1 * robotBodyLength, 0.2 * robotBodyWidth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Back left dent
	glPushMatrix();
	glTranslatef(-0.15 * robotBodyDepth, 0.0, -0.4 * robotBodyWidth);
	glScalef(0.1 * robotBodyDepth, 0.1 * robotBodyLength, 0.2 * robotBodyWidth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Back right dent
	glPushMatrix();
	glTranslatef(0.15 * robotBodyDepth, 0.0, -0.4 * robotBodyWidth);
	glScalef(0.1 * robotBodyDepth, 0.1 * robotBodyLength, 0.2 * robotBodyWidth);
	glutSolidCube(1.0);
	glPopMatrix();

	glPopMatrix(); // End left leg

	// Right leg
	glPushMatrix();
	glTranslatef(-0.5 * robotBodyWidth, -0.7 * robotBodyLength, 0.0);
	glRotatef(hipAngleRight, 1.0, 0.0, 0.0);

	// Upper right leg
	glPushMatrix();
	glScalef(0.2 * robotBodyWidth, 0.5 * robotBodyLength, 0.2 * robotBodyDepth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Kneecap
	glPushMatrix();
	glTranslatef(0.0, -0.25 * robotBodyLength, 0.10 * robotBodyDepth);
	glScalef(0.25 * robotBodyWidth, 0.1 * robotBodyLength, 0.25 * robotBodyDepth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Lower right leg
	glTranslatef(0.0, -0.5 * robotBodyLength, 0.0);
	glRotatef(kneeAngleRight, 1.0, 0.0, 0.0);
	glPushMatrix();
	glScalef(0.2 * robotBodyWidth, 0.5 * robotBodyLength, 0.2 * robotBodyDepth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Ankle
	glTranslatef(0.0, -0.5 * robotBodyLength, 0.0);
	glRotatef(ankleAngleRight, 1.0, 0.0, 0.0);

	// Foot
	glPushMatrix();
	glScalef(0.4 * robotBodyDepth, 0.1 * robotBodyLength, 0.6 * robotBodyWidth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Foot dents
	// Front left dent
	glPushMatrix();
	glTranslatef(-0.15 * robotBodyDepth, 0.0, 0.4 * robotBodyWidth);
	glScalef(0.1 * robotBodyDepth, 0.1 * robotBodyLength, 0.2 * robotBodyWidth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Front right dent
	glPushMatrix();
	glTranslatef(0.15 * robotBodyDepth, 0.0, 0.4 * robotBodyWidth);
	glScalef(0.1 * robotBodyDepth, 0.1 * robotBodyLength, 0.2 * robotBodyWidth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Back left dent
	glPushMatrix();
	glTranslatef(-0.15 * robotBodyDepth, 0.0, -0.4 * robotBodyWidth);
	glScalef(0.1 * robotBodyDepth, 0.1 * robotBodyLength, 0.2 * robotBodyWidth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Back right dent
	glPushMatrix();
	glTranslatef(0.15 * robotBodyDepth, 0.0, -0.4 * robotBodyWidth);
	glScalef(0.1 * robotBodyDepth, 0.1 * robotBodyLength, 0.2 * robotBodyWidth);
	glutSolidCube(1.0);
	glPopMatrix();

	glPopMatrix();

	glDisable(GL_TEXTURE_2D);
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
	glTranslatef(0.5 * robotBodyWidth + 0.5 * upperArmWidth, 0.3 * robotBodyLength, 0.0);

	// Draw upper arm (green part)
	glPushMatrix();
	glScalef(upperArmWidth, 0.6 * upperArmLength, upperArmWidth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Add the elbow joint (grey part)
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, dark_grey_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, dark_grey_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, dark_grey_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, dark_grey_shininess);

	// Position and scale the elbow
	glTranslatef(0.0, -0.5 * 0.6 * upperArmLength, 0.0);
	glScalef(1.2 * upperArmWidth, 0.1 * upperArmLength, 1.2 * upperArmWidth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Move down for the lower arm and translate further forward along Z-axis
	glTranslatef(0.0, -0.9 * 0.6 * upperArmLength, 1.1);

	// Apply rotation to the lower arm for an angled effect
	glRotatef(-30.0, 1.0, 0.0, 0.0);

	// Draw lower arm (green part)
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, green_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, green_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, green_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, green_mat_shininess);

	glScalef(upperArmWidth, 0.6 * upperArmLength, upperArmWidth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Now draw the hand
	glMaterialfv(GL_FRONT, GL_AMBIENT, dark_grey_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, dark_grey_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, dark_grey_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, dark_grey_shininess);

	// Position the hand slightly above and more inside the lower arm
	glPushMatrix();
	glTranslatef(0.0, -0.35 * 0.6 * upperArmLength - 0.15, 0.0);
	glScalef(0.7 * upperArmWidth, 0.5 * upperArmLength, 0.7 * upperArmWidth);
	glutSolidCube(1.0);

	// Add the fingers
	float fingerWidth = 0.06 * upperArmWidth;
	float fingerLength = 0.05 * upperArmLength;

	// Draw 5 fingers
	for (int i = -2; i <= 2; i++) {
		glPushMatrix();
		glTranslatef(i * (0.12 * upperArmWidth), -0.2 * (0.3 * upperArmLength), 0.0);
		glScalef(fingerWidth, fingerLength, fingerWidth);
		glutSolidCube(1.0);
		glPopMatrix();
	}

	glPopMatrix();
	glPopMatrix();
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
	glTranslatef(-(0.5 * robotBodyWidth + 0.5 * upperArmWidth), 0.3 * robotBodyLength, 0.2 * robotBodyDepth);

	glRotatef(-45.0, 1.0, 0.0, 0.0);

	// Draw upper arm (green part)
	glPushMatrix();
	glScalef(upperArmWidth, 0.6 * upperArmLength, upperArmWidth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Add the brown elbow joint
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, dark_grey_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, dark_grey_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, dark_grey_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, dark_grey_shininess);

	// Position and scale the elbow
	glTranslatef(0.0, -0.5 * 0.6 * upperArmLength, 0.0);
	glScalef(1.2 * upperArmWidth, 0.1 * upperArmLength, 1.2 * upperArmWidth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Move down for the lower arm, starting from the elbow
	glTranslatef(0.0, -0.75 * 0.8 * upperArmLength, 1.3);

	// Apply rotation to the lower arm for an angled effect
	glRotatef(-25.0, 1.0, 0.0, 0.0);

	// Draw lower arm (green part)
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, green_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, green_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, green_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, green_mat_shininess);

	glScalef(upperArmWidth, 0.7 * upperArmLength, upperArmWidth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Now handle the cannon attached to the lower arm
	glMaterialfv(GL_FRONT, GL_AMBIENT, dark_grey_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, dark_grey_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, dark_grey_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, dark_grey_shininess);

	// Position the cannon at the end of the lower arm
	glPushMatrix();
	glTranslatef(0.0, -0.4 * upperArmLength - 0.4 * gunLength, 0.0);

	// Apply cannon spin along its Y-axis (screw-like spin)
	if (spinCannon) {
		glRotatef(cannonSpinAngle, 0.0, 1.0, 0.0);
	}

	// Draw the gun (cannon body)
	glPushMatrix();
	glScalef(gunWidth, gunLength, gunDepth);
	glutSolidCube(1.0);
	glPopMatrix();

	// Draw the cannon barrel (cylinder)
	glPushMatrix();
	glTranslatef(0.0, -0.5 * gunLength, 0.0);
	glRotatef(90.0, 1.0, 0.0, 0.0);
	GLUquadric* quad = gluNewQuadric();
	gluCylinder(quad, 1.5, 1.5, 5.0, 40, 20);
	glPopMatrix();

	// Draw the orange projectile inside the cannon
	glPushMatrix();
	glMaterialfv(GL_FRONT, GL_AMBIENT, red_orange_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, red_orange_diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, red_orange_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, red_orange_shininess);
	glTranslatef(0.0, -2.5 * gunLength, 0.0);
	glScalef(gunWidth * 0.5, gunLength * 0.1, gunDepth * 0.5);
	glutSolidCube(1.0);
	glPopMatrix();

	// Draw the magazine under the cannon
	glPushMatrix();
	glTranslatef(0.0, -gunLength - 1.0, 0.0);
	glScalef(gunWidth * 0.8, gunLength * 0.4, gunDepth * 0.8);
	glutSolidCube(1.0);
	glPopMatrix();

	glPopMatrix();
	glPopMatrix();
}

void reshape(int w, int h)
{
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, (GLdouble)w / h, 0.2, 500.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt(0.0, 6.0, 35.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
}

void moveRobots(int value) {
	int disabledCount = 0;

	for (int i = 0; i < robotCount; i++) {
		if (robots[i].disabled) {
			if (robots[i].breakingTimer > 0) {
				robots[i].breakingTimer--;
			}
			else {
				robots[i].xOffset = 1000.0f;
				robots[i].zOffset = 1000.0f;
			}
			disabledCount++;
			continue;
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

	// If all robots are disabled, reset them
	if (disabledCount == robotCount) {
		resetRobots();
	}

	glutPostRedisplay();
	glutTimerFunc(16, moveRobots, 0);
}

bool stop = false;

void stepAnimation(int value)
{
	if (walking) {
		for (int i = 0; i < robotCount; i++) {
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
		cannonSpinAngle += 5.0f;
		if (cannonSpinAngle > 360.0f) {
			cannonSpinAngle -= 360.0f;
		}
		glutPostRedisplay();
		glutTimerFunc(10, cannonAnimation, 0);
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
	case ' ':  // Spacebar to fire projectile
		fireDefensiveCannonProjectile();
		break;
	case 'r': // "R" key to reset the game
		if (gameDisabled) {
			gameDisabled = false;
			resetApplication();
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

void handleMouseMotion(int x, int y) {
	static int lastX = x, lastY = y;

	// Adjust yaw (horizontal) and pitch (vertical) based on mouse movement
	float sensitivity = 0.1f;
	float deltaX = (x - lastX) * sensitivity;
	float deltaY = (y - lastY) * sensitivity;

	cameraYaw += deltaX;
	cameraPitch -= deltaY;

	// Adjust the range of cameraYaw to allow further left/right movement
	const float maxYaw = 180.0f;
	if (cameraYaw > maxYaw) cameraYaw = maxYaw;
	if (cameraYaw < -maxYaw) cameraYaw = -maxYaw;

	// Clamp the pitch to avoid excessive tilt
	const float maxPitch = 45.0f;
	if (cameraPitch > maxPitch) cameraPitch = maxPitch;
	if (cameraPitch < -maxPitch) cameraPitch = -maxPitch;

	// Sync cannon's barrel angles with camera's orientation
	barrelYawAngle = -cameraYaw;
	barrelTiltAngle = cameraPitch;

	// Save current mouse position
	lastX = x;
	lastY = y;

	glutPostRedisplay();
}
