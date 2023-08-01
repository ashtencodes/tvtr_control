
//-------------------------------MPU6050 Accelerometer and Gyroscope C++ library-----------------------------
//Copyright (c) 2019, Alex Mous
//Licensed under the CC BY-NC SA 4.0

//Example code

#include <MPU6050.h>
#include <stdio.h>

MPU6050 device(0x68, 1);
float axOffset;
float ayOffset;
float azOffset;
float gr_offset;
float gp_offset;
float gy_offset;

void CalibrateIMU(MPU6050& imu);

int main() {
	float ax, ay, az, gr, gp, gy; //Variables to store the accel, gyro and angle values

	sleep(1); //Wait for the MPU6050 to stabilize

/*
	//Calculate the offsets
	std::cout << "Calculating the offsets...\n    Please keep the accelerometer level and still\n    This could take a couple of minutes...";
	device.getOffsets(&ax, &ay, &az, &gr, &gp, &gy);
	std::cout << "Gyroscope R,P,Y: " << gr << "," << gp << "," << gy << "\nAccelerometer X,Y,Z: " << ax << "," << ay << "," << az << "\n";
*/

	//Read the current yaw angle
	device.calc_yaw = true;

	CalibrateIMU(device);

	for (int i = 0; i < 100000; i++) {
		// std::cout << "Current angle around the roll axis: " << gr << "\n";
		// std::cout << "Current angle around the pitch axis: " << gp << "\n";
		// std::cout << "Current angle around the yaw axis: " << gy << "\n";
		// device.getGyro(&gr, &gp, &gy);
		// std::cout << "Gyroscope Readings: X: " << gr << ", Y: " << gp << ", Z: " << gy << "\n";
		device.getAccel(&ax, &ay, &az);
		device.getGyro(&gr, &gp, &gy);
		// std::cout << "Accelerometer Readings: X: " << ax << ", Y: " << ay << ", Z: " << az << "\n";
		printf("Accelerometer Readings: X: % 05.3f, Y: % 05.3f, Z: % 05.3f\n", ax - axOffset, ay - ayOffset, az - azOffset);
		printf("Gyro Readings: X: % 04.1f, Y: % 04.1f, Z: % 04.1f\n", gr - gr_offset, gp - gp_offset, gy - gy_offset);
		usleep(25000); //0.025sec
	}

	//Get the current accelerometer values
	device.getAccel(&ax, &ay, &az);
	std::cout << "Accelerometer Readings: X: " << ax << ", Y: " << ay << ", Z: " << az << "\n";

	//Get the current gyroscope values
	device.getGyro(&gr, &gp, &gy);
	std::cout << "Gyroscope Readings: X: " << gr << ", Y: " << gp << ", Z: " << gy << "\n";

	return 0;
}

void CalibrateIMU(MPU6050& imu)
{
	// float ax, ay, az, gr, gp, gy; //Variables to store the accel, gyro and angle values
	float ax[1000], ay[1000], az[1000], gr[1000], gp[1000], gy[1000];
	for (int i = 0; i < 1000; i++)
	{
		// std::cout << "Current angle around the roll axis: " << gr << "\n";
		// std::cout << "Current angle around the pitch axis: " << gp << "\n";
		// std::cout << "Current angle around the yaw axis: " << gy << "\n";
		// device.getGyro(&gr, &gp, &gy);
		// std::cout << "Gyroscope Readings: X: " << gr << ", Y: " << gp << ", Z: " << gy << "\n";
		imu.getAccel(&ax[i], &ay[i], &az[i]);
		imu.getGyro(&gr[i], &gp[i], &gy[i]);
		// std::cout << "Accelerometer Readings: X: " << ax << ", Y: " << ay << ", Z: " << az << "\n";
		printf("Accelerometer Readings: X: %.3f, Y: %.3f, Z: %.3f\n", ax[i], ay[i], az[i]);
		printf("Gyro Readings: X: %.3f, Y: %.3f, Z: %.3f\n", gr[i], gp[i], gy[i]);
		usleep(100); //0.001sec
	}

	printf("Calculating Offsets\n");
	float axSum = 0.0f;
	float aySum = 0.0f;
	float azSum = 0.0f;
	float grSum = 0.0f;
	float gpSum = 0.0f;
	float gySum = 0.0f;
	for (int i = 0; i < 1000; i++)
	{
		axSum += ax[i];
		aySum += ay[i];
		azSum += az[i];
		grSum += gr[i];
		gpSum += gp[i];
		gySum += gy[i];
	}
	axOffset = axSum / 1000.0f;
	ayOffset = aySum / 1000.0f;
	azOffset = azSum / 1000.0f;
	gr_offset = grSum / 1000.0f;
	gp_offset = gpSum / 1000.0f;
	gy_offset = gySum / 1000.0f;
}