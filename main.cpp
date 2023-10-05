#include "pch.h"
#include "tester.h"

int main()
{
	////random seed////
	srand(time(0));

	////test in blensor simulation////
	testBlensor();

	////test in real-world////
	//findCameraIntrinsicInReal();
	//calibrateInReal();
	//calibrateInReal2();

	////pause////
	system("pause");
}
