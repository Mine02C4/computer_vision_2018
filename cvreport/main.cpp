#include "main.hpp"

#include <iostream>
#include <vector>
#include <iterator>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "videotest.hpp"
#include "Calibration.hpp"
#include "Stereo.hpp"


int main(int argc, char** argv)
{
	using namespace cv;
	using namespace std;
	int mode;
	cout << "Select mode" << endl;
	cout << "1: Feature detection test" << endl;
	cout << "2: Camera calibration(single)" << endl;
	cout << "3: Stereo camera calibration" << endl;
	cout << "4: Object detection by color" << endl;
	cin >> mode;
	switch (mode)
	{
	case 1:
	{
		VideoTest* vt = new VideoTest();
		vt->Init();

		while (1) {
			vt->ReadFrame();
			vt->TrackFeatures();
			int key = waitKey(1) & 0xff;
			if (key == kOPENCV_KEY_ESCAPE) {
				break;
			}
		}
		delete vt;
		break;
	}
	case 2:
	{
		SingleCalibration* test = new SingleCalibration();
		test->Run();
		break;
	}
	case 3:
	{
		StereoTest* test = new StereoTest();
		test->Run();
		break;
	}
	case 4:
	{
		VideoTest vt;
		vt.Init();
		while (1) {
			vt.ReadFrame();
			vt.DetectionByColor();
			int key = waitKey(1) & 0xff;
			if (key == kOPENCV_KEY_ESCAPE) {
				break;
			}
		}
		break;
	}
	case 5:
	{
		// TODO
		break;
	}
	default:
		break;
	}

	return 0;
}
