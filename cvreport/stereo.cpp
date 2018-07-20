#include "Stereo.hpp"

#include <filesystem>
#include <iostream>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <GL/freeglut.h>

#include "calibration.hpp"
#include "camera_manager.hpp"
#include "image_manager.hpp"

using namespace cv;
using namespace std;

int Stereo::CalcDisparity(cv::Mat& left_img, cv::Mat& right_img,
                          cv::Mat& disparity) {
  auto image_size = left_img.size();
#define CALIB_MODE_SGBM
  int SADWindowSize = 3;
  int minDisparity = 0;
  int numberOfDisparities = 0;
  numberOfDisparities = numberOfDisparities > 0
                            ? numberOfDisparities
                            : ((image_size.width / 8) + 15) & -16;
  int blockSize = SADWindowSize > 0 ? SADWindowSize : 9;
#ifdef CALIB_MODE_SGBM
  auto sgbm =
      StereoSGBM::create(minDisparity, numberOfDisparities, SADWindowSize);
  sgbm->setPreFilterCap(63);
  sgbm->setBlockSize(blockSize);
  int cn = 1;
  sgbm->setP1(8 * cn * blockSize * blockSize);
  sgbm->setP2(32 * cn * blockSize * blockSize);
  sgbm->setMinDisparity(0);
  sgbm->setNumDisparities(numberOfDisparities);
  sgbm->setUniquenessRatio(10);
  sgbm->setSpeckleWindowSize(100);
  sgbm->setSpeckleRange(32);
  sgbm->setDisp12MaxDiff(1);
  sgbm->setMode(StereoSGBM::MODE_SGBM);
  sgbm->compute(left_img, right_img, disparity);
#else
  Mat left_gray, right_gray;
  cvtColor(right_img, right_gray, COLOR_BGR2GRAY);
  cvtColor(left_img, left_gray, COLOR_BGR2GRAY);
  auto bm = StereoBM::create();
  bm->setROI1(validRoi[0]);
  bm->setROI2(validRoi[1]);
  bm->setPreFilterCap(31);
  bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
  bm->setMinDisparity(0);
  bm->setNumDisparities(numberOfDisparities);
  bm->setTextureThreshold(10);
  bm->setUniquenessRatio(15);
  bm->setSpeckleWindowSize(100);
  bm->setSpeckleRange(32);
  bm->setDisp12MaxDiff(1);
  bm->compute(left_gray, right_gray, disparity);
#endif
  return numberOfDisparities;
}

void Stereo::RenderDisparity(Mat& disparity, int numberOfDisparities) {
  Mat disparity8;
  disparity.convertTo(disparity8, CV_8U, 255 / (numberOfDisparities * 16.0));
  imshow("disparity", disparity8);
}

Stereo* ste = NULL;

static int drag_mouse_r = 0;
static int drag_mouse_l = 0;
static int last_mouse_x;
static int last_mouse_y;

static float viewpoint_x = 3.0;
static float viewpoint_y = 40.0;
static float viewpoint_z = 50.0;
static float sight_x = 0.0;
static float sight_y = 0.0;
static float sight_z = 0.0;
static float upper_x = 0.0;
static float upper_y = 0.0;
static float upper_z = 1.0;

static cv::Mat currentPose =
    (cv::Mat_<float>(3, 4) << 1.0f, 0.0f, -3.0f, 3.0f, 0.0f, 0.0f, -40.0f,
     40.0f, 0.0f, -1.0f, -50.0f, 50.0f);

void render() {
  if (ste != NULL) {
    ste->RenderGLWindow();
  }
}

void setCurrentCameraPose() {
  viewpoint_x = currentPose.at<float>(0, 3);
  viewpoint_y = currentPose.at<float>(1, 3);
  viewpoint_z = currentPose.at<float>(2, 3);
  sight_x = viewpoint_x + currentPose.at<float>(0, 2);
  sight_y = viewpoint_y + currentPose.at<float>(1, 2);
  sight_z = viewpoint_z + currentPose.at<float>(2, 2);
  upper_x = -1.0f * currentPose.at<float>(0, 1);
  upper_y = -1.0f * currentPose.at<float>(1, 1);
  upper_z = -1.0f * currentPose.at<float>(2, 1);
}

void Stereo::RenderGLWindow() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glPolygonMode(GL_FRONT, GL_FILL);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  gluLookAt(viewpoint_x, viewpoint_y, viewpoint_z, sight_x, sight_y, sight_z,
            upper_x, upper_y, upper_z);

  glEnable(GL_DEPTH_TEST);

  // axis
  glLineWidth(5.0f);
  glBegin(GL_LINES);
  glColor3f(1.0f, 0.0f, 0.0f);  // x axis(Red)
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(200.0f, 0.0f, 0.0f);
  glColor3f(0.0f, 1.0f, 0.0f);  // y axis(Green)
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 200.0f, 0.0f);
  glColor3f(0.0f, 0.0f, 1.0f);  // z axis(Blue)
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, 200.0f);
  glEnd();

  glColor3f(0.8f, 0.0f, 0.8f);
  glPointSize(2.0f);
  glBegin(GL_POINTS);
  for (int i = 0; i < img3d_.rows; ++i) {
    for (int j = 0; j < img3d_.cols; ++j) {
      Vec3f p = img3d_.at<Vec3f>(i, j);
      if (!isinf(p[0])) {
        glVertex3f(p[0], p[1], p[2]);
      }
    }
  }
  glEnd();
  glutSwapBuffers();
}

void Stereo::SetData(cv::Mat& img3d) {
  img3d_ = img3d.clone();
  render();
}

std::string StereoTest::GetLeftImagePath(int i) {
  return ImageManager::GetImagePath("stereo_cboard_" + to_string(i) + "_l.png");
}

std::string StereoTest::GetRightImagePath(int i) {
  return ImageManager::GetImagePath("stereo_cboard_" + to_string(i) + "_r.png");
}

void StereoTest::Run() {
  const int BOARD_W = 10;
  const int BOARD_H = 7;
  const Size BOARD_SIZE = Size(BOARD_W, BOARD_H);
  const int N_BOARDS = 10;
  const float SCALE = 23;

  vector<Mat> src_image(N_BOARDS * 2);
  bool files_exist = true;
  for (int i = 0; i < N_BOARDS; i++) {
    src_image[i * 2] = imread(GetRightImagePath(i));
    if (src_image[i * 2].data == NULL) {
      files_exist = false;
      for (i = i * 2 - 1; i >= 0; --i) {
        src_image[i].release();
      }
      break;
    }
    src_image[i * 2 + 1] = imread(GetLeftImagePath(i));
    if (src_image[i * 2 + 1].data == NULL) {
      files_exist = false;
      for (i = i * 2; i >= 0; --i) {
        src_image[i].release();
      }
      break;
    }
  }

  vector<vector<Point2f> > imagePoints[2];
  vector<vector<Point3f> > objectPoints;
  vector<Point3f> objectCorners;
  for (int j = 0; j < BOARD_H; j++) {
    for (int i = 0; i < BOARD_W; i++) {
      objectCorners.push_back(Point3f(i * SCALE, j * SCALE, 0.0f));
    }
  }
  Mat E, F;

  if (files_exist) {
    int input;
    cout << "Stereo chessboard images are already exist." << endl;
    cout << "1: Use the images" << endl;
    cout << "2: Take new images" << endl;
    cin >> input;
    switch (input) {
      case 1: {
        for (int i = 0; i < N_BOARDS * 2; i++) {
          vector<Point2f> imageCorners;
          Mat dst_image, gray_image;
          bool found = Calibration::DetectChessboardCorners(
              src_image[i], BOARD_SIZE, imageCorners);
          assert(found);
          imagePoints[i % 2].push_back(imageCorners);
        }
        break;
      }
      case 2:
        for (int i = 0; i < N_BOARDS * 2; ++i) {
          src_image[i].release();
        }
        files_exist = false;
        break;
      default:
        cerr << "Invalid input" << endl;
        exit(1);
    }
  }

  if (!files_exist) {
    Mat frame0, frame1;
    VideoCapture cap[2];
    int right_id, left_id;
    cout << "Input right camera ID: ";
    cin >> right_id;
    cout << "Input left camera ID: ";
    cin >> left_id;
    cap[0].open(right_id);
    cap[1].open(left_id);
    if (!cap[0].isOpened() || !cap[1].isOpened()) {
      cerr << "Cannot open camera." << endl;
      exit(1);
    }
    for (int i = 0; i < N_BOARDS; i++) {
      bool capture_phase = true;
      while (capture_phase) {
        cap[0] >> frame0;
        cap[1] >> frame1;
        imshow("chessboard camera Right", frame0);
        imshow("chessboard camera Left", frame1);
        int key = waitKey(10) & 0xff;
        switch (key) {
          case kOPENCV_KEY_ENTER:  // Enter
            src_image[i * 2] = frame0.clone();
            src_image[i * 2 + 1] = frame1.clone();
            capture_phase = false;
            break;
          case 27:  // Esc
            cout << "Canceled" << endl;
            return;
          default:
            break;
        }
      }
      {
        vector<Point2f> imageCorners[2];
        Mat cboard_preview[2];
        bool found[2];
        try {
          found[0] = Calibration::DetectChessboardCorners(
              src_image[i * 2], BOARD_SIZE, imageCorners[0]);
          found[1] = Calibration::DetectChessboardCorners(
              src_image[i * 2 + 1], BOARD_SIZE, imageCorners[1]);
        } catch (const std::exception& ex) {
          cerr << ex.what() << endl;
          --i;
          continue;
        }
        cboard_preview[0] = src_image[i * 2].clone();
        cboard_preview[1] = src_image[i * 2 + 1].clone();
        drawChessboardCorners(cboard_preview[0], BOARD_SIZE, imageCorners[0],
                              found);
        drawChessboardCorners(cboard_preview[1], BOARD_SIZE, imageCorners[1],
                              found);
        imshow("chessboard camera Right", cboard_preview[0]);
        imshow("chessboard camera Left", cboard_preview[1]);
        int key = waitKey(0) & 0xff;
        switch (key) {
          case kOPENCV_KEY_ENTER:  // Enter
            if (found[0] && found[1]) {
              imwrite(GetRightImagePath(i), src_image[i * 2]);
              imwrite(GetLeftImagePath(i), src_image[i * 2 + 1]);
              imagePoints[0].push_back(imageCorners[0]);
              imagePoints[1].push_back(imageCorners[1]);
            } else {
              --i;
            }
            break;
          case 32:  // Space
            --i;
            break;
          case 27:  // Esc
            cout << "Canceled" << endl;
            return;
          default:
            break;
        }
      }
    }
  }

  // Mat cameraMatrix[2], distCoeffs[2];
  // Mat R1, R2, P1, P2, Q;
  // Rect validRoi[2];
  StereoIntrinsicsParameter i_param("bf");
  StereoExtrinsicsParameter e_param("bf");
  bool calibrated = false;
  if (files_exist) {
    if (i_param.ReadFile() && e_param.ReadFile()) {
      int input;
      cout << "Calibration data are already exist." << endl;
      cout << "1: Use the calibration data" << endl;
      cout << "2: Calibration again" << endl;
      cin >> input;
      switch (input) {
        case 1: {
          calibrated = true;
          break;
        }
        case 2:
          break;
        default:
          cerr << "Invalid input" << endl;
          exit(1);
      }
    }
  }
  Size imageSize = src_image[0].size();
  if (!calibrated) {
    CameraParameter right_cp("calibration_r");
    CameraParameter left_cp("calibration_l");
    if (!left_cp.ReadFile() || !right_cp.ReadFile()) {
      cerr << "Cannot open calibration XML" << endl;
      exit(1);
    }
    i_param.SetByCameraParameters(left_cp, right_cp);

    for (int i = 0; i < N_BOARDS; i++) {
      objectPoints.push_back(objectCorners);
    }
    double rms = stereoCalibrate(
        objectPoints, imagePoints[0], imagePoints[1], i_param.cameraMatrix[0],
        i_param.distCoeffs[0], i_param.cameraMatrix[1], i_param.distCoeffs[1],
        imageSize, e_param.R, e_param.T, E, F,
        CALIB_FIX_ASPECT_RATIO + CALIB_ZERO_TANGENT_DIST + CALIB_FIX_INTRINSIC +
            CALIB_SAME_FOCAL_LENGTH + CALIB_RATIONAL_MODEL + CALIB_FIX_K3 +
            CALIB_FIX_K4 + CALIB_FIX_K5,
        TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
    cout << "RMS error = " << rms << endl;

    // Error check
    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for (int i = 0; i < N_BOARDS; i++) {
      int npt = (int)imagePoints[0][i].size();
      Mat imgpt[2];
      for (int k = 0; k < 2; k++) {
        imgpt[k] = Mat(imagePoints[k][i]);
        undistortPoints(imgpt[k], imgpt[k], i_param.cameraMatrix[k],
                        i_param.distCoeffs[k], Mat(), i_param.cameraMatrix[k]);
        computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);
      }
      for (int j = 0; j < npt; j++) {
        double errij =
            fabs(imagePoints[0][i][j].x * lines[1][j][0] +
                 imagePoints[0][i][j].y * lines[1][j][1] + lines[1][j][2]) +
            fabs(imagePoints[1][i][j].x * lines[0][j][0] +
                 imagePoints[1][i][j].y * lines[0][j][1] + lines[0][j][2]);
        err += errij;
      }
      npoints += npt;
    }
    cout << "average epipolar err = " << err / npoints << endl;

    // save intrinsic parameters
    FileStorage fs("intrinsics.xml", FileStorage::WRITE);
    if (fs.isOpened()) {
      fs << "M1" << i_param.cameraMatrix[0] << "D1" << i_param.distCoeffs[0]
         << "M2" << i_param.cameraMatrix[1] << "D2" << i_param.distCoeffs[1];
      fs.release();
    } else {
      cerr << "Error: can not save the intrinsic parameters" << endl;
    }

    stereoRectify(i_param.cameraMatrix[0], i_param.distCoeffs[0],
                  i_param.cameraMatrix[1], i_param.distCoeffs[1], imageSize,
                  e_param.R, e_param.T, e_param.R1, e_param.R2, e_param.P1,
                  e_param.P2, e_param.Q, CALIB_ZERO_DISPARITY, 1, imageSize,
                  &e_param.validRoi[0], &e_param.validRoi[1]);

    fs.open("extrinsics.xml", FileStorage::WRITE);
    if (fs.isOpened()) {
      fs << "R" << e_param.R << "T" << e_param.T << "R1" << e_param.R1 << "R2"
         << e_param.R2 << "P1" << e_param.P1 << "P2" << e_param.P2 << "Q"
         << e_param.Q;
      fs << "vroi0" << e_param.validRoi[0];
      fs << "vroi1" << e_param.validRoi[1];
      fs.release();
    } else {
      cerr << "Error: can not save the extrinsic parameters" << endl;
    }
  }

  // OpenCV can handle left-right
  // or up-down camera arrangements
  bool isVerticalStereo =
      fabs(e_param.P2.at<double>(1, 3)) > fabs(e_param.P2.at<double>(0, 3));

  Mat rmap[2][2];
  // IF BY CALIBRATED (BOUGUET'S METHOD)
  bool useCalibrated = false;
  if (useCalibrated) {
    // we already computed everything
  }
  // OR ELSE HARTLEY'S METHOD
  else
  // use intrinsic parameters of each camera, but
  // compute the rectification transformation directly
  // from the fundamental matrix
  {
    vector<Point2f> allimgpt[2];
    for (int k = 0; k < 2; k++) {
      for (int i = 0; i < N_BOARDS; i++)
        std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(),
                  back_inserter(allimgpt[k]));
    }
    F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
    Mat H1, H2;
    stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize,
                              H1, H2, 3);

    e_param.R1 = i_param.cameraMatrix[0].inv() * H1 * i_param.cameraMatrix[0];
    e_param.R2 = i_param.cameraMatrix[1].inv() * H2 * i_param.cameraMatrix[1];
    e_param.P1 = i_param.cameraMatrix[0];
    e_param.P2 = i_param.cameraMatrix[1];
  }

  // Precompute maps for cv::remap()
  initUndistortRectifyMap(i_param.cameraMatrix[0], i_param.distCoeffs[0],
                          e_param.R1, e_param.P1, imageSize, CV_16SC2,
                          rmap[0][0], rmap[0][1]);
  initUndistortRectifyMap(i_param.cameraMatrix[1], i_param.distCoeffs[1],
                          e_param.R2, e_param.P2, imageSize, CV_16SC2,
                          rmap[1][0], rmap[1][1]);

  Mat canvas;
  double sf;
  int w, h;
  if (!isVerticalStereo) {
    sf = 600. / MAX(imageSize.width, imageSize.height);
    w = cvRound(imageSize.width * sf);
    h = cvRound(imageSize.height * sf);
    canvas.create(h, w * 2, CV_8UC3);
  } else {
    sf = 300. / MAX(imageSize.width, imageSize.height);
    w = cvRound(imageSize.width * sf);
    h = cvRound(imageSize.height * sf);
    canvas.create(h * 2, w, CV_8UC3);
  }
  bool preview_phase = true;
  int preview_index = 0;

  // Additional Preview
  const int num_of_image_set = 12;
  for (int i = N_BOARDS; i < num_of_image_set; i++) {
    src_image.push_back(imread(GetRightImagePath(i)));
    src_image.push_back(imread(GetLeftImagePath(i)));
  }

  // GLUT
  Stereo ste;

  while (preview_phase) {
    for (int k = 0; k < 2; k++) {
      Mat img = src_image[preview_index * 2 + k], rimg;
      remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
      Mat canvasPart = !isVerticalStereo ? canvas(Rect(w * k, 0, w, h))
                                         : canvas(Rect(0, h * k, w, h));
      resize(rimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
      if (useCalibrated) {
        Rect vroi(cvRound(e_param.validRoi[k].x * sf),
                  cvRound(e_param.validRoi[k].y * sf),
                  cvRound(e_param.validRoi[k].width * sf),
                  cvRound(e_param.validRoi[k].height * sf));
        rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);
      }
    }
    if (!isVerticalStereo)
      for (int j = 0; j < canvas.rows; j += 16)
        line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1,
             8);
    else
      for (int j = 0; j < canvas.cols; j += 16)
        line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1,
             8);
    imshow("rectified", canvas);

    Mat disparity;
    Mat rleft, rright;
    remap(src_image[preview_index * 2], rright, rmap[0][0], rmap[0][1],
          INTER_LINEAR);
    remap(src_image[preview_index * 2 + 1], rleft, rmap[1][0], rmap[1][1],
          INTER_LINEAR);
    int numberOfDisparities = Stereo::CalcDisparity(rleft, rright, disparity);
    Stereo::RenderDisparity(disparity, numberOfDisparities);
    Mat img3d;
    reprojectImageTo3D(disparity, img3d, e_param.Q, true);
    ste.SetData(img3d);
    ste.RenderGLWindow();

    int key = waitKey(0) & 0xff;
    switch (key) {
      case 27:  // Esc
        preview_phase = false;
        break;
      case 'z':
        if (preview_index > 0) {
          --preview_index;
        }
        break;
      case 'x':
        if (preview_index < num_of_image_set - 1) {
          ++preview_index;
        }
        break;
      default:
        break;
    }
  }
}

void mouseWheel(int wheelNumber, int direction, int x, int y) {
  float dZ = (float)direction * 0.2f;
  currentPose.at<float>(0, 3) += dZ * currentPose.at<float>(0, 2);
  currentPose.at<float>(1, 3) += dZ * currentPose.at<float>(1, 2);
  currentPose.at<float>(2, 3) += dZ * currentPose.at<float>(2, 2);
  setCurrentCameraPose();
  render();
}

void motion(int mx, int my) {
  if (drag_mouse_l == 0 && drag_mouse_r == 1) {
    float dX = (last_mouse_x - mx) * 0.2f;
    float dY = (last_mouse_y - my) * 0.2f;
    currentPose.at<float>(0, 3) += dX * currentPose.at<float>(0, 0);
    currentPose.at<float>(1, 3) += dX * currentPose.at<float>(1, 0);
    currentPose.at<float>(2, 3) += dX * currentPose.at<float>(2, 0);
    currentPose.at<float>(0, 3) += dY * currentPose.at<float>(0, 1);
    currentPose.at<float>(1, 3) += dY * currentPose.at<float>(1, 1);
    currentPose.at<float>(2, 3) += dY * currentPose.at<float>(2, 1);
    setCurrentCameraPose();
  } else if (drag_mouse_l == 1 && drag_mouse_r == 0) {
    float dX = (float)(mx - last_mouse_x) * 0.01f;
    float dY = (float)(last_mouse_y - my) * 0.01f;
    cv::Mat rVecX, rVecY, rMatX, rMatY;
    rVecY = dX * currentPose(cv::Rect(
                     1, 0, 1, 3));  // mouse horizontal move -> y-axis rot
    cv::Rodrigues(rVecY, rMatY);
    rVecX = dY * currentPose(cv::Rect(0, 0, 1,
                                      3));  // mouse vertical move -> x-axis rot
    cv::Rodrigues(rVecX, rMatX);
    currentPose = rMatX * rMatY * currentPose;
    setCurrentCameraPose();
  }

  last_mouse_x = mx;
  last_mouse_y = my;

  render();
}

void mouse(int button, int state, int mx, int my) {
  if ((button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN))
    drag_mouse_l = 1;
  else if ((button == GLUT_LEFT_BUTTON) && (state == GLUT_UP))
    drag_mouse_l = 0;

  if ((button == GLUT_RIGHT_BUTTON) && (state == GLUT_DOWN))
    drag_mouse_r = 1;
  else if ((button == GLUT_RIGHT_BUTTON) && (state == GLUT_UP))
    drag_mouse_r = 0;

  last_mouse_x = mx;
  last_mouse_y = my;
}

static void resize(int w, int h) {
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(100.0, (double)w / (double)h, 1.0, 10000.0);
  glMatrixMode(GL_MODELVIEW);
}

Stereo::Stereo() {
  ste = this;
  int argc = 0;
  glutInit(&argc, NULL);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
  glutInitWindowSize(640, 480);
  gl_window_ = glutCreateWindow("Reconstruction");
  glutReshapeFunc(resize);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
  glutMouseWheelFunc(mouseWheel);
  resize(640, 480);
  setCurrentCameraPose();
}
