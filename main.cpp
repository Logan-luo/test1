#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <iostream>
#define g 9.8
#define PAI 3.14
using namespace std;
using namespace cv;
//求出两条直线交点
Point2f setzero(Point2f a)
{
    a.x = 0;
    a.y = 0;
    return a;
}
float calangle(float distance, float high, float speed)
{
    float a = high / distance;
    float x = distance, y = high;
    float angle1 = atanf(a), angle2 = atanf(a);
    float dy;
    do
    {
        float a = cos(angle1), b = sin(angle1);
        float time = x / 100 / speed * a;
        dy = y - speed / 100 * b + 1 / 2 * g * pow(time, 2);
        y = y + dy;
        angle1 = atan2f(y, x);
    } while (dy < 10);
    return (angle1 - angle2) / (2 * PAI) * 360;
}
Point2f findcen(Point2f a, Point2f b, Point2f c, Point2f d)
{
    int x, y;
    int X1 = a.x - b.x, Y1 = a.y - b.y, X2 = c.x - d.x, Y2 = c.y - d.y;
    if (X1 * Y2 == X2 * Y1)
        return Point((b.x + c.x) / 2, (b.y + c.y) / 2);
    else
    {
        int A = X1 * a.y - Y1 * a.x, B = X2 * c.y - Y2 * c.x;
        y = (A * Y2 - B * Y1) / (X1 * Y2 - X2 * Y1);
        x = (B * X1 - A * X2) / (Y1 * X2 - Y2 * X1);
        return Point2f(x, y);
    }
}

int main(int argc, char *argv[])
{
    if (argc != 2)
        return -1;
    string path = argv[1];
    VideoCapture cap(path);
    float fps = cap.get(CAP_PROP_FPS); // calculate the fps to get the time between each capture
    float ftime = 1 / fps;
    Point2f cent1, cent2;
    setzero(cent1);
    setzero(cent2);
    if (!cap.isOpened())
    {
        cout << "failed to open the video!" << endl;
        return -1;
    }
    Mat img;
    while (cap.read(img))
    {
        vector<Mat> channels(3);
        split(img, channels.data());
        Mat minus_channel = channels[2] - channels[0];
        vector<vector<Point>> contours;
        Mat binary;
        threshold(minus_channel, binary, 80, 255, THRESH_BINARY);
        // namedWindow("test1");
        //  imshow("Image", img);
        // imshow("test1", binary);
        findContours(binary, contours, RETR_EXTERNAL, RETR_LIST);

        vector<RotatedRect> tubes;
        vector<Point2f> fourPoints(4);
        vector<Point2f> testPoints(2);
        for (size_t i = 0; i < contours.size(); i++)
        {
            if (contourArea(contours[i]) < 100)
            {
                continue;
            }
            // drawContours(img,contours,i,Scalar(0,255,0),3);
            RotatedRect temp = minAreaRect(contours[i]);
            tubes.push_back(temp);
        }
        if (tubes.empty())
        {
            continue;
        }
        for (size_t i = 0; i < 2; i++)
        {
            tubes[i].points(fourPoints.data());
            testPoints[i] = fourPoints[i];
        }

        if (testPoints[1].x < testPoints[0].x)
        {
            tubes.push_back(tubes[0]);
            tubes[0] = tubes[1];
            tubes[1] = tubes[2];
        }
        // make sure the tubes[0] is the left and make sure the tubes[1] is the right
        for (size_t i = 0; i < tubes.size(); i++)
        {
            tubes[i].points(fourPoints.data());
            for (int j = 0; j < 4; j++)
            {
                line(img, fourPoints[j], fourPoints[(j + 1) % 4], Scalar(0, 0, 255));
            }
        }
        vector<Point2f> fimid(2);
        vector<Point2f> semid(2);
        tubes[0].points(fourPoints.data());
        float b = 0.;
        for (int j = 0; j < 4; j++)
        {
            b = b + fourPoints[j].y;
        }
        for (int i = 0; i < 4; i++)
        {
            if (fourPoints[i].y > (b / 4))
            {
                fimid[0].x = fimid[0].x + (fourPoints[i].x / 2);
                fimid[0].y = fimid[0].y + (fourPoints[i].y / 2);
            }
            if (fourPoints[i].y < (b / 4))
            {
                fimid[1].x = fimid[1].x + (fourPoints[i].x / 2);
                fimid[1].y = fimid[1].y + (fourPoints[i].y / 2);
            }
        }

        tubes[1].points(fourPoints.data());
        b = 0.;
        for (int j = 0; j < 4; j++)
        {
            b = b + fourPoints[j].y;
        }
        for (int i = 0; i < 4; i++)
        {
            if (fourPoints[i].y > (b / 4))
            {
                semid[0].x = semid[0].x + (fourPoints[i].x) / 2;
                semid[0].y = semid[0].y + (fourPoints[i].y) / 2;
            }
            if (fourPoints[i].y < (b / 4))
            {
                semid[1].x = semid[1].x + (fourPoints[i].x) / 2;
                semid[1].y = semid[1].y + (fourPoints[i].y) / 2;
            }
        }
        line(img, fimid[0], semid[1], Scalar(0, 0, 255));
        line(img, fimid[1], semid[0], Scalar(0, 0, 255));

        vector<Point3f> objP;
        Mat objM;
        objP.clear();
        objP.push_back(Point3f(15, -65, 0));
        objP.push_back(Point3f(-15, -65, 0));
        objP.push_back(Point3f(15, 65, 0));
        objP.push_back(Point3f(-15, 65, 0));

        //初始化相机参数Opencv
        double camD[9] = {
            1.6041191539594568e+03, 0., 6.3983687194220943e+02, 0.,
            1.6047833493341714e+03, 5.1222951297937527e+02, 0., 0., 1};
        Mat camera_matrix = Mat(3, 3, CV_64FC1, camD);

        //畸变参数
        double distCoeffD[5] = {-6.4910709385278609e-01, 8.6914328787426987e-01,
                                5.1733428362687644e-03, -4.1111054148847371e-03, 0.};
        Mat distortion_coefficients = Mat(5, 1, CV_64FC1, distCoeffD);
        // out put
        vector<Point2f> Points2D(4);
        Points2D[0] = fimid[0];
        Points2D[1] = fimid[1];
        Points2D[2] = semid[0];
        Points2D[3] = semid[1];
        Mat rvec, tvec;
        solvePnP(objP, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false);

        /*float dis = sqrt((pow(tvec.at<double>(0, 0), 2) + pow(tvec.at<double>(0, 1), 2) + pow(tvec.at<double>(0, 2), 2)));

        cout << "The distance is " << dis << endl;*/

        // using the distance to caculate the angle//
        float xydis = sqrt((pow(tvec.at<double>(0, 0), 2) + pow(tvec.at<double>(0, 1), 2)));
        float zdis = sqrt(pow(tvec.at<double>(0, 2), 2));
        float angle = calangle(xydis, zdis, 30);
        cout << "The angle is " << angle << endl;
        // caculate the speed//
        cent2 = findcen(fimid[0], semid[1], fimid[1], semid[0]);
        float vx = (cent2.x - cent1.x) / 100 / ftime;
        float vy = (cent2.y - cent1.y) / 100 / ftime;
        cent1 = cent2;
        Point2f cent3;
        cent3.x = cent2.x + fps * vx;
        cent3.y = cent2.y + fps * vy;
        line(img, cent2, cent3, Scalar(0, 0, 255), 2);
        putText(img, "The nest second of x and y will be" + to_string(cent3.x) + " " + to_string(cent3.y), Point(1, 20), FONT_HERSHEY_DUPLEX, 0.75, Scalar(255, 255, 255), 1);
        namedWindow("test2");
        imshow("test2", img);
        if (waitKey(30) == 27) // Esc
            if (waitKey(0) == 27)
                break;
    }
}
