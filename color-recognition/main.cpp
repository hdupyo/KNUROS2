//
//  main.cpp
//  opencv_project
//
//  Created by 김만기 on 28/04/2019.
//  Copyright © 2019 김만기. All rights reserved.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

enum MGColor {RED, GREEN, BLUE};

double angle(Point pt1, Point pt2, Point pt0)
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

void find_squares(Mat image, vector<vector<Point>>& squares)
{
    GaussianBlur(image, image, Size(5, 5), 1, 1);
    Mat gray0;
    cvtColor(image, gray0, COLOR_BGR2GRAY);
    Mat gray(image.size(), CV_8U);
    vector<vector<Point>> contours;
    
    gray = gray0 >= 100;
    
    findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
    
    vector<Point> approx;
    for(size_t i = 0; i < contours.size(); i++)
    {
        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.02, true);
        
        
        if(approx.size() == 4 && isContourConvex(Mat(approx)) && fabs(contourArea(Mat(approx))) > 1000)
        {
            if(approx[0].x == 0 || approx[0].y == 0)
                continue;
            if(approx[1].x == 0 || approx[1].y == 0)
                continue;
            if(approx[2].x == 0 || approx[2].y == 0)
                continue;
            if(approx[3].x == 0 || approx[3].y == 0)
                continue;
                    
            double maxCosine = 0;
            
            for(int j = 2; j < 5; j++)
            {
                double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                maxCosine = max(maxCosine, cosine);
            }
            
            if(maxCosine < 0.3)
                squares.push_back(approx);
        }
    }
}

void find_largest_square(const vector<vector<Point>>& squares, vector<Point>& biggest_square)
{
    if(squares.size() <= 0) return;
    
    int max_width = 0;
    int max_height = 0;
    int max_square_idx = 0;
//    const int n_points = 4; why used?
    
    for(int i = 0; i < (int)squares.size(); i++)
    {
        Rect rectangle = boundingRect(Mat(squares[i]));
        if((rectangle.width >= max_width) && (rectangle.height >= max_height))
        {
            max_width = rectangle.width;
            max_height = rectangle.height;
            max_square_idx = i;
        }
    }
    biggest_square = squares[max_square_idx];
}

// cropping matrix function
Rect crop_img(Mat& img, vector<Point>& largest_square)
{
    Point topLeft;
    Point bottomRight;
    
    sort(largest_square.begin(), largest_square.end(), [](Point a, Point b) {return a.x < b.x ;} );
    
    topLeft = largest_square[0];
    if(largest_square[0].y > largest_square[1].y)
        topLeft = largest_square[1];
    
    bottomRight = largest_square[2];
    if(largest_square[2].y < largest_square[3].y)
        bottomRight = largest_square[3];
    Rect ret(topLeft, bottomRight);
    return ret;
}

// calculating matrix color
// color is stored as B, G, R order
MGColor get_color(Mat& img)
{
    long long acc_b = 0, acc_g = 0, acc_r = 0;
    for(int i = 0; i < img.rows; i++)
    {
        for(int j = 0; j < img.cols; j++)
        {
            int b = img.at<Vec3b>(i, j)[0];
            int g = img.at<Vec3b>(i, j)[1];
            int r = img.at<Vec3b>(i, j)[2];
            
            acc_b += b;
            acc_g += g;
            acc_r += r;
        }
    }
    if(acc_b > acc_g && acc_b > acc_r)
    {
        return BLUE;
    }
    else if(acc_g > acc_b && acc_g > acc_r)
    {
        return GREEN;
    }
    else
    {
        return RED;
    }
    return RED;
}


int main(int argc, const char * argv[])
{
    Mat img = imread("red_squares.png", IMREAD_COLOR);
    vector<vector<Point>> squares;
    vector<Point> largest_square;
    
    find_squares(img, squares);
    find_largest_square(squares, largest_square);
    
    
    if(largest_square.size() <= 0)
    {
        cout << "no largest square!" << endl;
        exit(0);
    }
    
    Rect cropped_rect = crop_img(img, largest_square);
    Mat cropped_mat = img(cropped_rect);
    MGColor color = get_color(cropped_mat);
    
    if(color == BLUE)
    {
        cout << "BLUE" << endl;
    }
    else if(color == GREEN)
    {
        cout << "GREEN" << endl;
    }
    else
    {
        cout << "RED" << endl;
    }
    
    
    for(auto square : squares)
    {
        Rect cropped_rect = crop_img(img, square);
        rectangle(img, cropped_rect, Scalar(255, 255, 0), 4, 8);
    }
    
    rectangle(img, cropped_rect, Scalar(0, 255, 255), 4, 8);
    
    imshow("happy", img);
    waitKey(0);
    
    
    return 0;
}
