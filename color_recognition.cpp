#include "knuros.h"

// just use MGSign get_sign(Mat& img);

extern MGSign gSign;

double angle(Point pt1, Point pt2, Point pt0)
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

void find_polygon(Mat image, vector<vector<Point> >& squares, vector<vector<Point> >& triangles)
{
    GaussianBlur(image, image, Size(5, 5), 1, 1);
    Mat gray0;
    cvtColor(image, gray0, COLOR_BGR2GRAY);
    Mat gray(image.size(), CV_8U);
    vector<vector<Point> > contours;
    
    gray = gray0 >= 100;
    
    findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
    
    vector<Point> approx;
    for(size_t i = 0; i < contours.size(); i++)
    {
        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.02, true);
        
        if(approx.size() == 3 && isContourConvex(Mat(approx)) && fabs(contourArea(Mat(approx))) > 3000)
        {
            if(approx[0].x == 0 || approx[0].y == 0)
                continue;
            if(approx[1].x == 0 || approx[1].y == 0)
                continue;
            if(approx[2].x == 0 || approx[2].y == 0)
                continue;
            
            triangles.push_back(approx);
            
        }
        
        if(approx.size() == 4 && isContourConvex(Mat(approx)) && fabs(contourArea(Mat(approx))) > 3000)
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

void find_largest_triangle(const vector<vector<Point> >& triangles, vector<Point>& biggest_triangle)
{
    if(triangles.size() <= 0) return;
    
    int max_width = 0;
    int max_height = 0;
    int max_triangle_idx = 0;
    
    for(int i = 0; i < (int)triangles.size(); i++)
    {
        Rect bound = boundingRect(Mat(triangles[i]));
        if(bound.width >= max_width && bound.height >= max_height)
        {
            max_width = bound.width;
            max_height = bound.height;
            max_triangle_idx = i;
        }
    }
    biggest_triangle = triangles[max_triangle_idx];
}

void find_largest_square(const vector<vector<Point> >& squares, vector<Point>& biggest_square)
{
    if(squares.size() <= 0) return;
    
    int max_width = 0;
    int max_height = 0;
    int max_square_idx = 0;
    
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

// plz revise it
MGColor get_color(Mat& img)
{
    long long acc_b = 0, acc_g = 0, acc_r = 0;
    long long total_pixels = img.rows * img.cols;
    
    for(int i = 0; i < img.rows; i++)
    {
        for(int j = 0; j < img.cols; j++)
        {
            int b = img.at<Vec3b>(i, j)[0];
            int g = img.at<Vec3b>(i, j)[1];
            int r = img.at<Vec3b>(i, j)[2];

	    cout << "(R, G, B) : " << r << " " << g << " " << b << endl;
            
            // is it red?
            if(r >= g && r >= b)
                acc_r += 1;
            // is it green?
            else if(g >= r && g >= b)
                acc_g += 1;
            //is it blue?
            else if(b >= r && b >= g)
                acc_b += 1;
            
        }
    }

    cout << "total rgb" << endl;
    cout << acc_r << " " << acc_g << " " << acc_b << endl;


    // revise it! more accurate
    if(acc_r >= acc_b && acc_r >= acc_g)
    {
        cout << "RED color!" << endl;
        return RED;
    }
    if(acc_g >= acc_r && acc_g >= acc_b)
    {
        cout << "GREEN color!" << endl;
        return GREEN;
    }
    if(acc_b >= acc_r && acc_b >= acc_g)
    {
        cout << "BLUE color!" << endl;
        return BLUE;
    }
    
    cout << "NOCOLOR!" << endl;
    return NOCOLOR;
}


MGSign get_sign(Mat& img)
{
    vector<vector<Point> > squares, triangles;
    vector<Point> largest_square, largest_triangle;
    
    find_polygon(img, squares, triangles);
    find_largest_square(squares, largest_square);
    find_largest_triangle(triangles, largest_triangle);
    
    Rect cropped_rect;
    Mat cropped_mat;
    MGColor color;
    
    draw_recognition_display(squares, triangles, img);

    
    if(largest_square.size() <= 0 && largest_triangle.size() <= 0)
    {
        cout << "No square and triangle!" << endl;
        return NIL;
    }
    
    if(largest_square.size() <= 0) // only got triangle
    {
        cropped_rect = boundingRect(Mat(largest_triangle));
        cropped_mat = img(cropped_rect);
        color = get_color(cropped_mat);
        if(color == BLUE)
        {
            return STOP;
        }
        else if(color == GREEN)
        {
            return STOP;
        }
        else
        {
            return NIL;
        }
    }
    
    // got square
    cropped_rect = crop_img(img, largest_square);
    cropped_mat = img(cropped_rect);
    color = get_color(cropped_mat);
    
    if(color == BLUE)
    {
        return NIL;
    }
    else if(color == GREEN)
    {
        return GO;
    }
    else if(color == RED)
    {
        return PARKING_SIGN;
    }
    
    return NIL;
}

void draw_recognition_display(vector<vector<Point> >& squares, vector<vector<Point> >& triangles, Mat& img)
{
    //draw lines for triangle
    for(auto triangle : triangles)
    {
        for(int i = 0; i < triangle.size(); i++)
        {
            line(img, triangle[i % 3], triangle[(i+1) % 3], Scalar(255, 255, 0), 4, 8);
        }
    }
    
    // draw lines for rectangle
    for(auto square : squares)
    {
        Rect cropped_rect = crop_img(img, square);
        rectangle(img, cropped_rect, Scalar(255, 255, 0), 4, 8);
    }
 //   rectangle(img, cropped_rect, Scalar(0, 255, 255), 4, 8);
    
    imshow("happy", img);
    waitKey(30);
}


void postMessageRecievedRGB(const sensor_msgs::ImageConstPtr& msg)
{
    Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    //  Mat img_origin = img.clone();
    MGSign temp_sign = get_sign(img);
    
    switch (temp_sign)
    {
        case STOP:
            cout << "STOP" << endl;
            break;
        case GO:
            cout << "GO" << endl;
            break;
        case PARKING_AREA:
            cout << "PARKING_AREA" << endl;
            break;
        case PARKING_SIGN:
            cout << "PARKING_SIGN" << endl;
            break;
        default:
            cout << "NIL" << endl;
            break;
    }
    if(temp_sign != NIL)
    {
    	gSign = temp_sign;
    }
}
