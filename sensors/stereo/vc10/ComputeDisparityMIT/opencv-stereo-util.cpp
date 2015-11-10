/*
 * Utility functions for opencv-stereo
 *
 * Copyright 2013, Andrew Barry <abarry@csail.mit.edu>
 *
 */

#include "opencv-stereo-util.hpp"


/**
 * Loads XML stereo calibration files and reamps them.
 *
 * @param calibrationDir directory the calibration files are in
 * @param stereoCalibration calibration structure to fill in
 *
 * @retval true on success, false on falure.
 *
 */
bool LoadCalibration(string calibrationDir, OpenCvStereoCalibration *stereoCalibration)
{
    Mat qMat, mx1Mat, my1Mat, mx2Mat, my2Mat, m1Mat, d1Mat, r1Mat, p1Mat, r2Mat, p2Mat, m2Mat, d2Mat;

    CvMat *Q = (CvMat *)cvLoad((calibrationDir + "/Q.xml").c_str(),NULL,NULL,NULL);

    if (Q == NULL)
    {
        std::cerr << "Error: failed to read " + calibrationDir + "/Q.xml." << std::endl;
        return false;
    }

    CvMat *mx1 = (CvMat *)cvLoad((calibrationDir + "/mx1.xml").c_str(),NULL,NULL,NULL);

    if (mx1 == NULL)
    {
        std::cerr << "Error: failed to read " << calibrationDir << "/mx1.xml." << std::endl;
        return false;
    }

    CvMat *my1 = (CvMat *)cvLoad((calibrationDir + "/my1.xml").c_str(),NULL,NULL,NULL);

    if (my1 == NULL)
    {
        std::cerr << "Error: failed to read " << calibrationDir << "/my1.xml." << std::endl;
        return false;
    }

    CvMat *mx2 = (CvMat *)cvLoad((calibrationDir + "/mx2.xml").c_str(),NULL,NULL,NULL);

    if (mx2 == NULL)
    {
        std::cerr << "Error: failed to read " << calibrationDir << "/mx2.xml." << std::endl;
        return false;
    }

    CvMat *my2 = (CvMat *)cvLoad((calibrationDir + "/my2.xml").c_str(),NULL,NULL,NULL);

    if (my2 == NULL)
    {
        std::cerr << "Error: failed to read " << calibrationDir << "/my2.xml." << std::endl;
        return false;
    }

    CvMat *m1 = (CvMat *)cvLoad((calibrationDir + "/M1.xml").c_str(),NULL,NULL,NULL);

    if (m1 == NULL)
    {
        std::cerr << "Error: failed to read " << calibrationDir << "/M1.xml." << std::endl;
        return false;
    }

    CvMat *d1 = (CvMat *)cvLoad((calibrationDir + "/D1.xml").c_str(),NULL,NULL,NULL);

    if (d1 == NULL)
    {
        std::cerr << "Error: failed to read " << calibrationDir << "/D1.xml." << std::endl;
        return false;
    }

    CvMat *r1 = (CvMat *)cvLoad((calibrationDir + "/R1.xml").c_str(),NULL,NULL,NULL);

    if (r1 == NULL)
    {
        std::cerr << "Error: failed to read " << calibrationDir << "/R1.xml." << std::endl;
        return false;
    }

    CvMat *p1 = (CvMat *)cvLoad((calibrationDir + "/P1.xml").c_str(),NULL,NULL,NULL);

    if (p1 == NULL)
    {
        std::cerr << "Error: failed to read " << calibrationDir << "/P1.xml." << std::endl;
        return false;
    }

    CvMat *m2 = (CvMat *)cvLoad((calibrationDir + "/M2.xml").c_str(),NULL,NULL,NULL);

    if (m2 == NULL)
    {
        std::cerr << "Error: failed to read " << calibrationDir << "/M2.xml." << std::endl;
        return false;
    }

    CvMat *d2 = (CvMat *)cvLoad((calibrationDir + "/D2.xml").c_str(),NULL,NULL,NULL);

    if (d2 == NULL)
    {
        std::cerr << "Error: failed to read " << calibrationDir << "/D2.xml." << std::endl;
        return false;
    }

    CvMat *r2 = (CvMat *)cvLoad((calibrationDir + "/R2.xml").c_str(),NULL,NULL,NULL);

    if (r2 == NULL)
    {
        std::cerr << "Error: failed to read " << calibrationDir << "/R2.xml." << std::endl;
        return false;
    }

    CvMat *p2 = (CvMat *)cvLoad((calibrationDir + "/P2.xml").c_str(),NULL,NULL,NULL);

    if (p2 == NULL)
    {
        std::cerr << "Error: failed to read " << calibrationDir << "/P2.xml." << std::endl;
        return false;
    }



    qMat = Mat(Q, true);
    mx1Mat = Mat(mx1,true);
    my1Mat = Mat(my1,true);
    mx2Mat = Mat(mx2,true);
    my2Mat = Mat(my2,true);

    m1Mat = Mat(m1,true);
    d1Mat = Mat(d1,true);
    r1Mat = Mat(r1,true);
    p1Mat = Mat(p1,true);

    m2Mat = Mat(m2,true);
    d2Mat = Mat(d2,true);
    r2Mat = Mat(r2,true);
    p2Mat = Mat(p2,true);

    Mat mx1fp, empty1, mx2fp, empty2;

    // this will convert to a fixed-point notation
    convertMaps(mx1Mat, my1Mat, mx1fp, empty1, CV_16SC2, true);
    convertMaps(mx2Mat, my2Mat, mx2fp, empty2, CV_16SC2, true);

    stereoCalibration->qMat = qMat;
    stereoCalibration->mx1fp = mx1fp;
    stereoCalibration->mx2fp = mx2fp;

    stereoCalibration->M1 = m1Mat;
    stereoCalibration->D1 = d1Mat;
    stereoCalibration->R1 = r1Mat;
    stereoCalibration->P1 = p1Mat;

    stereoCalibration->M2 = m2Mat;
    stereoCalibration->D2 = d2Mat;
    stereoCalibration->R2 = r2Mat;
    stereoCalibration->P2 = p2Mat;


    return true;
}




/**
 * Draws 3D points onto a 2D image when given the camera calibration.
 *
 * @param camera_image image to draw onto
 * @param points_list_in vector<Point3f> of 3D points to draw. Likely obtained from Get3DPointsFromStereoMsg
 * @param cam_mat_m camera calibration matrix (usually M1.xml)
 * @param cam_mat_d distortion calibration matrix (usually D1.xml)
 * @param cam_mat_r rotation calibration matrix (usually R1.xml)
 * @param outline_color color to draw the box outlines (default: 128)
 * @param inside_color color to draw the inside of the boxes (default: 255). Set to -1 for no fill.
 * @param box_top if you only want to draw points inside a box, this specifies one coordinate of the box
 * @param box_bottom the second coordinate of the box
 * @param points_in_box if you pass box_top and box_bottom, this will be filled with the indicies of
 *          the points inside the box.
 * @param min_z minimum z value allowable to draw the point
 * @param max_z maximum z value allowable to draw the point
 * @param box_size size of the box (default = 4)
 */
void Draw3DPointsOnImage(Mat camera_image, vector<Point3f> *points_list_in, Mat cam_mat_m, Mat cam_mat_d, Mat cam_mat_r, Scalar outline_color, Scalar inside_color, Point2d box_top, Point2d box_bottom, vector<int> *points_in_box,
float min_z, float max_z, int box_size) {
    vector<Point3f> &points_list = *points_list_in;

    if (points_list.size() <= 0)
    {
        //std::cout << "Draw3DPointsOnimage: zero sized points list" << std::endl;
        return;
    }


    vector<Point2f> img_points_list;

    projectPoints(points_list, cam_mat_r.inv(), Mat::zeros(3, 1, CV_32F), cam_mat_m, cam_mat_d, img_points_list);


    int min_x = min(box_top.x, box_bottom.x);
    int min_y = min(box_top.y, box_bottom.y);
    int max_x = max(box_top.x, box_bottom.x);
    int max_y = max(box_top.y, box_bottom.y);
    bool box_bounding = false;

    if (box_top.x != -1 || box_top.y != -1 || box_bottom.x != -1 || box_bottom.y != -1) {
        box_bounding = true;
    }

    int thickness = CV_FILLED;
    if (inside_color[0] == -1) {
        thickness = 1;
    }

    // now draw the points onto the image
    for (int i=0; i<int(img_points_list.size()); i++)
    {

        //line(camera_image, Point(img_points_list[i].x, 0), Point(img_points_list[i].x, camera_image.rows), color);
        //line(camera_image, Point(0, img_points_list[i].y), Point(camera_image.cols, img_points_list[i].y), color);

        bool flag = false;

        if (box_bounding) {

            if (img_points_list[i].x >= min_x && img_points_list[i].x <= max_x &&
                img_points_list[i].y >= min_y && img_points_list[i].y <= max_y) {

                if (points_in_box) {
                    points_in_box->push_back(i);
                }

                flag = true;
            }
        }


        if (box_bounding == false || flag == true) {

            if (min_z == 0 || points_list[i].z >= min_z) {

                if (max_z == 0 || points_list[i].z <= max_z) {

                    rectangle(camera_image, Point(img_points_list[i].x - box_size, img_points_list[i].y - box_size),
                        Point(img_points_list[i].x + box_size, img_points_list[i].y + box_size), outline_color, thickness);

                    if (inside_color[0] != -1) {
                        rectangle(camera_image, Point(img_points_list[i].x - 2, img_points_list[i].y - box_size/2),
                            Point(img_points_list[i].x + box_size/2, img_points_list[i].y + box_size/2), inside_color, thickness);
                    }

                }
            }
        }

    }
}

int GetDisparityForDistance(double distance, const OpenCvStereoCalibration &calibration, int *inf_disparity) {

    int min_search = -100;
    int max_search = 100;

    cv::vector<Point3f> disparity_candidates;
    cv::vector<int> disparity_values;
    for (int i = min_search; i <= max_search; i++) {
        disparity_candidates.push_back(Point3f(0, 0, -i)); // note the negative! it is correct!
        disparity_values.push_back(i);
    }
    cv::vector<Point3f> vector_3d_out;

    perspectiveTransform(disparity_candidates, vector_3d_out, calibration.qMat);

    int best_disparity = 0;
    double best_dist_abs = -1;
    double max_dist = -1000;
    int max_disparity = 0;

    for (int i = 0; i < (int)vector_3d_out.size(); i++) {
        double this_dist_abs = fabs(vector_3d_out.at(i).z - distance);
        //std::cout << "Disp: " << disparity_values.at(i) << " ----> " << vector_3d_out.at(i).z << " (dist = " << this_dist_abs << ")" << std::endl;
        if (best_dist_abs == -1 || this_dist_abs < best_dist_abs) {
            best_disparity = disparity_values.at(i);
            best_dist_abs = this_dist_abs;
        }

        if (vector_3d_out.at(i).z > max_dist) {
            max_dist = vector_3d_out.at(i).z;
            max_disparity = disparity_values.at(i);
        }
    }

    if (inf_disparity != nullptr) {
        *inf_disparity = max_disparity;
    }

    return best_disparity;
}

