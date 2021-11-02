#include "MatrixReaderWriter.h"
#include "PlaneEstimation.h"
#include "PLYWriter.h"
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;

#define THRESHOLD 0.2 // RANSAC threshold (if Velodyne scans are processed, the unit is meter)
#define RANSAC_ITER  200 // RANSAC iteration
#define FILTER_LOWEST_DISTANCE 0.3 // threshold for pre-filtering

int main(int argc, char** argv)
{    
    if (argc!=3)
    {
        printf("Usage:\n SeqRansac input.xyz output.ply\n");
        exit(EXIT_FAILURE);
    }
    
    MatrixReaderWriter mrw(argv[1]);
    
    int num=mrw.rowNum;
    
    cout<< "Rows:" << num <<endl;
    cout<< "Cols:" << mrw.columnNum << endl;

    // Read data from text file
    
    vector<Point3f> points;
    
    for (int idx=0;idx<num;idx++)
    {
       double x=mrw.data[3*idx];
       double y=mrw.data[3*idx+1];
       double z=mrw.data[3*idx+2];
       
       float distFromOrigo=sqrt(x*x+y*y+z*z);


        // First filter: minimal work distance for a LiDAR limited.        
       
       if (distFromOrigo>FILTER_LOWEST_DISTANCE)
       {
           Point3f newPt;
           newPt.x=x;
           newPt.y=y;
           newPt.z=z;
           points.push_back(newPt);
       }       
    }   
    
    // number of initial points
    num=points.size();

    // final vector of points and colors with indices synchronized
    vector<Point3f> final_points;
    vector<Point3i> colorsRANSAC;

    RANSACDiffs differences = findDifferences(points, THRESHOLD, RANSAC_ITER);

    // other necessary variables
    vector<Point3f> new_points;
    Point3i color;
    Point3f current_point;

    for (int i = 0; i < 3; i++)
    {
        for (int idx = 0; idx < num; idx++)
        {
            // colors depend on iteration (which plane we're looking for)
            switch (i)
            {
                case 0:
                    color.x = 255;
                    color.y = 0;
                    color.z = 0;
                    break;
                case 1:
                    color.x = 0;
                    color.y = 255;
                    color.z = 0;
                    break;
                case 2:
                    color.x = 255;
                    color.y = 153;
                    color.z = 255;
                    break;
            }            

            current_point = points.at(idx);

            if (differences.isInliers.at(idx))
            {
                final_points.push_back(current_point);
                colorsRANSAC.push_back(color);
            }
            else
            {
                new_points.push_back(current_point);
            }
        }
        
        num = new_points.size(); // number of points after detected plane points are removed (outlier points)
        points.clear();
        points = new_points;
        printf("Number of outlier points for iteration (i.e., deteted plane) number %i: %i\n", i+1, num);

        if (i < 2)
        {
            differences = findDifferences(new_points, THRESHOLD, RANSAC_ITER);
            new_points.clear();
        }
        // the remainder outlier points (colored blue) can be added using this code block
        /*
        else
        {
            for (int idx = 0; idx < num; idx++)
            {
                color.x = 0;
                color.y = 0;
                color.z = 255;

                current_point = points.at(idx);

                final_points.push_back(current_point);
                colorsRANSAC.push_back(color);
            }
        }
        */
    }
    
    // Write results into a PLY file. 
    // It can be visualized by open-source 3D application Meshlab (www.meshlab.org)

    WritePLY(argv[2], final_points, colorsRANSAC);        
}