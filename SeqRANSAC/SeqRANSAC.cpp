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
    
    // Number of points:

    num=points.size();

    for (int i = 0; i < 3; i++)
    {

    }
    /*
    
    // Estimate plane parameters without robustification
    
    float* plane=EstimatePlaneImplicit(points);
    printf("Plane fitting results for the whole data:\nA:%f B:%f C:%f D:%f\n",plane[0],plane[1],plane[2],plane[3]);
    
    delete[] plane;    
    
    // RANSAC-based robust estimation
    
    float* planeParams=EstimatePlaneRANSAC(points,THERSHOLD,RANSAC_ITER);
    
    printf("Plane params RANSAC:\n A:%f B:%f C:%f D:%f \n",planeParams[0],planeParams[1],planeParams[2],planeParams[3]);
    
    // Compute differences of the fitted plane in order to separate inliers from outliers
    
    RANSACDiffs differences=PlanePointRANSACDifferences(points,planeParams,THERSHOLD);

    delete[] planeParams;    
    
    // Inliers and outliers are coloured by green and red, respectively

    */
    RANSACDiffs differences = findDifferences(points, THRESHOLD, RANSAC_ITER);
    
    vector<Point3i> colorsRANSAC;
    
    for (int idx=0; idx<num; idx++)
    {
        Point3i newColor;

        if (differences.isInliers.at(idx))
        {
           newColor.x=0;
           newColor.y=255;
           newColor.z=0;
        }
        else
        {
           newColor.x=255;
           newColor.y=0;
           newColor.z=0;
        }
        
        colorsRANSAC.push_back(newColor);            
    }
    
    // Write results into a PLY file. 
    // It can be visualized by open-source 3D application Meshlab (www.meshlab.org)

    WritePLY(argv[2],points,colorsRANSAC);
        
}