/**
 * ICP localization
 * Skeleton code for teaching
 * A3M33MKR
 * Czech Technical University
 * Faculty of Electrical Engineering
 * Intelligent and Mobile Robotics group
 *
 * Authors: Miroslav Kulich, Zdeněk Kasl, Karel Košnar kulich@labe.felk.cvut.cz kosnar@labe.felk.cvut.cz
 *
 * Licence: MIT (see LICENSE file)
 **/

#include<cmath>
#include<cassert>
#include<cstdlib>
#include<fstream>
#include<iostream>
#include <cstdlib>
#include <ctime>
#include <vtkPNGReader.h>

#include "gui/gui.h"
#include "dataLoader/laserDataLoader.h"
#include "laserSimulator/lasersimulator.h"
using namespace std;
using namespace imr;
using namespace gui;
using namespace laserDataLoader;

#define COUNT_PARTICLE 3000

double toRadians(const double alpha);

//function performing the iterative closest point method - you must to implement it
RobotPosition icp(const Measurement reference, const Measurement actual, RobotPosition previousPosition);

//function convert RobotPosition to Point structure (for drawing)
Point robotPosition2point(const RobotPosition &rp);

//convert the laser scan into vector of points in Cartesian coordinate system using the odometry from the Measurement structure
void calculateRawPoints(RawPoints &rp, Measurement m);

//convert the laser scan into vector of points in Cartesian coordinate system using given robot position p
void calculateRawPoints(RawPoints &rp, Measurement m, RobotPosition p);

//convert point in polar coordinate system  into Cartesian coordinate system (possibly moved by Robot Position)
void polar2cartesian(Point &p, const double &alpha, const double &r, const RobotPosition &p0);

void help(char** argv)
{
    std::cout << "\nUsage of the program " << argv[0]+2 << ":\n"
         << "Parameter [-h or -H] displays this message.\n"
         << "Parameter [-f or -F] specifies path to data."
         << "Parameter [-m or -M] specifies number of measurements taken,\n"
         << "   default number of measurements is 2.\n"
         << std::endl;
}


int main(int argc, char** argv)
{
    int nMeasurements = 2;
    char *dataFile;

    // argument count must be greater than three
    // >>> at least source file must be specified
    if (argc < 3) {
        help(argv);
        return EXIT_FAILURE;
    }

    // Parse all console parameters
    for (int i=0; i<argc; i++) {
        if (argv[i][0]=='-') {
            switch(argv[i][1]) {
                //> HELP
                case 'H' : case 'h' :
                    help(argv);
                    break;

                //> Source file
                case 'F' : case 'f' :
                    assert(i+1 < argc);
                    dataFile = argv[i+1];
                    break;

                //> Number of Measurements
                case 'M' : case 'm' :
                    assert(i+1 < argc);
                    assert(atoi(argv[i+1])>1);
                    nMeasurements = atoi(argv[i+1]);
                    break;

                default :
                    std::cout << "Parameter \033[1;31m" << argv[i] << "\033[0m is not valid!\n"
                        << "Use parameter -h or -H for help." << std::endl;
                    break;
            }
        }
    }
    // All parameters parsed

    std::cout << "Number of measuremetns taken: " << nMeasurements << "\n"
        << "Source file: " << dataFile
        << std::endl;

    // Load data
    LaserDataLoader loader(dataFile, nMeasurements, "FLASER");
    LaserScan scan;
    RawPoints scanPoints;
    Measurement measurement;
    RobotPosition pos;

    // Load map as a grid
    vtkSmartPointer<vtkPNGReader> reader =  vtkSmartPointer<vtkPNGReader>::New();
    reader->SetFileName("../data/belgioioso-map5b.png");
    imr::LaserSimulator simul(reader);
    Gui gui(/*initial,tentative,robotPosition2point(loader[0].position),*/reader);
    std::cout << "X: " << simul.grid2realX(0) << " " << simul.grid2realX(1872) << std::endl;
    std::cout << "Y: " << simul.grid2realY(0) << " " << simul.grid2realY(5015) << std::endl;
    Particle p;
    ParticleVector particles;
    double x,y, phi;
    for(size_t i=0;i<COUNT_PARTICLE;) {
      x = (rand() % 3669 - 1696)/100.0;
      y = (rand() % 9828 - 4325)/100.0;
      phi = (rand() % 31415)/1000;
      p.pos = RobotPosition(x,y,phi);
      if (simul.isFeasible(p.pos)) {
        p.weight = 1.0/COUNT_PARTICLE;
        particles.push_back(p);
        i++;
      }
    }

    clock_t begin = clock();
    for(int i=0; i<nMeasurements; i++) {
//       double angle = 8.0/180.0*M_PI;
//       pos.x = loader[i].position.x*cos(angle) - loader[i].position.y*sin(angle);
//       pos.y = loader[i].position.x*sin(angle) + loader[i].position.y*cos(angle);
//       pos.phi = loader[i].position.phi + angle;
         pos = loader[i].position;
//         std::cout << "POSE " << pos.x << " " << pos.y << std::endl;
      scan = simul.getScan(pos);
      scanPoints = simul.getRawPoints();
      // gui.clearPositionPoints();

      gui.setPosition(robotPosition2point(pos));
      gui.clearMapPoints();
      gui.setPointsToMap(scanPoints, robotPosition2point(pos));
      gui.setParticlePoints(particles);
      gui.startInteractor();
      for(size_t k=0;k<particles.size();k++) {
        particles[k].pos.x +=0.05;
      }

    }
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "ELAPSED TIME: " << elapsed_secs << " s" << std::endl;
    gui.startInteractor();
    return EXIT_SUCCESS;
}

void WeightParticles(ParticleVector &particles, RobotPosition &pos){
    for(size_t k=0;k<particles.size();k++) {
        particles[k].+=0.05;
    }

}



/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

Point robotPosition2point(const RobotPosition &rp)
{
   return Point(rp.x,rp.y);
}

void calculateRawPoints(RawPoints &rp, Measurement m)
{
   calculateRawPoints(rp,m,m.position);
}

void calculateRawPoints(RawPoints &rp, Measurement m, RobotPosition pos)
{
   const double laserResolution = 0.5; // deg
   const double laserShift = -90.0;
   for (int j=0; j<m.scan.size(); j++) {
      if (m.scan[j] > 50.0) continue;
      Point p;
      polar2cartesian(p,toRadians(j*laserResolution+laserShift),m.scan[j],pos);
      rp.push_back(p);
   }
}

void polar2cartesian(Point &p, const double &alpha, const double &r, const RobotPosition &p0)
{
   p.x = p0.x+r*cos(alpha+p0.phi);
   p.y = p0.y+r*sin(alpha+p0.phi);
}

double toRadians(const double alpha)
{
   return (alpha*M_PI)/180.0;
}
