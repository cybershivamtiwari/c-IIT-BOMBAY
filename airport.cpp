#include <queue>
#include <string>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <vector>
#include <cassert>
#include <simplecpp>

class taxiway : public Line, public Resource{
public:
  int traversalT;
  double stepsize;
  taxiway(float xa, float ya, float xb, float yb, int trT)
    : Line(xa,ya,xb,yb), traversalT(trT),
    stepsize(sqrt(pow(xa-xb,2)+pow(ya-yb,2))/traversalT) {}
};

const int nGates = 10, nSegments = 6+3*nGates;
const int toGates = 5+nGates, fromGates = 5+2*nGates, halfRW = 5 + 3*nGates;

const int preLanding = -1, landing = 0, rightTaxiway = 1,
  leftTaxiway = toGates-2, takeOff = toGates-1;

class planeShape : public Polygon{
 public:
  planeShape(){
    double pts[14][2] = {{10, 0},
			 {0,3}, {-8,15}, {-5,3}, {-10,3},{-13,6},
			 {-13,-6},{-10,-3},{-5,-3},{-8,-15},{0,-3},{10,0}};
    reset(canvas_width()/2,canvas_height()/2, pts, 13);
    setScale(2.8);
    setFill();
    setColor(COLOR("blue"));
    //    penDown();
  }
};




class plane : public planeShape {
  int id;                // identifying number for plane
  int arrivalT;          // arrival time
  int serviceT;          // how long the aircraft waits at the gate
  int segment;           // index of taxiway segment the aircraft is on
  int timeToSegmentEnd;  // how far from the end of the segment
  int gate;              // id of allocated gate
  vector<taxiway*> &taxiways;
public:
  plane(int i, int at, int st, vector<taxiway*> &tw)
    : id(i), arrivalT(at), serviceT(st), taxiways(tw) {
    segment = preLanding; // currently before the landing runway.
    timeToSegmentEnd = 0;
    hide();
    penUp();
    gate = -10*nGates;   // indicates gate not allocated
    sim::post(arrivalT, [=](){prepareToEnterSegment();});
  }
  void prepareToEnterSegment();
  void land();
  void enter(int newsegment);
  void moveOnSegment();
  void requestGate();
  void turnToGate();
  void atGate();
  void backOnTaxiway();
  void prepareToTakeOff();
  void depart();
  void ordinarySegment();
};

void plane::prepareToEnterSegment(){
  if(segment == preLanding) land();
  else if(segment == rightTaxiway) requestGate();
  else if(segment == rightTaxiway + 1 + gate) turnToGate();
  else if(segment == toGates + gate) atGate();
  else if(segment == fromGates + gate) backOnTaxiway();
  else if(segment == leftTaxiway) prepareToTakeOff();
  else if(segment == takeOff) depart();
  else ordinarySegment();
}

void plane::land(){
  taxiways[landing]->acquire([=](){
    taxiways[halfRW]->acquire([=](){
      sim::log()<< "Plane " << id << " lands. scheduled arrival "
		<< arrivalT << ", Service time " << serviceT << endl;
      show();
      enter(landing);
    });
  });
}

void plane::enter(int newSegment){
  segment = newSegment;
  Position linestart = taxiways[segment]->getStart();
  moveTo(linestart.getX(), linestart.getY());
  Position lineend = taxiways[segment]->getEnd();
  face(lineend.getX(), lineend.getY());
  timeToSegmentEnd = taxiways[segment]->traversalT;
  sim::post(0, [=](){this->moveOnSegment();});
}

void plane::moveOnSegment(){
  if(timeToSegmentEnd != 0){
    if((segment == landing || segment == takeOff)
       && timeToSegmentEnd == taxiways[segment]->traversalT/2){
      taxiways[halfRW]->release();
    }
    forward(taxiways[segment]->stepsize);
    --timeToSegmentEnd;
    sim::post(1, [=](){moveOnSegment();});
  }
  else
    prepareToEnterSegment();
}

void plane::requestGate(){
  for(int i=0;i<nGates;++i)
    if (taxiways[toGates + i]->reserve()){
      gate = i;
      break;
    }
  if(gate < 0) sim::post(1,[=](){requestGate();});
  else taxiways[segment+1]->acquire([=](){
    taxiways[segment]->release();
    enter(segment+1);
  });
}

void plane::turnToGate(){
  taxiways[segment]->release();
  enter(toGates + gate);
}

void plane::atGate(){
  sim::log()<< " Plane " << id << " at gate " << gate
		 << " will wait for " << serviceT << endl;
  sim::post(serviceT, [=](){     // wait for service
    enter(fromGates + gate);
  });
}

void plane::backOnTaxiway(){
  taxiways[rightTaxiway + 2 + gate]->acquire([=](){
    taxiways[toGates + gate]->release();
    enter(rightTaxiway + 2 + gate);
  });
}

void plane::prepareToTakeOff(){
  taxiways[takeOff]->acquire([=](){
    taxiways[halfRW]->acquire([=](){
      taxiways[segment]->release();
      enter(segment+1);
    });
  });
}

void plane::depart(){
  taxiways[segment]->release();
  hide();
  sim::log() << " Plane " << id << " left." << endl;
  delete this;
  //  sim::post(0,[=](){delete this;});
}

void plane::ordinarySegment(){
  taxiways[segment+1]->acquire([=](){
    taxiways[segment]->release();
    enter(segment+1);
  });
}

const int RW1X1=100, RW1Y1=100, RW1X2=900, RW1Y2=300;
const int RW2X1=100, RW2Y1=300, RW2X2=900, RW2Y2=100;
const int RWXm=500, RWYm=200;

const int TWX1=950, TWY1=500, TWX2=50, TWY2=500;

const int TWYT = 600;  // level of the terminals
const int tRW = 30, tVT = 6, tMT = 5, tBT = 5;

void configure_taxiways_and_runways(vector<taxiway*> &taxiways){
  taxiways[0] = new taxiway(RW1X1,RW1Y1,RW1X2,RW1Y2,tRW); // landing runway
  taxiways[1] = new taxiway(RW1X2,RW1Y2,TWX1,TWY1,tVT);   // right taxiway

  float twXdisp = ((float)TWX2-TWX1)/(nGates+1);
  float twYdisp = ((float)TWY2-TWY1)/(nGates+1);

  for(int i=0; i<= nGates; ++i){              // main taxiway: 11 segments
    taxiways[2+i] = new taxiway(int(TWX1+i*twXdisp), int(TWY1+i*twYdisp),
				int(TWX1+(i+1)*twXdisp),
				int(TWY1+(i+1)*twYdisp), tMT);
  }
  taxiways[3+nGates] = new taxiway(TWX2,TWY2,RW2X1,RW2Y1,tVT); // left taxiway
  taxiways[4+nGates] = new taxiway(RW2X1,RW2Y1,RW2X2,RW2Y2,tRW);
                                                             // takeoff runway

  for(int i=0; i<nGates; ++i){                 // branch to gate
    taxiways[5+nGates+i] = new taxiway(int(TWX1+(i+1)*twXdisp),
				       int(TWY1+(i+1)*twYdisp),
				       int(TWX1+(i+1)*twXdisp), TWYT, tBT);
  }
  for(int i=0; i< nGates; ++i){             // branch from gate
    taxiways[5+2*nGates+i] = new taxiway(int(TWX1+(i+1)*twXdisp), TWYT,
					 int(TWX1+(i+1)*twXdisp),
					 int(TWY1+(i+1)*twYdisp), tBT);
  }
  taxiways[5+3*nGates] = new taxiway(0,0,0,0,0);  // halfRW
}

void post_plane_arrivals(vector<taxiway*> &taxiways, ifstream &planeDataFile){
  int arrivalT, serviceT;
  int id = 1;                 // identifier for each plane
  beginFrame();
  while(planeDataFile >> arrivalT){
    planeDataFile >> serviceT;
    new plane(id++, arrivalT, serviceT, taxiways);
  }
  endFrame();
}

int main(int argc, char** argv){
  vector<taxiway*> taxiways(nSegments);  // including halfRW
  initCanvas("Airport Simulator",1000,1000);

  configure_taxiways_and_runways(taxiways);
  ifstream planeDataFile(argv[1]);
  post_plane_arrivals(taxiways, planeDataFile);
  sim::processAll();
  getClick();
}
