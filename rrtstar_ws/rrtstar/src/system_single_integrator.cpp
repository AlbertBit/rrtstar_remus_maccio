#include "system_single_integrator.h"
#include <cmath>
#include <cstdlib>

#include <iostream>

using namespace std;
using namespace SingleIntegrator;

#define DISCRETIZATION_STEP 0.01


region::region () {

    numDimensions = 0;

    center = NULL;
    size = NULL;
}


region::~region () {

    if (center)
        delete [] center;
    if (size)
        delete [] size;

}


int region::setNumDimensions (int numDimensionsIn) {

    numDimensions = numDimensionsIn;

    if (center)
        delete [] center;
    center = new double[numDimensions];

    if (size)
        delete [] size;
    size = new double[numDimensions];

    return 1;

}


State::State () {

    numDimensions = 0;

    center = NULL;
    size = NULL;
}


State::~State () {

    if (center)
        delete [] center;
    if (size)
        delete [] size;
}


State::State (const State &stateIn) {

	//clock_t start = clock();

    numDimensions = stateIn.numDimensions;

    if (numDimensions > 0) {
        center = new double[numDimensions];
        size = new double[numDimensions];

        for (int i = 0; i < numDimensions; i++) {
            center[i] = stateIn.center[i];
            size[i] = stateIn.size[i];
        }

    }
    else {
        center = NULL;
        size = NULL;
    }
	//clock_t finish = clock();
	//cout << "Time : " << ((double)(finish-start)) << endl;
}


State& State::operator=(const State &stateIn){

    if (this == &stateIn)
        return *this;

    if (numDimensions != stateIn.numDimensions) {
        if (center)
            delete [] center;
        if (size)
            delete [] size;
        numDimensions = stateIn.numDimensions;
        if (numDimensions > 0) {
            center = new double[numDimensions];
            size = new double[numDimensions];
        }

    }

    for (int i = 0; i < numDimensions; i++) {
        center[i] = stateIn.center[i];
        size[i] = stateIn.size[i];
    }


    return *this;
}


int State::setNumDimensions (int numDimensionsIn) {

    if (center)
        delete [] center;
    if (size)
        delete [] size;

    if (numDimensions < 0)
        return 0;

    numDimensions = numDimensionsIn;

    if (numDimensions > 0) {
        center = new double[numDimensions];
        size = new double[numDimensions];
    }

    return 1;
}


Trajectory::Trajectory () {

    endState = NULL;
}


Trajectory::~Trajectory () {

    if (endState)
        delete endState;
}


Trajectory::Trajectory (const Trajectory &trajectoryIn) {

    endState = new State (trajectoryIn.getEndState());

}


Trajectory& Trajectory::operator=(const Trajectory &trajectoryIn) {

    if (this == &trajectoryIn)
        return *this;

    if (endState)
        delete endState;


    endState = new State (trajectoryIn.getEndState());

    totalVariation = trajectoryIn.totalVariation;

    return *this;
}


double Trajectory::evaluateCost () {

    return totalVariation;
}


System::System () {

    numDimensions = 0;
}


System::~System () {

}


int System::setNumDimensions (int numDimensionsIn) {

    if (numDimensions < 0)
        return 0;

    numDimensions = numDimensionsIn;

    rootState.setNumDimensions (numDimensions);

    return 1;
}


int System::getStateKey (State &stateIn, double* stateKey) {

    for (int i = 0; i < numDimensions; i++)
        stateKey[i] =  stateIn.center[i] / regionOperating.size[i];

    return 1;
}



bool System::isReachingTarget (State &stateIn) {


    for (int i = 0; i < numDimensions; i++) {

        if (fabs(stateIn.center[i] - regionGoal.center[i]) > regionGoal.size[i]/2.0 )
            return false;
    }

    return true;
}

bool System::IsInCollision (State &stateIn) {

	 for (list<region*>::iterator iter = obstacles.begin(); iter != obstacles.end(); iter++) {

        region *obstacleCurr = *iter;
        bool collisionFound = true;

        for( int i = 0; i < numDimensions; i++) {
            if (fabs(obstacleCurr->center[i]-stateIn.center[i]) > (rootState.size[i]/2.0 + obstacleCurr->size[i]/2.0) ) {
                collisionFound = false;

                break;
            }
        }


        if (collisionFound) {
	               return true;
        }

    }
	return false;
}

int System::sampleState (State &randomStateOut) {

    //clock_t start = clock();
    randomStateOut.setNumDimensions (numDimensions);

    for (int i = 0; i < numDimensions; i++) {

        randomStateOut.center[i] = ( (double)rand()/(RAND_MAX + 1.0) * (regionOperating.size[i] - rootState.size[i]) )
            - regionOperating.size[i]/2.0  + rootState.size[i]/2.0 + regionOperating.center[i];
    }

    if (IsInCollision (randomStateOut)) {
        /*clock_t finish = clock();
        sample_state_time += finish - start;
        failing_counter++;*/
        return 0;
    }


    /*clock_t finish = clock();
    sample_state_time += finish - start;*/

    return 1;
}



int System::extendTo (State &stateFromIn, State &stateTowardsIn, Trajectory &trajectoryOut, bool &exactConnectionOut) {

    //clock_t start = clock();
    double *dists = new double[numDimensions];
    for (int i = 0; i < numDimensions; i++)
        dists[i] = stateTowardsIn.center[i] - stateFromIn.center[i];

    double distTotal = 0.0;
    for (int i = 0; i < numDimensions; i++)
        distTotal += dists[i]*dists[i];
    distTotal = sqrt (distTotal);

    double incrementTotal = distTotal/DISCRETIZATION_STEP;

    // normalize the distance according to the disretization step
    for (int i = 0; i < numDimensions; i++)
        dists[i] /= incrementTotal;

    int numSegments = (int)floor(incrementTotal);

    //clock_t start = clock();
    State stateCurr = stateFromIn;


    for (int i = 0; i < numSegments; i++) {

        if (IsInCollision (stateCurr)) {
            /*clock_t finish = clock();
            extend_to_time += finish - start;*/

            return 0;
        }


        for (int i = 0; i < numDimensions; i++)
            stateCurr.center[i] += dists[i];
    }


    if (IsInCollision (stateTowardsIn)) {

        /*clock_t finish = clock();
        extend_to_time += finish - start;*/
        return 0;
    }
    /*clock_t finish = clock();
    extend_to_time += finish - start;*/

    trajectoryOut.endState = new State (stateTowardsIn);
    trajectoryOut.totalVariation = distTotal;

    delete [] dists;

    exactConnectionOut = true;

    return 1;
}


double System::evaluateExtensionCost (State& stateFromIn, State& stateTowardsIn, bool &exactConnectionOut) {


    exactConnectionOut = true;

    double distTotal = 0.0;
    for (int i = 0; i < numDimensions; i++) {
        double distCurr = stateTowardsIn.center[i] - stateFromIn.center[i];
        distTotal += distCurr*distCurr;
    }

    return sqrt(distTotal);

}


int System::getTrajectory (State& stateFromIn, State& stateToIn, list<double*>& trajectoryOut) {

    double *stateArr = new double[numDimensions];
    for (int i = 0; i < numDimensions; i++)
        stateArr[i] = stateToIn.center[i];
    trajectoryOut.push_front (stateArr);

    return 1;

}


double System::evaluateCostToGo (State& stateIn) {

    double radius = 0.0;
    for (int i = 0; i < numDimensions; i++)
        radius += regionGoal.size[i] * regionGoal.size[i];
    radius = sqrt(radius);

    double dist = 0.0;
    for (int i = 0; i < numDimensions; i++)
        dist += (stateIn.center[i] - regionGoal.center[i])*(stateIn.center[0] - regionGoal.center[i]);
    dist = sqrt(dist);

    return dist - radius;
}
