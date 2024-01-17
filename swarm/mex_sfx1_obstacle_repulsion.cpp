#include <mex.h>
#include <iostream>
#include <stdexcept>
#include <cstdlib>

#include "sfx1_obstacle_repulsion.hpp"
#include "utils/tracing.hpp"

struct Container
{
    // in
    //!Distance of an obstacle seen from the left [m],the definition of "left" is made by the configuration "angleLeft".
    float distanceLeft;
    //!Distance of an obstacle seen from the front [m].
    float distanceFront;
    //!Distance of an obstacle seen from the right [m],the definition of "right" is made by the configuration "angleRight".
    float distanceRight;
    //!Distance of an obstacle seen from the back [m].
    float distanceBack;

    // out
    Eigen::Quaternionf qRef;

    // conf
    //!Angle discribing the direction of left seen obstacles expressed in the body frame, [rad].
    float angleLeft;
    //!Angle discribing the direction of right seen obstacles expressed in the body frame, [rad].
    float angleRight;
    //!Minimal angle that correspond to the repulsion desired when the distance is equal to "distanceMaxToObstacle", must be in [0,2*pi] [rad].
    float angleMin;
    //!Maximal angle that correspond to the repulsion desired when the distance is equal to "distanceMinToObstacle", must be in [0,2*pi] [rad].
    float angleMax;
    //!Distance from whitch we desire to saturate the repulsion, [m].
    float distanceMinToObstacle;
    //!Distance from whitch we desire to start the repulsion, [m].
    float distanceMaxToObstacle;


    Colibry::SFObstacleRepulsionController::Controller obstacleRepulsion;

    Container();
};

Container::Container():
		obstacleRepulsion(
	                Colibry::SFObstacleRepulsionController::Input(distanceLeft, distanceFront, distanceRight, distanceBack),
	                Colibry::SFObstacleRepulsionController::Output(qRef),
	                Colibry::SFObstacleRepulsionController::Config(angleLeft, angleRight, angleMin, angleMax,
	                		distanceMinToObstacle, distanceMaxToObstacle)
	        )
{
	angleLeft = -55.0f * 3.14f/180.0f;
	angleRight = 55.0f * 3.14f/180.0f;
	angleMin = 5.0f * 3.14f/180.0f;
	angleMax = 30.0f * 3.14f/180.0f;
	distanceMinToObstacle = 1.0f;
	distanceMaxToObstacle = 3.0f;
}
Container container;
void parseTraces(mxArray* plhs[]);
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{

    if (nrhs > 0)
    {
        if(mxIsChar(prhs[0]) != 1) { throw std::invalid_argument("First argument is invalid"); }
        if(mxGetM(prhs[0]) != 1) { throw std::invalid_argument("First argument is invalid"); }

        std::string str;
        {
            std::size_t len = (mxGetN(prhs[0]));
            char* ptr = new char[len+1];

            int status = mxGetString(prhs[0], ptr, len+1);
            if(status != 0) { throw std::invalid_argument("First argument is invalid"); }
            str = std::string(ptr, len);

            delete [] ptr;
        }
        if (str == "step")
        {
            if (nrhs != 5) { throw std::invalid_argument("Not the right arguments (qRef, qEst)"); }
            //if (nlhs != 3) { throw std::invalid_argument("Not the right returned variables (tauX, tauY, tauZ)"); }
            double* ptr;
            ptr = mxGetPr(prhs[1]);
            container.distanceLeft = *(ptr+0);

            ptr = mxGetPr(prhs[2]);
            container.distanceFront = *(ptr+0);

            ptr = mxGetPr(prhs[3]);
            container.distanceRight = *(ptr+0);

            ptr = mxGetPr(prhs[4]);
            container.distanceBack = *(ptr+0);

            container.obstacleRepulsion.process();

            plhs[0] = mxCreateDoubleMatrix(4, 1, mxREAL);
            ptr = mxGetPr(plhs[0]);
            ptr[0] = container.qRef.w();
            ptr[1] = container.qRef.x();
            ptr[2] = container.qRef.y();
            ptr[3] = container.qRef.z();
        }
        else if (str == "set_gains")
      {
          std::cerr << "SET GAINS " << nrhs << std::endl;
          if (nrhs != 7) { std::cerr << "  "<< nrhs << std::endl;throw std::invalid_argument("Not the right arguments (****)"); } // { throw std::invalid_argument("Not the right arguments"); }
          double* ptr;
          ptr = mxGetPr(prhs[1]);
          container.angleLeft = *(ptr);
          ptr = mxGetPr(prhs[2]);
          container.angleRight = *(ptr);
          ptr = mxGetPr(prhs[3]);
          container.angleMin = *(ptr);
          ptr = mxGetPr(prhs[4]);
          container.angleMax = *(ptr);
          ptr = mxGetPr(prhs[5]);
          container.distanceMinToObstacle = *(ptr);
          ptr = mxGetPr(prhs[6]);
          container.distanceMaxToObstacle = *(ptr);

        }
        else if (str == "get_traces")
        {
            if (nlhs != 1) { throw std::invalid_argument("Missing returned argument (traces)"); }
            parseTraces(plhs);
        }
        else
        {
            throw std::invalid_argument("Command not recognized");
        }


    }

}

void parseTraces(mxArray* plhs[])
{
    const TracingModule::TracesType& traces = tracing().getTraces();
    // Getting variables list
    unsigned int varNum = traces.size();
    char** varNames = new char*[varNum];
    unsigned int i = 0;
    for (auto it : traces)
    {
        varNames[i] = new char [it.first.size()];
        std::strcpy(varNames[i], it.first.c_str());
        std::cerr << varNames[i]  << " " << it.second.timestamps.size() <<std::endl;
        i++;
    }
    plhs[0] = mxCreateStructMatrix(1, 1, varNum, const_cast<const char**>(varNames));

    // Filling each variable
    const char* fieldsNames[] = {"rows", "cols", "timestamps", "values"};
    i = 0;
    for (auto it : traces)
    {
        std::cerr << varNames[i]  << std::endl;
        // Metadata
        mxArray* meta = mxCreateStructMatrix(1, 1, 4, fieldsNames);
        mxSetField(meta,0, "rows", mxCreateDoubleScalar(it.second.rows));
        mxSetField(meta,0, "cols", mxCreateDoubleScalar(it.second.cols));

        // Timestamps
        mxArray* ts = mxCreateDoubleMatrix(1, it.second.timestamps.size(), mxREAL);
        std::memcpy(mxGetPr(ts), it.second.timestamps.data(), it.second.timestamps.size()*sizeof(double));
        mxSetField(meta,0, "timestamps", ts);

        // Values
        mxArray* val = mxCreateDoubleMatrix(it.second.rows*it.second.cols, it.second.values.size(), mxREAL);
        double* ptrVal = (double*)mxGetPr(val);
        for (std::size_t k=0; k<it.second.values.size(); k++)
        {
            std::memcpy(&ptrVal[k*it.second.rows*it.second.cols], it.second.values[k].data(), it.second.rows*it.second.cols*sizeof(double));
        }
        mxSetField(meta,0, "values", val);

        mxSetField(plhs[0],0, varNames[i], meta);

        i++;
    }

}
