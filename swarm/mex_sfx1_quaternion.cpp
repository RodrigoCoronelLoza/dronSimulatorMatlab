#include <mex.h>
#include <iostream>
#include <stdexcept>
#include <cstdlib>

#include "sfx1_quaternion.hpp"
#include "utils/tracing.hpp"

struct Container
{
    // in
    Eigen::Quaternionf qRef;
    Eigen::Quaternionf qEst;

    // out
    float tauX;
    float tauY;
    float tauZ;

    // conf
    float kpx;
    float kpy;
    float kpz;
    float kix;
    float kiy;
    float kiz;
    float kdx;
    float kdy;
    float kdz;
    float satIx;
    float satIy;
    float satIz;

    Colibry::SFQuaternionController::Mode mode;
    float cmdMax;

    Colibry::SFQuaternionController::Controller quaternionController;

    Container();
};

Container::Container():
	        quaternionController(
	                Colibry::SFQuaternionController::Input(qRef, qEst),
	                Colibry::SFQuaternionController::Output(tauX, tauY, tauZ),
	                Colibry::SFQuaternionController::Config(kpx, kpy, kpz, kix, kiy, kiz, kdx, kdy, kdz, satIx, satIy, satIz, mode, cmdMax)
	        )
{
    kpx = 6.5;
    kpy = 4.5;
    kpz = 0;
    kdx = 0;
    kdy = 0;
    kdz = 0;
    kix = 0.006;
    kiy = 0.006;
    kiz = 0;
    satIx = 0.6;
    satIy = 0.6;
    satIz = 0;
    cmdMax = 4.5;
    mode = Colibry::SFQuaternionController::Mode::PITCH_ROLL;
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
            if (nrhs != 3) { throw std::invalid_argument("Not the right arguments (qRef, qEst)"); }
            if (nlhs != 3) { throw std::invalid_argument("Not the right returned variables (tauX, tauY, tauZ)"); }
            double* ptr;
            ptr = mxGetPr(prhs[1]);
            container.qRef.w() = *(ptr+0);
            container.qRef.x() = *(ptr+1);
            container.qRef.y() = *(ptr+2);
            container.qRef.z() = *(ptr+3);

            ptr = mxGetPr(prhs[2]);
            container.qEst.w() = *(ptr+0);
            container.qEst.x() = *(ptr+1);
            container.qEst.y() = *(ptr+2);
            container.qEst.z() = *(ptr+3);

            container.quaternionController.process();

            plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
            ptr = mxGetPr(plhs[0]);
            ptr[0] = container.tauX;

            plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
            ptr = mxGetPr(plhs[1]);
            ptr[0] = container.tauY;

            plhs[2] = mxCreateDoubleMatrix(1, 1, mxREAL);
            ptr = mxGetPr(plhs[2]);
            ptr[0] = container.tauZ;
        }
        else if (str == "set_gains")
      {
          std::cerr << "SET GAINS " << nrhs << std::endl;
          if (nrhs != 13) { std::cerr << "  "<< nrhs << std::endl;throw std::invalid_argument("Not the right arguments (****)"); } // { throw std::invalid_argument("Not the right arguments"); }
          double* ptr;
          ptr = mxGetPr(prhs[1]);
          container.kpx = *(ptr);
          ptr = mxGetPr(prhs[2]);
          container.kpy = *(ptr);
          ptr = mxGetPr(prhs[3]);
          container.kpz = *(ptr);
          ptr = mxGetPr(prhs[4]);
          container.kix = *(ptr);
          ptr = mxGetPr(prhs[5]);
          container.kiy = *(ptr);
          ptr = mxGetPr(prhs[6]);
          container.kiz = *(ptr);
          ptr = mxGetPr(prhs[7]);
          container.kdx = *(ptr);
          ptr = mxGetPr(prhs[8]);
          container.kdy = *(ptr);
          ptr = mxGetPr(prhs[9]);
          container.kdz = *(ptr);
          ptr = mxGetPr(prhs[10]);
          container.satIx = *(ptr);
          ptr = mxGetPr(prhs[11]);
          container.satIy = *(ptr);
          ptr = mxGetPr(prhs[12]);
          container.satIz = *(ptr);

        }
        else if (str == "get_traces")
        {
            if (nlhs != 1) { throw std::invalid_argument("Missing returned argument (traces)"); }
            parseTraces(plhs);
        }
        // else if (str == "get_traces")
        // {
        //      if (nlhs != 1) { throw std::invalid_argument("Missing returned argument (traces)"); }
        //      parseTraces(plhs);
        // }      else if (str == "clear")
        // {
        //     // if (nrhs != 2) { throw std::invalid_argument("Missing second argument (flightPlan)"); }

        //     std::cerr << "[Wrapper] Clear flighPlan "<< std::endl;
        //     global.colibryEmulation->fp.clear();

        // }
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
