#include <mex.h>
#include <iostream>
#include <stdexcept>
#include <cstdlib>

#include "sfx1_yawRate.hpp"
#include "utils/tracing.hpp"


struct Container
{

    float yawRateRef;
    float yawRateEst;

    float tauZ;

    float kp;
    float ki;
    float kd;
    float satI;
    float tauMax;

    Colibry::SFYawRateController::Controller yawRateCtrl;

    Container();
};

Container::Container():
        yawRateCtrl(
                    Colibry::SFYawRateController::Input(
                            yawRateRef,
                            yawRateEst),
                    Colibry::SFYawRateController::Output(tauZ),
                    Colibry::SFYawRateController::Config(
                            kp,
                            ki,
                            kd,
                            satI,
                            tauMax)
                    )
{
    yawRateRef = 0.0f;
    yawRateEst = 0.0f;

    tauZ = 0.0f;

    kp = 0.25f;
    kd = 0.0f;
    ki = 0.0008f;
    satI = 0.1f;
    tauMax = 0.2f;
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
     if (nrhs != 3) { throw std::invalid_argument("Not the right arguments (yawRateRef, yawRateEst)"); }
     if (nlhs != 1) { throw std::invalid_argument("Not the right returned variables (tauZ)"); }
     double* ptr;
     ptr = mxGetPr(prhs[1]);
     container.yawRateRef = *(ptr);

     ptr = mxGetPr(prhs[2]);
     container.yawRateEst = *(ptr);

     container.yawRateCtrl.process();

     plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
     ptr = mxGetPr(plhs[0]);
     ptr[0] = container.tauZ;

      }
      else if (str == "set_gains")
      {
          std::cerr << "SET GAINS " << nrhs << std::endl;
          if (nrhs != 6) { std::cerr << "  "<< nrhs << std::endl;throw std::invalid_argument("Not the right arguments (****)"); } // { throw std::invalid_argument("Not the right arguments"); }
          double* ptr;
          ptr = mxGetPr(prhs[1]);
          container.kp = *(ptr);
          ptr = mxGetPr(prhs[2]);
          container.ki = *(ptr);
          ptr = mxGetPr(prhs[3]);
          container.kd = *(ptr);
          ptr = mxGetPr(prhs[4]);
          container.satI = *(ptr);
          ptr = mxGetPr(prhs[5]);
          container.tauMax = *(ptr);
          
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
