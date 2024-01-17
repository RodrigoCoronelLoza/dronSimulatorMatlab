#include <mex.h>
#include <iostream>
#include <stdexcept>
#include <cstdlib>

#include "sfx1_angular_rates.hpp"
#include "utils/tracing.hpp"


struct Container
{

    Eigen::Vector2f wEst;
    Eigen::Vector2f wRef;

    Eigen::Vector2f tauXY;

    float kpx;
    float kpy;
    float kix;
    float kiy;
    float kdx;
    float kdy;
    float satIx;
    float satIy;
    float maxCmd;

    Colibry::SFAngularRatesController::Controller angularRatesCtrl;

    Container();
};

Container::Container():
        angularRatesCtrl(
                    Colibry::SFAngularRatesController::Input(
                            wEst,
                            wRef),
                    Colibry::SFAngularRatesController::Output(tauXY),
                    Colibry::SFAngularRatesController::Config(
                            kpx,
                            kpy,
                            kix,
                            kiy,
                            kdx,
                            kdy,
                            satIx,
                            satIy,
                            maxCmd)
                    )
{
    wEst.setZero();
    wRef.setZero();

    tauXY.setZero();

    kpx = 0.55f;
    kpy =  0.45f;
    kdx = 0.0f;
    kdy = 0.0f;
    kix = 0.0016f;
    kiy = 0.0002f;
    satIx = 0.4f;
    satIy = 0.2f;
    maxCmd = 1.1f;
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
     if (nrhs != 3) { throw std::invalid_argument("Not the right arguments (wEst, wRef)"); }
     if (nlhs != 1) { throw std::invalid_argument("Not the right returned variables (tauXY)"); }
     double* ptr;
     ptr = mxGetPr(prhs[1]);
     container.wEst << *(ptr+0), *(ptr+1);

     ptr = mxGetPr(prhs[2]);
     container.wRef << *(ptr+0), *(ptr+1);

     container.angularRatesCtrl.process();

     plhs[0] = mxCreateDoubleMatrix(2, 1, mxREAL);
     ptr = mxGetPr(plhs[0]);
     ptr[0] = container.tauXY(0);
     ptr[1] = container.tauXY(1);

      }
      else if (str == "set_gains")
      {
          std::cerr << "SET GAINS " << nrhs << std::endl;
          if (nrhs != 10) { std::cerr << "  "<< nrhs << std::endl;throw std::invalid_argument("Not the right arguments (****)"); } // { throw std::invalid_argument("Not the right arguments"); }
          double* ptr;
          ptr = mxGetPr(prhs[1]);
          container.kpx = *(ptr);
          ptr = mxGetPr(prhs[2]);
          container.kpy = *(ptr);
          ptr = mxGetPr(prhs[3]);
          container.kix = *(ptr);
          ptr = mxGetPr(prhs[4]);
          container.kiy = *(ptr);
          ptr = mxGetPr(prhs[5]);
          container.kdx = *(ptr);
          ptr = mxGetPr(prhs[6]);
          container.kdy = *(ptr);
          ptr = mxGetPr(prhs[7]);
          container.satIx = *(ptr);
          ptr = mxGetPr(prhs[8]);
          container.satIy = *(ptr);
          ptr = mxGetPr(prhs[9]);
          container.maxCmd = *(ptr);

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
