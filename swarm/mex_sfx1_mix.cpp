#include <mex.h>
#include <iostream>
#include <stdexcept>
#include <cstdlib>

#include "sfx1_mix.hpp"
#include "utils/tracing.hpp"


struct Container
{
    // in
    float tauX;
    float tauY;
    float tauZ;
    float feedForward;
    float altitude;

    // out
    bool saturatedMotors;
    uint16_t rpms[4];
    Eigen::Matrix<float, 6, 1> finalCommandDetailed;

    // conf
    float D1;
    float D2;
    float D3;
    float D4;
    float K;
    float Kt;
    float rho0;
    float rho;
    float rpmMin;
    float rpmMax;
    float pwmMax;
    float tauZMin;

    Colibry::SFMixController::Controller mix;

    Container();
};

Container::Container():
    mix(
    Colibry::SFMixController::Input(tauX, tauY, tauZ, feedForward, altitude),
    Colibry::SFMixController::Output(saturatedMotors, rpms, finalCommandDetailed),
    Colibry::SFMixController::Config(D1, D2, D3, D4, K, Kt, rho0, rho, rpmMin, rpmMax, pwmMax, tauZMin)
    )
{
    // D1 = 0.151;
    // D2 = 0.111;
    // D3 = 0.250;
    // D4 = 0.180;
    // K = 0.000015811;
    // Kt = 0.00000024562;

    D1 = 0.131;
    D2 =  0.131;
    D3 = 0.25029;
    D4 = 0.18029;
    K = 0.000013216;
    Kt = 0.000000247;
    rho0 = 1.096323134727160;
    rho = 1.096323134727160;

    rpmMin =  1800.0;
    rpmMax =  7000.0;
    pwmMax =  255.0;
    tauZMin = 0.0;

    // D1 =108.75e-3;
    // D2 =132.22e-3;
    // D3 = 224.27e-3;
    // D4 = 130.45e-3;
    // K = 9.0365e-6;
    // Kt = 1.2324e-7;
    // rpmMin = 0;
    // rpmMax = 8000;


    mix.init();
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
     if (nrhs != 6) { throw std::invalid_argument("Not the right arguments (tauX, tauY, tauZ, ff, altitude)"); }
     if (nlhs != 3) { throw std::invalid_argument("Not the right returned variables (rpms, bool, finalDetailed)"); }
     double* ptr;
     ptr = mxGetPr(prhs[1]);
     container.tauX = *(ptr);

     ptr = mxGetPr(prhs[2]);
     container.tauY = *(ptr);

     ptr = mxGetPr(prhs[3]);
     container.tauZ = *(ptr);

     ptr = mxGetPr(prhs[4]);
     container.feedForward = *(ptr);

     ptr = mxGetPr(prhs[5]);
     container.altitude = *(ptr);

     container.mix.process();

     plhs[0] = mxCreateDoubleMatrix(4, 1, mxREAL);
     ptr = mxGetPr(plhs[0]);
     ptr[0] = container.rpms[0];
     ptr[1] = container.rpms[1];
     ptr[2] = container.rpms[2];
     ptr[3] = container.rpms[3];

     plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
     ptr = mxGetPr(plhs[1]);
     ptr[0] = container.saturatedMotors;

     plhs[2] = mxCreateDoubleMatrix(6, 1, mxREAL);
     ptr = mxGetPr(plhs[2]);
     ptr[0] = container.finalCommandDetailed(0);
     ptr[1] = container.finalCommandDetailed(1);
     ptr[2] = container.finalCommandDetailed(2);
     ptr[3] = container.finalCommandDetailed(3);
     ptr[4] = container.finalCommandDetailed(4);
     ptr[5] = container.finalCommandDetailed(5);

      }
      else if (str == "set_gains")
      {
          std::cerr << "SET GAINS " << nrhs << std::endl;
          if (nrhs != 13) { std::cerr << "  "<< nrhs << std::endl;throw std::invalid_argument("Not the right arguments (****)"); } // { throw std::invalid_argument("Not the right arguments"); }
          double* ptr;
          ptr = mxGetPr(prhs[1]);
          container.D1 = *(ptr);
          ptr = mxGetPr(prhs[2]);
          container.D2 = *(ptr);
          ptr = mxGetPr(prhs[3]);
          container.D3 = *(ptr);
          ptr = mxGetPr(prhs[4]);
          container.D4 = *(ptr);
          ptr = mxGetPr(prhs[5]);
          container.K = *(ptr);
          ptr = mxGetPr(prhs[6]);
          container.Kt = *(ptr);
          ptr = mxGetPr(prhs[7]);
          container.rho0 = *(ptr);
          ptr = mxGetPr(prhs[8]);
          container.rho = *(ptr);
          ptr = mxGetPr(prhs[9]);
          container.rpmMin = *(ptr);
          ptr = mxGetPr(prhs[10]);
          container.rpmMax = *(ptr);
          ptr = mxGetPr(prhs[11]);
          container.pwmMax = *(ptr);
          ptr = mxGetPr(prhs[12]);
          container.tauZMin = *(ptr);

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
