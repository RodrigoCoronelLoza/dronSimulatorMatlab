#include <mex.h>
#include <iostream>
#include <stdexcept>
#include <cstdlib>

#include "sfx1_altitude.hpp"


struct Container
{
    // in
    float heightEst;
    float heightRef;
    float vel;

    // out
    float thrust;

    // conf
    float kp;
    float ki;
    float kd;
    float satI;
    float satO;
    Eigen::Vector2f altitudeDerivativeCommandFilterA;
    Eigen::Vector2f altitudeDerivativeCommandFilterB;

    Colibry::SFAltitudeController::Controller altitudeController;

    Container();
};

Container::Container():
            altitudeController(
                    Colibry::SFAltitudeController::Input(heightEst, heightRef, vel),
                    Colibry::SFAltitudeController::Output(thrust),
                    Colibry::SFAltitudeController::Config(kp, ki, kd, satI, satO, altitudeDerivativeCommandFilterA, altitudeDerivativeCommandFilterB)
            )
{

    kp = 7;
    kd = 5.0;
    ki = 0.001;
    satI = 4;
    satO = 8;
    altitudeDerivativeCommandFilterA(0) = 1.0f;
    altitudeDerivativeCommandFilterA(1) = -0.9892f;
    altitudeDerivativeCommandFilterB(0) = 0.0108f;
    altitudeDerivativeCommandFilterB(1) = 0.0f;

    altitudeController.init();
}

Container container;

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
            if (nrhs != 4) { throw std::invalid_argument("Not the right arguments (heightEst, heightRef, vel)"); }
            if (nlhs != 1) { throw std::invalid_argument("Not the right returned variables (thrust)"); }
            double* ptr;

            ptr = mxGetPr(prhs[1]);
            container.heightEst = *(ptr);

            ptr = mxGetPr(prhs[2]);
            container.heightRef = *(ptr);

            ptr = mxGetPr(prhs[3]);
            container.vel = *(ptr);

            container.altitudeController.process();

            plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
            ptr = mxGetPr(plhs[0]);
            ptr[0] = container.thrust;
        }
        else if (str == "set_gains")
        {
          std::cerr << "SET GAINS " << nrhs << std::endl;
          if (nrhs != 10) { std::cerr << "  "<< nrhs << std::endl;throw std::invalid_argument("Not the right arguments (****)"); } // { throw std::invalid_argument("Not the right arguments"); }
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
          container.satO = *(ptr);
          ptr = mxGetPr(prhs[6]);
          container.altitudeDerivativeCommandFilterA(0) = *(ptr);
          ptr = mxGetPr(prhs[7]);
          container.altitudeDerivativeCommandFilterA(1) = *(ptr);
          ptr = mxGetPr(prhs[8]);
          container.altitudeDerivativeCommandFilterB(0) = *(ptr);
          ptr = mxGetPr(prhs[9]);
          container.altitudeDerivativeCommandFilterB(1) = *(ptr);
          
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
