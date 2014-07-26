#include <yarp/sig/Image.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Vector.h>

#include <cv.h>
#include <highgui.h>

#include <iostream>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <cstdio>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

class MyModule:public RFModule
{
//     Port handlerPort; //a port to handle messages
    BufferedPort<ImageOf<PixelRgb>  >   imageInL;
//     BufferedPort<Bottle>                targetPort;
    BufferedPort<ImageOf<PixelMono>  >  imageOutL;
    BufferedPort<ImageOf<PixelRgb>  >   imageInR;
    BufferedPort<ImageOf<PixelMono>  >  imageOutR;
    cv::Mat KPimgL;
    cv::Mat KPimgR;
    cv::Mat imageL1;
    cv::Mat imageL2;
    cv::Mat imageR1;
    cv::Mat imageR2;
public:

    double getPeriod()
    {
        return 0.1; //module periodicity (seconds)
    }

    
    void imageAcq()
    {
      if (imageL1.empty() and imageR1.empty()){
	imageL1 = (IplImage *) imageInL.read(true)->getIplImage();
	imageR1 = (IplImage *) imageInR.read(true)->getIplImage();
	imageL2 = (IplImage *) imageInL.read(true)->getIplImage();
	imageR2 = (IplImage *) imageInR.read(true)->getIplImage();
      }
      else{
	imageL1 = imageL2;
	imageR1 = imageR2;
	imageL2 = (IplImage *) imageInL.read(true)->getIplImage();
	imageR2 = (IplImage *) imageInR.read(true)->getIplImage();
      }
      
    }    
    bool updateModule()
    {
        if (imageInL.getInputCount()>0 and imageInR.getInputCount()>0)
        {
            ImageOf<PixelMono> &outImgL = imageOutL.prepare();
            ImageOf<PixelMono> &outImgR = imageOutR.prepare();
	    imageAcq();
            KPimgL = harrisCorner(imageL2);
	    KPimgR = harrisCorner(imageR2);
	    cv::Mat LKimgL;
	    cv::Mat LKimgR;
	    cv::Mat Status;
	    cv::Mat Error;
	    cv::calcOpticalFlowPyrLK(imageL2,imageL1,KPimgL,LKimgL, Status, Error);
	    
//             Bottle &target=targetPort.prepare();
//             target.clear();
            

//             targetPort.write();
        }
        return true;
    }
    
    cv::Mat harrisCorner(cv::Mat src)
    {
      int thresh = 200;
      int max_thresh = 255;
      cv::cvtColor( src, src, CV_BGR2GRAY );
      cv::Mat dst, dst_norm, dst_norm_scaled;
      dst = cv::Mat::zeros(src.size(), CV_32FC1 );

      /// Detector parameters
      int blockSize = 2;
      int apertureSize = 3;
      double k = 0.04;

      /// Detecting corners
      cv::cornerHarris( src, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT );

      /// Normalizing
      cv::normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
      cv::convertScaleAbs( dst_norm, dst_norm_scaled );
      return dst_norm;
    }

    bool respond(const Bottle& command, Bottle& reply) 
    {
        cout<<"Got something, echo is on"<<endl;
        if (command.get(0).asString()=="quit")
            return false;
        else
            reply=command;
        return true;
    }

    /* 
    * Configure function. Receive a previously initialized
    * resource finder object. Use it to configure your module.
    * Open port and attach it to message handler.
    */
    bool configure(yarp::os::ResourceFinder &rf)
    {
        string moduleName = "objectDetector";

//         string handleportString = "/" + moduleName;
//         handlerPort.open(handleportString.c_str());

        string inputNameLImage = "/" + moduleName + "/Limage:i";
        imageInL.open( inputNameLImage.c_str() );

        string outputNameLImage = "/" + moduleName + "/imageL:o";
        imageOutL.open( outputNameLImage.c_str() );

	string inputNameRImage = "/" + moduleName + "/Rimage:i";
        imageInR.open( inputNameRImage.c_str() );

        string outputNameRImage = "/" + moduleName + "/imageR:o";
        imageOutR.open( outputNameRImage.c_str() );
	
//         string outputNameTarget = "/" + moduleName + "/target:o";
//         targetPort.open(outputNameTarget.c_str());

//         attach(handlerPort);

        return true;
    }

    /*
    * Interrupt function.
    */
    bool interruptModule()
    {
        imageInL.interrupt();
	imageInR.interrupt();
        imageOutL.interrupt();
	imageOutR.interrupt();
        cout<<"Interrupting your module, for port cleanup"<<endl;
        return true;
    }

    /*
    * Close function, to perform cleanup.
    */
    bool close()
    {
        cout<<"Calling close function\n";
//         handlerPort.close();
	imageInL.close();
	imageInR.close();
        imageOutL.close();
	imageOutR.close();
//         targetPort.close();
        
        return true;
    }
};

int main(int argc, char * argv[])
{
    Network yarp;

    MyModule module;
    ResourceFinder rf;
    rf.configure(argc, argv);
    rf.setVerbose(true);

    cout<<"Configure module..."<<endl;
    module.configure(rf);
    cout<<"Start module..."<<endl;
    module.runModule();

    cout<<"Main returning..."<<endl;
    return 0;
}



